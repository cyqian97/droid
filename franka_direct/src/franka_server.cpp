// franka_server.cpp
//
// Lightweight gRPC server that drives a Franka robot with libfranka's
// joint position controller.
//
// Architecture:
//   - Main thread:   libfranka robot.control() 1 kHz RT loop (no gRPC inside)
//   - gRPC threads:  handle SetJointTarget / GetRobotState at any rate
//   - Shared state:  a mutex-protected struct exchanged between both sides
//
// Waypoint interpolation:
//   The policy runs at ~25 Hz and sends sparse joint position targets.  The
//   RT loop linearly interpolates between the last received goal and the new
//   one over N = ceil(1000 / policy_hz) ticks so the joint position controller
//   always receives smooth, closely-spaced targets.
//
//   When a new goal arrives mid-segment the interpolation restarts from the
//   current interpolated position — the arm never jumps.
//
// Usage:
//   ./franka_server <robot_ip> <grpc_listen_addr> [policy_hz]
//   e.g. ./franka_server 192.168.1.11 0.0.0.0:50052 25

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include <grpcpp/grpcpp.h>
#include "franka_control.grpc.pb.h"
#include "franka_control.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

// ── Shared state between gRPC threads and the RT control loop ──────────────

struct SharedState {
    std::mutex mtx;

    // Written by gRPC SetJointTarget; read by RT loop each tick.
    std::array<double, 7> goal_q{};   // latest joint target from Python
    uint64_t              goal_seq{0}; // incremented on each SetJointTarget

    // Written by RT loop; read by gRPC GetRobotState.
    std::array<double, 16> current_pose{};  // O_T_EE — actual measured pose
    std::array<double, 7>  current_q{};     // actual measured joint positions
    std::array<double, 7>  target_q{};      // current interpolated joint command
    double cmd_success_rate{0.0};
    bool   ready{false};
    std::string error{};

    // Atomic flags — safe to read/write without mutex.
    std::atomic<bool> stop{false};
};

// ── gRPC service implementation ─────────────────────────────────────────────

class FrankaControlImpl final : public franka_control::FrankaControl::Service {
public:
    explicit FrankaControlImpl(SharedState& s) : s_(s) {}

    Status SetJointTarget(ServerContext*,
                          const franka_control::JointTarget* req,
                          franka_control::CommandResult* rep) override {
        if (req->q_size() != 7) {
            rep->set_success(false);
            rep->set_message("Expected exactly 7 doubles for joint positions");
            return Status::OK;
        }
        {
            std::lock_guard<std::mutex> lk(s_.mtx);
            for (int i = 0; i < 7; ++i)
                s_.goal_q[i] = req->q(i);
            ++s_.goal_seq;
        }
        rep->set_success(true);
        return Status::OK;
    }

    Status GetRobotState(ServerContext*,
                         const franka_control::Empty*,
                         franka_control::RobotState* rep) override {
        std::lock_guard<std::mutex> lk(s_.mtx);
        for (double v : s_.current_pose) rep->add_pose(v);
        for (double v : s_.current_q)    rep->add_q(v);
        // target_q = current interpolated joint command, useful as baseline for
        // the Python client's next SetJointTarget call.
        for (double v : s_.target_q)     rep->add_target_q(v);
        rep->set_cmd_success_rate(s_.cmd_success_rate);
        rep->set_ready(s_.ready);
        rep->set_error(s_.error);
        return Status::OK;
    }

    Status Stop(ServerContext*,
                const franka_control::Empty*,
                franka_control::CommandResult* rep) override {
        s_.stop = true;
        rep->set_success(true);
        rep->set_message("Stop requested");
        return Status::OK;
    }

private:
    SharedState& s_;
};

// ── gRPC server launcher (runs in background thread) ───────────────────────

static std::unique_ptr<Server> g_server;

void run_grpc_server(SharedState& state, const std::string& addr) {
    FrankaControlImpl service(state);
    ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    g_server = builder.BuildAndStart();
    std::cout << "[franka_server] gRPC listening on " << addr << std::endl;
    g_server->Wait();
}

// ── Signal handler ──────────────────────────────────────────────────────────

static SharedState* g_state_ptr = nullptr;

void signal_handler(int) {
    std::cout << "\n[franka_server] SIGINT received, stopping..." << std::endl;
    if (g_state_ptr) g_state_ptr->stop = true;
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const std::string robot_ip   = (argc > 1) ? argv[1] : "192.168.1.11";
    const std::string grpc_addr  = (argc > 2) ? argv[2] : "0.0.0.0:50052";
    const double      policy_hz  = (argc > 3) ? std::stod(argv[3]) : 25.0;

    // Number of 1 kHz ticks to spend travelling from one waypoint to the next.
    // At 25 Hz: N = 40; increase if the policy runs slower.
    const int interp_N = static_cast<int>(std::ceil(1000.0 / policy_hz));

    std::cout << "[franka_server] policy_hz=" << policy_hz
              << "  interp_N=" << interp_N << " ticks/waypoint" << std::endl;

    SharedState state;
    g_state_ptr = &state;
    std::signal(SIGINT, signal_handler);

    // ── Start gRPC server in background thread ──────────────────────────────
    std::thread grpc_thread([&]() { run_grpc_server(state, grpc_addr); });
    grpc_thread.detach();

    // ── Connect to robot ────────────────────────────────────────────────────
    std::cout << "[franka_server] Connecting to robot at " << robot_ip << " ..." << std::endl;
    franka::Robot robot(robot_ip);
    std::cout << "[franka_server] Connected." << std::endl;

    // ── libfranka joint position control loop (with auto-recovery) ──────────
    //
    // The callback runs at 1 kHz.  It does NOT call gRPC — it only reads
    // goal_q / goal_seq from SharedState (mutex, fast) and does arithmetic.
    //
    // Interpolation state (RT-private, no mutex needed):
    //   interp_start  – joints at the beginning of the current segment
    //   interp_goal   – joints at the end of the current segment (= last goal_q)
    //   interp_q      – current interpolated command, sent to the robot
    //   interp_tick   – ticks elapsed in this segment  [0 .. interp_N]
    //   last_goal_seq – goal_seq value when interp_goal was last set
    //
    while (!state.stop) {
        bool control_initialized = false;

        // Re-apply robot settings before every robot.control() call.
        // automaticErrorRecovery() resets these to factory defaults, so they
        // must be set again on each outer-loop iteration.
        // NOTE: these must be set before robot.control(), never inside the
        // 1 kHz callback (per libfranka documentation).
        robot.setCollisionBehavior(
            {{40,40,40,40,40,40,40}}, {{40,40,40,40,40,40,40}},
            {{40,40,40,40,40,40}},    {{40,40,40,40,40,40}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        // RT-private interpolation state — reset on each outer loop iteration.
        std::array<double, 7> interp_start{};
        std::array<double, 7> interp_goal{};
        std::array<double, 7> interp_q{};
        int      interp_tick   = 0;
        uint64_t last_goal_seq = 0;

        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::JointPositions
                {
                    // ── Lock shared state (minimal critical section) ───────
                    std::lock_guard<std::mutex> lk(state.mtx);

                    // ── Initialise on first callback ───────────────────────
                    if (!control_initialized) {
                        // Seed everything from the robot's current actual joint
                        // positions so there is zero discontinuity at startup.
                        interp_start = rs.q;
                        interp_goal  = rs.q;
                        interp_q     = rs.q;
                        interp_tick  = interp_N;   // already "arrived"

                        // Publish goal_q so Python can read a valid baseline
                        // from GetRobotState before sending the first target.
                        state.goal_q   = rs.q;
                        state.target_q = rs.q;
                        last_goal_seq  = state.goal_seq;

                        state.ready = true;
                        control_initialized = true;
                        std::cout << "[franka_server] Control loop started. q = ["
                                  << rs.q[0] << ", " << rs.q[1] << ", " << rs.q[2] << ", "
                                  << rs.q[3] << ", " << rs.q[4] << ", " << rs.q[5] << ", "
                                  << rs.q[6] << "]" << std::endl;
                    }

                    // ── Detect new waypoint from Python ───────────────────
                    if (state.goal_seq != last_goal_seq) {
                        // A new goal has arrived.  Start a fresh segment from
                        // wherever we currently are (interp_q), so the arm
                        // never jumps.
                        interp_start  = interp_q;
                        interp_goal   = state.goal_q;
                        interp_tick   = 0;
                        last_goal_seq = state.goal_seq;
                    }

                    // ── Advance linear interpolation ──────────────────────
                    // interp_tick counts ticks elapsed in the current segment.
                    // Clamp at interp_N so the arm holds the goal once reached
                    // (until the next waypoint arrives).
                    if (interp_tick < interp_N) {
                        ++interp_tick;
                    }
                    const double alpha = static_cast<double>(interp_tick) / interp_N;
                    for (int i = 0; i < 7; ++i) {
                        interp_q[i] = interp_start[i]
                                      + alpha * (interp_goal[i] - interp_start[i]);
                    }

                    // ── Update telemetry ──────────────────────────────────
                    state.current_pose     = rs.O_T_EE;
                    state.current_q        = rs.q;
                    state.cmd_success_rate = rs.control_command_success_rate;
                    // Publish the interpolated command so Python's
                    // GetRobotState("target_q") sees where the arm is heading.
                    state.target_q = interp_q;

                    // ── Finish motion if stop requested ───────────────────
                    if (state.stop) {
                        franka::JointPositions p(interp_q);
                        p.motion_finished = true;
                        return p;
                    }

                    return franka::JointPositions(interp_q);
                }
            );

        } catch (const franka::ControlException& e) {
            std::cerr << "[franka_server] Control exception: " << e.what() << std::endl;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                const auto& cq = state.current_q;  // actual joint positions
                const auto& tq = interp_q;          // last commanded (RT-private, no mutex needed)
                std::cerr << std::fixed << std::setprecision(6)
                    << "[franka_server]   commanded (interp_q): ["
                    << tq[0] << ", " << tq[1] << ", " << tq[2] << ", "
                    << tq[3] << ", " << tq[4] << ", " << tq[5] << ", " << tq[6] << "]"
                    << "\n[franka_server]   actual    (q):        ["
                    << cq[0] << ", " << cq[1] << ", " << cq[2] << ", "
                    << cq[3] << ", " << cq[4] << ", " << cq[5] << ", " << cq[6] << "]"
                    << "\n[franka_server]   delta (cmd - actual): [";
                double max_delta = 0.0;
                for (int i = 0; i < 7; ++i) {
                    double d = tq[i] - cq[i];
                    if (std::abs(d) > max_delta) max_delta = std::abs(d);
                    std::cerr << d;
                    if (i < 6) std::cerr << ", ";
                }
                std::cerr << "]  max_delta=" << std::setprecision(4) << max_delta << " rad"
                    << "  interp_tick=" << interp_tick << "/" << interp_N
                    << "  alpha=" << static_cast<double>(interp_tick) / interp_N
                    << std::endl;
                state.ready = false;
            }
            try {
                robot.automaticErrorRecovery();
                std::cerr << "[franka_server] Recovered from error, resuming control." << std::endl;
            } catch (const franka::Exception& e2) {
                std::cerr << "[franka_server] Recovery failed: " << e2.what() << std::endl;
                std::lock_guard<std::mutex> lk(state.mtx);
                state.error = e2.what();
                state.ready = false;
                break;
            }
        } catch (const franka::Exception& e) {
            std::cerr << "[franka_server] Fatal Franka exception: " << e.what() << std::endl;
            std::lock_guard<std::mutex> lk(state.mtx);
            state.error = e.what();
            state.ready = false;
            break;
        }
    }

    std::cout << "[franka_server] Control loop exited." << std::endl;
    if (g_server) g_server->Shutdown();
    return 0;
}
