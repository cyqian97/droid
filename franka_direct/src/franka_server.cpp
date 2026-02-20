// franka_server.cpp
//
// Lightweight gRPC server that drives a Franka robot with libfranka's
// built-in Cartesian impedance controller.
//
// Architecture:
//   - Main thread:   libfranka robot.control() 1 kHz RT loop (no gRPC inside)
//   - gRPC threads:  handle SetCartesianTarget / GetRobotState at any rate
//   - Shared state:  a mutex-protected struct exchanged between both sides
//
// Waypoint interpolation:
//   The policy runs at ~25 Hz and sends sparse Cartesian pose targets.  The
//   RT loop linearly interpolates between the last received goal and the new
//   one over N = ceil(1000 / policy_hz) ticks so the Franka motion generator
//   always receives smooth, closely-spaced targets.
//
//   When a new goal arrives mid-segment the interpolation restarts from the
//   current interpolated pose — the arm never jumps.
//
// Usage:
//   ./franka_server <robot_ip> <grpc_listen_addr> [policy_hz]
//   e.g. ./franka_server 192.168.1.11 0.0.0.0:50052 25

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
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

    // Written by gRPC SetCartesianTarget; read by RT loop each tick.
    std::array<double, 16> goal_pose{};  // latest waypoint from Python
    uint64_t               goal_seq{0};  // incremented on each SetCartesianTarget

    // Written by RT loop; read by gRPC GetRobotState.
    std::array<double, 16> current_pose{};  // O_T_EE — actual measured pose
    std::array<double, 16> target_pose{};   // current interpolated command (≠ goal_pose)
    std::array<double, 7>  current_q{};
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

    Status SetCartesianTarget(ServerContext*,
                              const franka_control::CartesianTarget* req,
                              franka_control::CommandResult* rep) override {
        if (req->pose_size() != 16) {
            rep->set_success(false);
            rep->set_message("Expected exactly 16 doubles for O_T_EE pose");
            return Status::OK;
        }
        {
            std::lock_guard<std::mutex> lk(s_.mtx);
            for (int i = 0; i < 16; ++i)
                s_.goal_pose[i] = req->pose(i);
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
        // target_pose = current interpolated command, useful as baseline for
        // the Python client's next SetCartesianTarget call.
        for (double v : s_.target_pose)  rep->add_target_pose(v);
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

    robot.setCollisionBehavior(
        {{40,40,40,40,40,40,40}}, {{40,40,40,40,40,40,40}},
        {{40,40,40,40,40,40}},    {{40,40,40,40,40,40}});

    // ── libfranka Cartesian pose control loop (with auto-recovery) ──────────
    //
    // The callback runs at 1 kHz.  It does NOT call gRPC — it only reads
    // goal_pose / goal_seq from SharedState (mutex, fast) and does arithmetic.
    //
    // Interpolation state (RT-private, no mutex needed):
    //   interp_start  – pose at the beginning of the current segment
    //   interp_goal   – pose at the end of the current segment (= last goal_pose)
    //   interp_pose   – current interpolated command, sent to the robot
    //   interp_tick   – ticks elapsed in this segment  [0 .. interp_N]
    //   last_goal_seq – goal_seq value when interp_goal was last set
    //
    while (!state.stop) {
        bool control_initialized = false;

        // RT-private interpolation state — reset on each outer loop iteration.
        std::array<double, 16> interp_start{};
        std::array<double, 16> interp_goal{};
        std::array<double, 16> interp_pose{};
        int      interp_tick    = 0;
        uint64_t last_goal_seq  = 0;

        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::CartesianPose
                {
                    // ── Lock shared state (minimal critical section) ───────
                    std::lock_guard<std::mutex> lk(state.mtx);

                    // ── Initialise on first callback ───────────────────────
                    if (!control_initialized) {
                        // Seed everything from the robot's current commanded
                        // pose so there is zero discontinuity at startup.
                        interp_start = rs.O_T_EE_d;
                        interp_goal  = rs.O_T_EE_d;
                        interp_pose  = rs.O_T_EE_d;
                        interp_tick  = interp_N;   // already "arrived"

                        // Publish goal_pose so Python can read a valid baseline
                        // from GetRobotState before sending the first target.
                        state.goal_pose   = rs.O_T_EE_d;
                        state.target_pose = rs.O_T_EE_d;
                        last_goal_seq     = state.goal_seq;

                        state.ready = true;
                        control_initialized = true;
                        std::cout << "[franka_server] Control loop started. EE z = "
                                  << rs.O_T_EE[14] << " m" << std::endl;
                    }

                    // ── Detect new waypoint from Python ───────────────────
                    if (state.goal_seq != last_goal_seq) {
                        // A new goal has arrived.  Start a fresh segment from
                        // wherever we currently are (interp_pose), so the arm
                        // never jumps.
                        interp_start   = interp_pose;
                        interp_goal    = state.goal_pose;
                        interp_tick    = 0;
                        last_goal_seq  = state.goal_seq;
                    }

                    // ── Advance linear interpolation ──────────────────────
                    // interp_tick counts ticks elapsed in the current segment.
                    // Clamp at interp_N so the arm holds the goal pose once
                    // reached (until the next waypoint arrives).
                    if (interp_tick < interp_N) {
                        ++interp_tick;
                    }
                    const double alpha = static_cast<double>(interp_tick) / interp_N;
                    for (int i = 0; i < 16; ++i) {
                        interp_pose[i] = interp_start[i]
                                         + alpha * (interp_goal[i] - interp_start[i]);
                    }

                    // ── Update telemetry ──────────────────────────────────
                    state.current_pose     = rs.O_T_EE;
                    state.current_q        = rs.q;
                    state.cmd_success_rate = rs.control_command_success_rate;
                    // Publish the interpolated command so Python's
                    // GetRobotState("target_pose") sees where the arm is
                    // actually heading this tick.
                    state.target_pose = interp_pose;

                    // ── Finish motion if stop requested ───────────────────
                    if (state.stop) {
                        franka::CartesianPose p(interp_pose);
                        p.motion_finished = true;
                        return p;
                    }

                    return franka::CartesianPose(interp_pose);
                },
                franka::ControllerMode::kJointImpedance,
                true,   // limit_rate
                100.0   // cutoff_frequency [Hz]
            );

        } catch (const franka::ControlException& e) {
            std::cerr << "[franka_server] Control exception: " << e.what() << std::endl;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
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
