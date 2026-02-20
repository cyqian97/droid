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
// Interpolation (proportional step + velocity cap):
//   Each 1 kHz tick the RT loop computes:
//
//     step = (goal_q - rs.q) / interp_N        ← spread delta over 1 policy period
//     step = clamp(step, -max_step, +max_step)  ← hard velocity cap at 1 rad/s
//     interp_q += step
//
//   interp_N = round(1000 / policy_hz) = 40 at 25 Hz.
//
//   Normal case (goal_q advances by ~0.00175 rad every 40 ms):
//     step ≈ 0.00175 / 40 = 0.0000437 rad/tick — smooth, well within limits.
//
//   Large jump (e.g. accumulated Python goal after auto-recovery):
//     step is capped at 0.001 rad/tick (1 rad/s), approaches goal gradually.
//
// Usage:
//   ./franka_server <robot_ip> <grpc_listen_addr> [policy_hz]
//   e.g. ./franka_server 192.168.1.11 0.0.0.0:50052 25

#include <array>
#include <atomic>
#include <algorithm>
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
    std::array<double, 7>  target_q{};      // current velocity-limited command
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
        // target_q = current velocity-limited command, use as baseline for
        // the next SetJointTarget call to avoid large jumps.
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
    const std::string robot_ip  = (argc > 1) ? argv[1] : "192.168.1.11";
    const std::string grpc_addr = (argc > 2) ? argv[2] : "0.0.0.0:50052";
    const double      policy_hz = (argc > 3) ? std::stod(argv[3]) : 25.0;

    // Number of 1 kHz ticks per policy period — used to spread each waypoint
    // evenly: step_per_tick = delta / interp_N.
    const int interp_N = static_cast<int>(std::round(1000.0 / policy_hz));

    // Hard velocity cap: regardless of the computed step, never move more than
    // max_step per tick (= 1 rad/s).  Protects against large jumps after
    // auto-recovery when the accumulated Python goal is far from rs.q.
    constexpr double max_step = 0.001;  // rad per tick (= 1.0 rad/s)

    std::cout << "[franka_server] policy_hz=" << policy_hz
              << "  interp_N=" << interp_N << " ticks/waypoint"
              << "  max_step=" << max_step << " rad/tick" << std::endl;

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
    // goal_q from SharedState (mutex, fast) and does arithmetic.
    //
    // RT-private state:
    //   interp_q       – current commanded position, tracking goal_q
    //   last_goal_seq  – last seen goal_seq; detects new 25 Hz commands
    //   ticks_remaining – ticks left in the current 25 Hz command period;
    //                     reset to interp_N on each new goal, then counted
    //                     down each tick so step = d / ticks_remaining.
    //
    while (!state.stop) {
        bool control_initialized = false;

        // Re-apply robot settings before every robot.control() call.
        // automaticErrorRecovery() resets these to factory defaults, so they
        // must be set again on each outer-loop iteration.
        // NOTE: must be called before robot.control(), never inside the callback.
        robot.setCollisionBehavior(
            {{40,40,40,40,40,40,40}}, {{40,40,40,40,40,40,40}},
            {{40,40,40,40,40,40}},    {{40,40,40,40,40,40}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        // RT-private state — reset on each outer loop iteration.
        std::array<double, 7> interp_q{};  // current commanded position
        uint64_t last_goal_seq  = 0;       // detects new goals from Python
        int  ticks_remaining    = 0;       // ticks left in current 25 Hz period

        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::JointPositions
                {
                    // ── Lock shared state (minimal critical section) ───────
                    std::lock_guard<std::mutex> lk(state.mtx);

                    // ── Initialise on first callback ───────────────────────
                    if (!control_initialized) {
                        // Seed from actual robot position: zero discontinuity.
                        interp_q = rs.q;

                        // Overwrite any accumulated goal from a previous run
                        // so the first new command from Python is always just
                        // one small step ahead of the current position.
                        state.goal_q   = rs.q;
                        state.target_q = rs.q;

                        state.ready = true;
                        control_initialized = true;
                        std::cout << "[franka_server] Control loop started. q = ["
                                  << rs.q[0] << ", " << rs.q[1] << ", " << rs.q[2] << ", "
                                  << rs.q[3] << ", " << rs.q[4] << ", " << rs.q[5] << ", "
                                  << rs.q[6] << "]" << std::endl;
                    }

                    // ── Velocity-limited tracking toward goal_q ───────────
                    // When a new 25 Hz command arrives (goal_seq changed),
                    // reset ticks_remaining = interp_N.
                    //
                    // Each tick:
                    //   step = (goal - actual) / ticks_remaining
                    //        ← remaining delta spread over remaining ticks
                    //   step = clamp(step, -max_step, +max_step)
                    //   interp_q[i] += step
                    //   --ticks_remaining
                    //
                    // When ticks_remaining reaches 0 (between 25 Hz commands),
                    // hold position (step = 0).
                    if (state.goal_seq != last_goal_seq) {
                        last_goal_seq   = state.goal_seq;
                        ticks_remaining = interp_N;
                    }
                    const auto& gq = state.goal_q;
                    for (int i = 0; i < 7; ++i) {
                        double d    = gq[i] - rs.q[i];
                        double step = (ticks_remaining > 0) ? d / ticks_remaining : d;
                        interp_q[i] += std::max(-max_step, std::min(step, max_step));
                    }
                    if (ticks_remaining > 0) --ticks_remaining;

                    // ── Update telemetry ──────────────────────────────────
                    state.current_pose     = rs.O_T_EE;
                    state.current_q        = rs.q;
                    state.cmd_success_rate = rs.control_command_success_rate;
                    state.target_q         = interp_q;

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
                const auto& tq = interp_q;          // last commanded (RT-private)
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
