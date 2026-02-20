// franka_server.cpp
//
// Lightweight gRPC server that drives a Franka robot with libfranka's
// torque controller.
//
// Architecture:
//   - Main thread:   libfranka robot.control() 1 kHz RT loop (no gRPC inside)
//   - gRPC threads:  handle SetJointTarget / GetRobotState at any rate
//   - Shared state:  a mutex-protected struct exchanged between both sides
//
// Startup sequence:
//   1. gRPC server starts accepting connections immediately.
//   2. Main thread connects to the robot, calls readOnce() to populate
//      telemetry, sets ready=true so Python can read current q as baseline.
//   3. Main thread WAITS for the first SetJointTarget command.
//   4. First command received → enter robot.control() 1 kHz RT loop.
//   5. On ControlException: automaticErrorRecovery(), then WAIT for a new
//      SetJointTarget before re-entering robot.control().
//
// Torque control — matches polymetis DefaultController algorithm:
//
//   At each 1 kHz tick:
//
//     interp_q tracks goal_q via linear interpolation (same as before):
//       step = (goal_q - interp_q) / ticks_remaining
//       step = clamp(step, -max_step, +max_step)
//       interp_q += step
//
//     Joint impedance torque:
//       τ[i] = Kp[i] × (interp_q[i] − q[i])   ← position error
//            − Kd[i] × dq[i]                   ← velocity damping
//            + coriolis[i]                      ← feedforward (Coriolis+centrifugal)
//       τ[i] = clamp(τ[i], −tau_limit[i], +tau_limit[i])
//
//   libfranka adds gravity compensation automatically in torque control mode,
//   so gravity is NOT included in τ (matches polymetis JointImpedanceControl).
//
//   Gains match polymetis franka_hardware.yaml defaults:
//     Kp = [40, 30, 50, 25, 35, 25, 10]  N⋅m/rad
//     Kd = [ 4,  6,  5,  5,  3,  2,  1]  N⋅m⋅s/rad
//
// Usage:
//   ./franka_server <robot_ip> <grpc_listen_addr> [policy_hz]
//   e.g. ./franka_server 192.168.1.11 0.0.0.0:50052 25

#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/model.h>
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
    std::condition_variable goal_cv;  // notified on each SetJointTarget / Stop

    // Written by gRPC SetJointTarget; read by RT loop each tick.
    std::array<double, 7> goal_q{};   // latest joint target from Python
    uint64_t              goal_seq{0}; // incremented on each SetJointTarget

    // Written by RT loop (or readOnce init); read by gRPC GetRobotState.
    std::array<double, 16> current_pose{};  // O_T_EE — actual measured pose
    std::array<double, 7>  current_q{};     // actual measured joint positions
    std::array<double, 7>  target_q{};      // current interpolated command
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
        s_.goal_cv.notify_one();
        rep->set_success(true);
        return Status::OK;
    }

    Status GetRobotState(ServerContext*,
                         const franka_control::Empty*,
                         franka_control::RobotState* rep) override {
        std::lock_guard<std::mutex> lk(s_.mtx);
        for (double v : s_.current_pose) rep->add_pose(v);
        for (double v : s_.current_q)    rep->add_q(v);
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
        s_.goal_cv.notify_all();
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
    if (g_state_ptr) {
        g_state_ptr->stop = true;
        g_state_ptr->goal_cv.notify_all();
    }
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const std::string robot_ip  = (argc > 1) ? argv[1] : "192.168.1.11";
    const std::string grpc_addr = (argc > 2) ? argv[2] : "0.0.0.0:50052";
    const double      policy_hz = (argc > 3) ? std::stod(argv[3]) : 25.0;

    const int interp_N = static_cast<int>(std::round(1000.0 / policy_hz));
    constexpr double max_step = 0.001;  // rad per tick (= 1.0 rad/s)

    // ── Joint impedance gains — polymetis franka_hardware.yaml defaults ──────
    //
    // These match the DefaultController gains used by TorchScriptedController:
    //   Kp = default_Kq  = [40, 30, 50, 25, 35, 25, 10]  N⋅m/rad
    //   Kd = default_Kqd = [ 4,  6,  5,  5,  3,  2,  1]  N⋅m⋅s/rad
    const std::array<double, 7> kp = {{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0}};
    const std::array<double, 7> kd = {{ 4.0,  6.0,  5.0,  5.0,  3.0,  2.0,  1.0}};

    // Torque limits per joint [N⋅m] — Franka Panda / FR3.
    // Joints 1-4: ±87 N⋅m, joints 5-7: ±12 N⋅m.
    const std::array<double, 7> tau_limit = {{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};

    std::cout << "[franka_server] policy_hz=" << policy_hz
              << "  interp_N=" << interp_N << " ticks/waypoint"
              << "  max_step=" << max_step << " rad/tick" << std::endl;

    SharedState state;
    g_state_ptr = &state;
    std::signal(SIGINT, signal_handler);

    // ── Start gRPC server in background thread ──────────────────────────────
    std::thread grpc_thread([&]() { run_grpc_server(state, grpc_addr); });
    grpc_thread.detach();

    // ── Connect to robot and load dynamics model ────────────────────────────
    std::cout << "[franka_server] Connecting to robot at " << robot_ip << " ..." << std::endl;
    franka::Robot robot(robot_ip);
    franka::Model model = robot.loadModel();
    std::cout << "[franka_server] Connected." << std::endl;

    // Read initial state so Python can see current q before the RT loop starts.
    // Set ready=true so Python's wait_until_ready() returns and reads the
    // baseline for its first SetJointTarget command.
    {
        franka::RobotState rs0 = robot.readOnce();
        std::lock_guard<std::mutex> lk(state.mtx);
        state.current_pose = rs0.O_T_EE;
        state.current_q    = rs0.q;
        state.target_q     = rs0.q;
        state.goal_q       = rs0.q;
        state.ready        = true;
    }

    // Helper: block until goal_seq > min_seq, or stop requested.
    auto wait_for_goal = [&](uint64_t min_seq) -> bool {
        std::unique_lock<std::mutex> lk(state.mtx);
        state.goal_cv.wait(lk, [&]{
            return state.goal_seq > min_seq || state.stop.load();
        });
        return !state.stop.load();
    };

    // ── Wait for the first command before entering the RT loop ─────────────
    std::cout << "[franka_server] Ready. Waiting for first SetJointTarget..." << std::endl;
    if (!wait_for_goal(0)) {
        std::cout << "[franka_server] Stopped before first command." << std::endl;
        if (g_server) g_server->Shutdown();
        return 0;
    }
    std::cout << "[franka_server] First command received, starting control." << std::endl;

    // ── libfranka torque control loop (with auto-recovery) ──────────────────
    while (!state.stop) {

        // Fresh robot state just before entering the RT loop.
        // Seed interp_q from actual position to ensure zero discontinuity.
        franka::RobotState rs0 = robot.readOnce();

        // Collision behavior — set before robot.control(), never inside callback.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // RT-private state — reset on each outer loop iteration.
        std::array<double, 7> interp_q = rs0.q;  // seeded from actual position
        uint64_t last_goal_seq  = 0;
        int  ticks_remaining    = 0;
        uint64_t tick           = 0;

        // Update shared telemetry with the fresh read.
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.current_pose = rs0.O_T_EE;
            state.current_q    = rs0.q;
            state.target_q     = rs0.q;
            state.error        = {};
        }

        std::cout << "[franka_server] Control loop starting. q = ["
                  << rs0.q[0] << ", " << rs0.q[1] << ", " << rs0.q[2] << ", "
                  << rs0.q[3] << ", " << rs0.q[4] << ", " << rs0.q[5] << ", "
                  << rs0.q[6] << "]" << std::endl;

        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::Torques
                {
                    // ── Coriolis feedforward (outside mutex — pure computation) ──
                    //
                    // model.coriolis(rs) = C(q, dq) · dq
                    //   = Coriolis + centrifugal forces (no gravity).
                    //
                    // libfranka adds gravity compensation automatically in torque
                    // control mode, so we only need coriolis for feedforward.
                    // This matches polymetis JointImpedanceControl:
                    //   τ_ff = invdyn(q, dq, 0) − invdyn(q, 0, 0)  ← coriolis only
                    const std::array<double, 7> coriolis = model.coriolis(rs);

                    // ── Lock shared state ──────────────────────────────────
                    std::lock_guard<std::mutex> lk(state.mtx);

                    // ── Interpolate interp_q toward goal_q ────────────────
                    //
                    // On new 25 Hz command: reset ticks_remaining = interp_N.
                    // step = (goal_q - interp_q) / ticks_remaining  ← constant/period
                    // step = clamp(step, ±max_step)                 ← safety cap
                    if (state.goal_seq != last_goal_seq) {
                        last_goal_seq   = state.goal_seq;
                        ticks_remaining = interp_N;
                    }
                    const auto& gq = state.goal_q;
                    for (int i = 0; i < 7; ++i) {
                        double d    = gq[i] - interp_q[i];
                        double step = (ticks_remaining > 0) ? d / ticks_remaining : 0.0;
                        interp_q[i] += std::max(-max_step, std::min(step, max_step));
                    }
                    if (ticks_remaining > 0) --ticks_remaining;
                    ++tick;

                    // ── Joint impedance torque (DefaultController formula) ─
                    //
                    //   τ[i] = Kp[i] × (interp_q[i] − q[i])   position error
                    //        − Kd[i] × dq[i]                   velocity damping
                    //        + coriolis[i]                      feedforward
                    //
                    // Clamped to hardware torque limits.
                    std::array<double, 7> tau;
                    for (int i = 0; i < 7; ++i) {
                        tau[i] = kp[i] * (interp_q[i] - rs.q[i])
                               - kd[i] * rs.dq[i]
                               + coriolis[i];
                        tau[i] = std::max(-tau_limit[i], std::min(tau[i], tau_limit[i]));
                    }

                    // ── Update telemetry ──────────────────────────────────
                    state.current_pose     = rs.O_T_EE;
                    state.current_q        = rs.q;
                    state.cmd_success_rate = rs.control_command_success_rate;
                    state.target_q         = interp_q;

                    // ── Finish if stop requested ──────────────────────────
                    if (state.stop) {
                        franka::Torques t(tau);
                        t.motion_finished = true;
                        return t;
                    }

                    return franka::Torques(tau);
                }
            );

        } catch (const franka::ControlException& e) {
            std::cerr << "[franka_server] Control exception at tick " << tick
                      << " (" << (tick / 1000.0) << " s): " << e.what() << std::endl;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                const auto& cq = state.current_q;
                const auto& tq = interp_q;
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
            } catch (const franka::Exception& e2) {
                std::cerr << "[franka_server] Recovery failed: " << e2.what() << std::endl;
                std::lock_guard<std::mutex> lk(state.mtx);
                state.error = e2.what();
                break;
            }
            uint64_t seq_at_recovery;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                seq_at_recovery = state.goal_seq;
                state.ready = true;
            }
            std::cerr << "[franka_server] Recovered. Waiting for new SetJointTarget..." << std::endl;
            if (!wait_for_goal(seq_at_recovery)) break;
            std::cerr << "[franka_server] New command received, resuming control." << std::endl;

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
