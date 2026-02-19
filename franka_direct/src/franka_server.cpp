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
// Usage:
//   ./franka_server <robot_ip> <grpc_listen_addr>
//   e.g. ./franka_server 192.168.1.11 0.0.0.0:50052

#include <array>
#include <atomic>
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

    // Written by gRPC thread, read by RT loop
    std::array<double, 16> target_pose{};

    // Written by RT loop, read by gRPC thread
    std::array<double, 16> current_pose{};
    std::array<double, 7>  current_q{};
    double cmd_success_rate{0.0};
    bool   ready{false};
    std::string error{};

    // Atomic flags — safe to read/write without mutex
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
                s_.target_pose[i] = req->pose(i);
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
    const std::string robot_ip  = (argc > 1) ? argv[1] : "192.168.1.11";
    const std::string grpc_addr = (argc > 2) ? argv[2] : "0.0.0.0:50052";

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

    // Configure collision thresholds (same as polymetis defaults)
    robot.setCollisionBehavior(
        {{40,40,40,40,40,40,40}}, {{40,40,40,40,40,40,40}},
        {{40,40,40,40,40,40}},    {{40,40,40,40,40,40}});

    // ── libfranka Cartesian pose control loop (with auto-recovery) ──────────
    //
    // The callback runs at 1 kHz and only reads/writes shared memory — no
    // gRPC calls inside.  This keeps the callback well within the 1 ms budget.
    //
    // target_pose is seeded from the FIRST callback invocation (not readOnce)
    // so there is no stale-pose discontinuity at startup.  ready is set only
    // after that first call so the Python client cannot send targets early.
    //
    while (!state.stop) {
        bool control_initialized = false;
        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::CartesianPose
                {
                    std::lock_guard<std::mutex> lk(state.mtx);

                    // First callback: initialize target from robot's exact
                    // current commanded pose and mark server as ready.
                    if (!control_initialized) {
                        state.target_pose = rs.O_T_EE_d;
                        state.ready = true;
                        control_initialized = true;
                        std::cout << "[franka_server] Control loop started. EE z = "
                                  << rs.O_T_EE[14] << " m" << std::endl;
                    }

                    // Update telemetry (visible to gRPC GetRobotState)
                    state.current_pose     = rs.O_T_EE;
                    state.current_q        = rs.q;
                    state.cmd_success_rate = rs.control_command_success_rate;

                    // Finish motion if stop requested
                    if (state.stop) {
                        franka::CartesianPose p(state.target_pose);
                        p.motion_finished = true;
                        return p;
                    }

                    return franka::CartesianPose(state.target_pose);
                },
                franka::ControllerMode::kJointImpedance,
                true,   // limit_rate
                100.0   // cutoff_frequency [Hz]
            );

        } catch (const franka::ControlException& e) {
            std::cerr << "[franka_server] Control exception: " << e.what() << std::endl;
            // Attempt automatic error recovery (mirrors franka_panda_client behavior)
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                state.ready = false;  // block Python client until re-initialized
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
