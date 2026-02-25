// franka_server_cartesian.cpp
//
// Lightweight gRPC server that drives a Franka robot with Cartesian velocity
// control via libfranka.
//
// Architecture:
//   - Main thread:   libfranka robot.control() 1 kHz RT loop
//                     returning franka::CartesianVelocities
//   - gRPC threads:  handle SetEETarget / GetRobotState at any rate
//   - Gripper thread: independent franka::Gripper, blocking move/grasp
//   - Shared state:  mutex-protected structs exchanged between threads
//
// Control:
//   Python sends a desired EE pose (4x4 col-major, same as O_T_EE).
//   The 1 kHz RT callback computes a PD velocity command:
//
//     v_lin = Kp_lin * (p_desired - p_current) - Kd_lin * v_current
//     v_rot = Kp_rot * rot_error(R_desired, R_current) - Kd_rot * w_current
//
//   where rot_error extracts the axis-angle rotation vector from
//   R_desired * R_current^T.
//
//   The velocity is clamped to max_lin_vel / max_rot_vel and returned as
//   franka::CartesianVelocities.  libfranka's internal joint impedance
//   controller handles the rest (IK, gravity, etc.).
//
// Usage:
//   ./franka_server_cartesian <robot_ip> <grpc_addr> [config_path]
//   e.g. ./franka_server_cartesian 192.168.1.11 0.0.0.0:50052 config/controller_cartesian.yaml

#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <grpcpp/grpcpp.h>
#include "franka_control.grpc.pb.h"
#include "franka_control.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

// ── Minimal 3D math helpers ─────────────────────────────────────────────────
//
// O_T_EE is column-major 4×4:
//   index:  [0]=R00  [1]=R10  [2]=R20  [3]=0
//           [4]=R01  [5]=R11  [6]=R21  [7]=0
//           [8]=R02  [9]=R12 [10]=R22 [11]=0
//          [12]=tx  [13]=ty  [14]=tz  [15]=1

struct Vec3 { double x, y, z; };
struct Mat3 { double m[3][3]; };  // m[row][col]

static Vec3 pose_position(const std::array<double,16>& T) {
    return {T[12], T[13], T[14]};
}

static Mat3 pose_rotation(const std::array<double,16>& T) {
    return {{{ T[0], T[4], T[8]  },
             { T[1], T[5], T[9]  },
             { T[2], T[6], T[10] }}};
}

// C = A * B^T
static Mat3 mat3_mul_Bt(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C.m[i][j] += A.m[i][k] * B.m[j][k];
    return C;
}

// Extract axis-angle rotation vector from rotation matrix.
//   rot_error = angle * axis   (3D vector, magnitude = angle in radians)
//
// For small angles (< 1e-6 rad), returns zero.
// For near-180° the result degrades but is acceptable for a teleop PD
// controller where the error should always be small.
static Vec3 rotation_error(const Mat3& Re) {
    double cos_a = 0.5 * (Re.m[0][0] + Re.m[1][1] + Re.m[2][2] - 1.0);
    cos_a = std::max(-1.0, std::min(1.0, cos_a));
    double angle = std::acos(cos_a);

    if (angle < 1e-10) return {0, 0, 0};

    double sin_a = std::sin(angle);
    // k = angle / (2 * sin(angle));  for angle → 0, k → 0.5
    double k = (sin_a > 1e-6) ? angle / (2.0 * sin_a) : 0.5;

    return { k * (Re.m[2][1] - Re.m[1][2]),
             k * (Re.m[0][2] - Re.m[2][0]),
             k * (Re.m[1][0] - Re.m[0][1]) };
}

// ── Controller config (loaded from YAML at startup) ──────────────────────────

struct ControllerConfig {
    double kp_lin      = 3.0;    // position P gain [1/s]
    double kd_lin      = 0.5;    // linear velocity damping
    double kp_rot      = 2.0;    // orientation P gain [1/s]
    double kd_rot      = 0.3;    // angular velocity damping
    double max_lin_vel = 0.5;    // clamp [m/s]
    double max_rot_vel = 2.5;    // clamp [rad/s]
    double cutoff_freq = 100.0;  // libfranka internal velocity LPF [Hz]
    double gripper_force   =  20.0;
    double gripper_eps_in  =  0.08;
    double gripper_eps_out =  0.08;
};

static std::string cfg_trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static ControllerConfig load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open config file: " + path);
    ControllerConfig cfg;
    std::string line;
    while (std::getline(f, line)) {
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = cfg_trim(line);
        if (line.empty()) continue;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = cfg_trim(line.substr(0, colon));
        std::string val = cfg_trim(line.substr(colon + 1));
        if (val.empty()) continue;
        if      (key == "kp_lin")          cfg.kp_lin          = std::stod(val);
        else if (key == "kd_lin")          cfg.kd_lin          = std::stod(val);
        else if (key == "kp_rot")          cfg.kp_rot          = std::stod(val);
        else if (key == "kd_rot")          cfg.kd_rot          = std::stod(val);
        else if (key == "max_lin_vel")     cfg.max_lin_vel     = std::stod(val);
        else if (key == "max_rot_vel")     cfg.max_rot_vel     = std::stod(val);
        else if (key == "cutoff_freq")     cfg.cutoff_freq     = std::stod(val);
        else if (key == "gripper_force")   cfg.gripper_force   = std::stod(val);
        else if (key == "gripper_eps_in")  cfg.gripper_eps_in  = std::stod(val);
        else if (key == "gripper_eps_out") cfg.gripper_eps_out = std::stod(val);
    }
    return cfg;
}

// ── Gripper shared state ─────────────────────────────────────────────────────

struct GripperSharedState {
    std::mutex mtx;
    std::condition_variable cv;

    double   desired_width{0.08};
    double   desired_speed{0.1};
    uint64_t cmd_seq{0};

    double current_width{0.08};
    bool   is_grasped{false};
    bool   ready{false};
    std::string error;

    std::atomic<bool> stop{false};
};

// ── Gripper thread ────────────────────────────────────────────────────────────

void run_gripper_thread(GripperSharedState& gs, const std::string& robot_ip,
                        const ControllerConfig& cfg) {
    try {
        franka::Gripper gripper(robot_ip);
        std::cout << "[cartesian_server] Gripper connected." << std::endl;

        {
            auto s = gripper.readOnce();
            std::lock_guard<std::mutex> lk(gs.mtx);
            gs.current_width = s.width;
            gs.is_grasped    = s.is_grasped;
            gs.ready         = true;
        }
        std::cout << "[cartesian_server] Gripper ready. Width = "
                  << gripper.readOnce().width << " m" << std::endl;

        uint64_t last_seq = 0;
        while (!gs.stop) {
            double width, speed;
            {
                std::unique_lock<std::mutex> lk(gs.mtx);
                gs.cv.wait_for(lk, std::chrono::milliseconds(200), [&] {
                    return gs.cmd_seq != last_seq || gs.stop.load();
                });
                if (gs.stop) break;
                if (gs.cmd_seq == last_seq) continue;
                width    = std::max(0.0, std::min(gs.desired_width, 0.08));
                speed    = std::max(0.01, std::min(gs.desired_speed, 0.2));
                last_seq = gs.cmd_seq;
            }

            try {
                if (width > 0.04) {
                    gripper.move(width, speed);
                } else {
                    gripper.grasp(width, speed, cfg.gripper_force,
                                  cfg.gripper_eps_in, cfg.gripper_eps_out);
                }
            } catch (const franka::Exception& e) {
                std::cerr << "[cartesian_server] Gripper error: " << e.what() << std::endl;
                try { gripper.stop(); } catch (...) {}
            }

            try {
                auto s = gripper.readOnce();
                std::lock_guard<std::mutex> lk(gs.mtx);
                gs.current_width = s.width;
                gs.is_grasped    = s.is_grasped;
            } catch (...) {}
        }
    } catch (const franka::Exception& e) {
        std::cerr << "[cartesian_server] Gripper init failed: " << e.what()
                  << "  (arm control continues without gripper)" << std::endl;
        std::lock_guard<std::mutex> lk(gs.mtx);
        gs.error = e.what();
    }
}

// ── Shared state between gRPC threads and the RT control loop ──────────────

struct SharedState {
    std::mutex mtx;
    std::condition_variable goal_cv;

    // Written by gRPC SetEETarget; read by RT loop.
    std::array<double, 16> goal_pose{};   // desired O_T_EE
    uint64_t               goal_seq{0};

    // Written by RT loop; read by gRPC GetRobotState.
    std::array<double, 16> current_pose{};
    std::array<double, 7>  current_q{};
    std::array<double, 16> target_pose{};   // what the PD is tracking
    double cmd_success_rate{0.0};
    bool   ready{false};
    std::string error{};

    std::atomic<bool> stop{false};
};

// ── gRPC service implementation ─────────────────────────────────────────────

class FrankaControlImpl final : public franka_control::FrankaControl::Service {
public:
    explicit FrankaControlImpl(SharedState& s, GripperSharedState& gs) : s_(s), gs_(gs) {}

    // Joint target — not used by this server, return error.
    Status SetJointTarget(ServerContext*,
                          const franka_control::JointTarget*,
                          franka_control::CommandResult* rep) override {
        rep->set_success(false);
        rep->set_message("This is the Cartesian server. Use SetEETarget instead.");
        return Status::OK;
    }

    Status SetEETarget(ServerContext*,
                       const franka_control::EETarget* req,
                       franka_control::CommandResult* rep) override {
        if (req->pose_size() != 16) {
            rep->set_success(false);
            rep->set_message("Expected 16 doubles (4x4 col-major transform)");
            return Status::OK;
        }
        {
            std::lock_guard<std::mutex> lk(s_.mtx);
            for (int i = 0; i < 16; ++i)
                s_.goal_pose[i] = req->pose(i);
            ++s_.goal_seq;
        }
        s_.goal_cv.notify_one();
        rep->set_success(true);
        return Status::OK;
    }

    Status SetGripperTarget(ServerContext*,
                            const franka_control::GripperTarget* req,
                            franka_control::CommandResult* rep) override {
        {
            std::lock_guard<std::mutex> lk(gs_.mtx);
            gs_.desired_width = std::max(0.0, std::min(req->width(), 0.08));
            gs_.desired_speed = req->speed() > 0.0 ? req->speed() : 0.1;
            ++gs_.cmd_seq;
        }
        gs_.cv.notify_one();
        rep->set_success(true);
        return Status::OK;
    }

    Status GetRobotState(ServerContext*,
                         const franka_control::Empty*,
                         franka_control::RobotState* rep) override {
        {
            std::lock_guard<std::mutex> lk(s_.mtx);
            for (double v : s_.current_pose)  rep->add_pose(v);
            for (double v : s_.current_q)     rep->add_q(v);
            for (double v : s_.target_pose)   rep->add_target_pose(v);
            rep->set_cmd_success_rate(s_.cmd_success_rate);
            rep->set_ready(s_.ready);
            rep->set_error(s_.error);
        }
        {
            std::lock_guard<std::mutex> lk(gs_.mtx);
            rep->set_gripper_width(gs_.current_width);
            rep->set_gripper_grasping(gs_.is_grasped);
        }
        return Status::OK;
    }

    Status Stop(ServerContext*,
                const franka_control::Empty*,
                franka_control::CommandResult* rep) override {
        s_.stop  = true;
        gs_.stop = true;
        s_.goal_cv.notify_all();
        gs_.cv.notify_all();
        rep->set_success(true);
        rep->set_message("Stop requested");
        return Status::OK;
    }

private:
    SharedState&        s_;
    GripperSharedState& gs_;
};

// ── gRPC server launcher ────────────────────────────────────────────────────

static std::unique_ptr<Server> g_server;

void run_grpc_server(SharedState& state, GripperSharedState& gs, const std::string& addr) {
    FrankaControlImpl service(state, gs);
    ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    g_server = builder.BuildAndStart();
    std::cout << "[cartesian_server] gRPC listening on " << addr << std::endl;
    g_server->Wait();
}

// ── Signal handler ──────────────────────────────────────────────────────────

static SharedState*        g_state_ptr   = nullptr;
static GripperSharedState* g_gripper_ptr = nullptr;

void signal_handler(int) {
    std::cout << "\n[cartesian_server] SIGINT received, stopping..." << std::endl;
    if (g_state_ptr) {
        g_state_ptr->stop = true;
        g_state_ptr->goal_cv.notify_all();
    }
    if (g_gripper_ptr) {
        g_gripper_ptr->stop = true;
        g_gripper_ptr->cv.notify_all();
    }
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const std::string robot_ip    = (argc > 1) ? argv[1] : "192.168.1.11";
    const std::string grpc_addr   = (argc > 2) ? argv[2] : "0.0.0.0:50052";
    const std::string config_path = (argc > 3) ? argv[3] : "";

    // ── Load config ──────────────────────────────────────────────────────────
    ControllerConfig cfg;
    if (!config_path.empty()) {
        try {
            cfg = load_config(config_path);
            std::cout << "[cartesian_server] Loaded config: " << config_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[cartesian_server] Config error: " << e.what() << std::endl;
            return 1;
        }
    } else {
        std::cout << "[cartesian_server] No config — using built-in defaults." << std::endl;
    }

    const double kp_lin      = cfg.kp_lin;
    const double kd_lin      = cfg.kd_lin;
    const double kp_rot      = cfg.kp_rot;
    const double kd_rot      = cfg.kd_rot;
    const double max_lin_vel = cfg.max_lin_vel;
    const double max_rot_vel = cfg.max_rot_vel;

    std::cout << "[cartesian_server] PD gains: kp_lin=" << kp_lin
              << "  kd_lin=" << kd_lin
              << "  kp_rot=" << kp_rot
              << "  kd_rot=" << kd_rot
              << "\n[cartesian_server] Velocity limits: lin=" << max_lin_vel
              << " m/s  rot=" << max_rot_vel
              << " rad/s  cutoff=" << cfg.cutoff_freq << " Hz" << std::endl;

    SharedState        state;
    GripperSharedState gripper_state;
    g_state_ptr   = &state;
    g_gripper_ptr = &gripper_state;
    std::signal(SIGINT, signal_handler);

    // ── Start gripper & gRPC threads ─────────────────────────────────────────
    std::thread gripper_thread([&]() { run_gripper_thread(gripper_state, robot_ip, cfg); });
    gripper_thread.detach();

    std::thread grpc_thread([&]() { run_grpc_server(state, gripper_state, grpc_addr); });
    grpc_thread.detach();

    // ── Connect to robot ─────────────────────────────────────────────────────
    std::cout << "[cartesian_server] Connecting to robot at " << robot_ip << " ..." << std::endl;
    franka::Robot robot(robot_ip);
    std::cout << "[cartesian_server] Connected." << std::endl;

    // Seed shared state from initial robot state.
    {
        franka::RobotState rs0 = robot.readOnce();
        std::lock_guard<std::mutex> lk(state.mtx);
        state.current_pose = rs0.O_T_EE;
        state.current_q    = rs0.q;
        state.goal_pose    = rs0.O_T_EE;  // hold current pose until first command
        state.target_pose  = rs0.O_T_EE;
        state.ready        = true;
    }

    // Helper: wait for first / new command.
    auto wait_for_goal = [&](uint64_t min_seq) -> bool {
        std::unique_lock<std::mutex> lk(state.mtx);
        state.goal_cv.wait(lk, [&]{
            return state.goal_seq > min_seq || state.stop.load();
        });
        return !state.stop.load();
    };

    std::cout << "[cartesian_server] Ready. Waiting for first SetEETarget..." << std::endl;
    if (!wait_for_goal(0)) {
        std::cout << "[cartesian_server] Stopped before first command." << std::endl;
        if (g_server) g_server->Shutdown();
        return 0;
    }
    std::cout << "[cartesian_server] First command received, starting control." << std::endl;

    // ── Cartesian velocity control loop (with auto-recovery) ────────────────
    while (!state.stop) {

        // Set collision thresholds BEFORE robot.control().
        robot.setCollisionBehavior(
            {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
            {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
            {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
            {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},
            {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}},
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},
            {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}});

        // Set internal joint impedance (used by the internal joint controller
        // that tracks our Cartesian velocity commands).
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

        uint64_t tick = 0;

        // Update shared telemetry from fresh read.
        {
            franka::RobotState rs0 = robot.readOnce();
            std::lock_guard<std::mutex> lk(state.mtx);
            state.current_pose = rs0.O_T_EE;
            state.current_q    = rs0.q;
            state.target_pose  = state.goal_pose;
            state.error        = {};
        }

        std::cout << "[cartesian_server] Entering Cartesian velocity control loop." << std::endl;

        try {
            robot.control(
                [&](const franka::RobotState& rs,
                    franka::Duration) -> franka::CartesianVelocities
                {
                    // ── Read goal pose under lock ────────────────────────
                    std::array<double, 16> gp;
                    {
                        std::lock_guard<std::mutex> lk(state.mtx);
                        gp = state.goal_pose;
                    }

                    // ── Position error ───────────────────────────────────
                    Vec3 p_cur = pose_position(rs.O_T_EE);
                    Vec3 p_des = pose_position(gp);
                    double ex = p_des.x - p_cur.x;
                    double ey = p_des.y - p_cur.y;
                    double ez = p_des.z - p_cur.z;

                    // ── Orientation error ────────────────────────────────
                    //   R_err = R_desired * R_current^T
                    //   rot_err = axis-angle vector of R_err
                    Mat3 R_cur = pose_rotation(rs.O_T_EE);
                    Mat3 R_des = pose_rotation(gp);
                    Mat3 R_err = mat3_mul_Bt(R_des, R_cur);
                    Vec3 rot_err = rotation_error(R_err);

                    // ── Current velocity (last commanded twist) ──────────
                    const auto& v = rs.O_dP_EE_c;
                    // v[0..2] = linear velocity, v[3..5] = angular velocity

                    // ── PD controller ────────────────────────────────────
                    double vx = kp_lin * ex - kd_lin * v[0];
                    double vy = kp_lin * ey - kd_lin * v[1];
                    double vz = kp_lin * ez - kd_lin * v[2];

                    double wx = kp_rot * rot_err.x - kd_rot * v[3];
                    double wy = kp_rot * rot_err.y - kd_rot * v[4];
                    double wz = kp_rot * rot_err.z - kd_rot * v[5];

                    // ── Clamp linear velocity ────────────────────────────
                    double lin_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
                    if (lin_norm > max_lin_vel) {
                        double s = max_lin_vel / lin_norm;
                        vx *= s; vy *= s; vz *= s;
                    }

                    // ── Clamp angular velocity ───────────────────────────
                    double rot_norm = std::sqrt(wx*wx + wy*wy + wz*wz);
                    if (rot_norm > max_rot_vel) {
                        double s = max_rot_vel / rot_norm;
                        wx *= s; wy *= s; wz *= s;
                    }

                    // ── Update telemetry under lock ──────────────────────
                    {
                        std::lock_guard<std::mutex> lk(state.mtx);
                        state.current_pose     = rs.O_T_EE;
                        state.current_q        = rs.q;
                        state.target_pose      = gp;
                        state.cmd_success_rate = rs.control_command_success_rate;
                    }
                    ++tick;

                    // ── Stop if requested ────────────────────────────────
                    if (state.stop) {
                        std::array<double, 6> zero_vel = {0, 0, 0, 0, 0, 0};
                        franka::CartesianVelocities cv(zero_vel);
                        cv.motion_finished = true;
                        return cv;
                    }

                    std::array<double, 6> cmd_vel = {vx, vy, vz, wx, wy, wz};
                    return franka::CartesianVelocities(cmd_vel);
                },
                franka::ControllerMode::kJointImpedance,
                true,               // limit_rate — prevents velocity discontinuities
                cfg.cutoff_freq     // internal velocity LPF
            );

        } catch (const franka::ControlException& e) {
            std::cerr << "[cartesian_server] ControlException at tick " << tick
                      << " (" << (tick / 1000.0) << " s): " << e.what() << std::endl;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                state.ready = false;
            }
            try {
                robot.automaticErrorRecovery();
            } catch (const franka::Exception& e2) {
                std::cerr << "[cartesian_server] Recovery failed: " << e2.what() << std::endl;
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
            std::cerr << "[cartesian_server] Recovered. Waiting for new SetEETarget..." << std::endl;
            if (!wait_for_goal(seq_at_recovery)) break;
            std::cerr << "[cartesian_server] New command received, resuming control." << std::endl;

        } catch (const franka::Exception& e) {
            std::cerr << "[cartesian_server] Fatal exception: " << e.what() << std::endl;
            std::lock_guard<std::mutex> lk(state.mtx);
            state.error = e.what();
            state.ready = false;
            break;
        }
    }

    std::cout << "[cartesian_server] Control loop exited." << std::endl;
    if (g_server) g_server->Shutdown();
    return 0;
}
