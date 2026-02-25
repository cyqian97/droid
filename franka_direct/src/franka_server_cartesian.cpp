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
    // Per-axis linear PD gains.  If kp_x/y/z are not set in YAML,
    // they default to kp_lin (and similarly for kd).
    double kp_lin      = 3.0;    // fallback P gain [1/s]
    double kd_lin      = 0.5;    // fallback D gain
    double kp_x        = -1;     // per-axis overrides (-1 = use kp_lin)
    double kp_y        = -1;
    double kp_z        = -1;
    double kd_x        = -1;
    double kd_y        = -1;
    double kd_z        = -1;
    double kp_rot      = 2.0;    // orientation P gain [1/s]
    double kd_rot      = 0.3;    // angular velocity damping
    double max_lin_vel   = 0.5;    // clamp [m/s]
    double max_rot_vel   = 2.5;    // clamp [rad/s]
    double max_lin_accel = 5.0;    // max linear acceleration [m/s²]
    double max_rot_accel = 10.0;   // max angular acceleration [rad/s²]
    double cutoff_freq = 100.0;  // libfranka internal velocity LPF [Hz]
    double ramp_duration = 0.5;  // velocity ramp-up/down duration [s]
    double gripper_force   =  20.0;
    double gripper_eps_in  =  0.08;
    double gripper_eps_out =  0.08;

    // Initial joint configuration.  If non-empty, the robot moves here
    // (using a smooth joint-space motion generator) before entering the
    // Cartesian velocity control loop.
    std::array<double, 7> init_q{};
    bool   has_init_q     = false;
    double init_speed     = 0.3;   // fraction of max joint velocity [0..1]
    double init_duration  = 5.0;   // max seconds for the move

    // Resolve per-axis gains: use explicit override if set, else fallback.
    void resolve() {
        if (kp_x < 0) kp_x = kp_lin;
        if (kp_y < 0) kp_y = kp_lin;
        if (kp_z < 0) kp_z = kp_lin;
        if (kd_x < 0) kd_x = kd_lin;
        if (kd_y < 0) kd_y = kd_lin;
        if (kd_z < 0) kd_z = kd_lin;
    }
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
        else if (key == "kp_x")            cfg.kp_x            = std::stod(val);
        else if (key == "kp_y")            cfg.kp_y            = std::stod(val);
        else if (key == "kp_z")            cfg.kp_z            = std::stod(val);
        else if (key == "kd_x")            cfg.kd_x            = std::stod(val);
        else if (key == "kd_y")            cfg.kd_y            = std::stod(val);
        else if (key == "kd_z")            cfg.kd_z            = std::stod(val);
        else if (key == "kp_rot")          cfg.kp_rot          = std::stod(val);
        else if (key == "kd_rot")          cfg.kd_rot          = std::stod(val);
        else if (key == "max_lin_vel")     cfg.max_lin_vel     = std::stod(val);
        else if (key == "max_rot_vel")     cfg.max_rot_vel     = std::stod(val);
        else if (key == "max_lin_accel")   cfg.max_lin_accel   = std::stod(val);
        else if (key == "max_rot_accel")   cfg.max_rot_accel   = std::stod(val);
        else if (key == "cutoff_freq")     cfg.cutoff_freq     = std::stod(val);
        else if (key == "ramp_duration")   cfg.ramp_duration   = std::stod(val);
        else if (key == "init_speed")      cfg.init_speed      = std::stod(val);
        else if (key == "init_duration")   cfg.init_duration   = std::stod(val);
        else if (key == "gripper_force")   cfg.gripper_force   = std::stod(val);
        else if (key == "gripper_eps_in")  cfg.gripper_eps_in  = std::stod(val);
        else if (key == "gripper_eps_out") cfg.gripper_eps_out = std::stod(val);
        else if (key == "init_q") {
            // Parse comma-separated list of 7 doubles
            std::istringstream ss(val);
            std::string tok;
            int i = 0;
            while (std::getline(ss, tok, ',') && i < 7) {
                cfg.init_q[i++] = std::stod(cfg_trim(tok));
            }
            if (i == 7) cfg.has_init_q = true;
            else std::cerr << "[cartesian_server] Warning: init_q needs 7 values, got " << i << std::endl;
        }
    }
    cfg.resolve();
    return cfg;
}

// ── Joint-space motion generator ─────────────────────────────────────────
//
// Used by both init_q (startup) and the ResetToJoints RPC (runtime).
//
// Moves the robot to a target joint configuration using a cosine
// interpolation profile.  Velocity and acceleration are exactly zero at
// both endpoints → no discontinuity reflexes.
//
//   q(t) = q_start + (q_goal - q_start) * 0.5 * (1 - cos(pi * t / T))
//
// Total duration T is computed per-joint from max velocity (scaled by
// speed_factor), then synchronized to the slowest joint.

static void move_to_joint_pose(franka::Robot& robot,
                                const std::array<double, 7>& q_goal,
                                double speed_factor,
                                double max_duration) {
    // FR3 max joint velocities [rad/s] (from datasheet, conservative).
    constexpr std::array<double, 7> dq_max = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    std::array<double, 7> q_start{};
    double T = 0.0;   // synchronized duration
    double t = 0.0;
    bool   first_tick = true;

    robot.control(
        [&](const franka::RobotState& rs, franka::Duration period) -> franka::JointPositions {
            t += period.toSec();

            if (first_tick) {
                first_tick = false;
                // First tick: capture start pose and compute duration.
                q_start = rs.q;

                // Per-joint time based on distance / (speed_factor * max_vel).
                // The cosine profile has peak velocity = pi/(2*T) * delta_q,
                // so T_j = pi * |delta_q_j| / (2 * speed_factor * dq_max_j).
                for (int i = 0; i < 7; ++i) {
                    double dq = std::abs(q_goal[i] - q_start[i]);
                    double Tj = M_PI * dq / (2.0 * speed_factor * dq_max[i]);
                    if (Tj > T) T = Tj;
                }
                // Enforce minimum duration (avoid div-by-zero if already there).
                T = std::max(T, 0.5);
                T = std::min(T, max_duration);
                std::cout << "[cartesian_server] Moving to init_q (T=" << T << " s)..." << std::endl;
            }

            double frac = std::min(t / T, 1.0);
            double alpha = 0.5 * (1.0 - std::cos(M_PI * frac));

            std::array<double, 7> q_cmd;
            for (int i = 0; i < 7; ++i)
                q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * alpha;

            franka::JointPositions output(q_cmd);
            if (frac >= 1.0) {
                output.motion_finished = true;
            }
            return output;
        });
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

    // ── Reset request (gRPC → main loop) ────────────────────────────────
    // gRPC ResetToJoints sets these, then waits on reset_cv.
    // The RT callback ramps down; the main loop does the joint move.
    std::atomic<bool>      reset_requested{false};
    std::array<double, 7>  reset_q{};
    double                 reset_speed{0.3};
    double                 reset_max_duration{5.0};
    bool                   reset_complete{false};
    std::string            reset_error{};
    std::condition_variable reset_cv;
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

    Status ResetToJoints(ServerContext*,
                         const franka_control::JointResetTarget* req,
                         franka_control::CommandResult* rep) override {
        if (req->q_size() != 7) {
            rep->set_success(false);
            rep->set_message("Expected 7 joint angles");
            return Status::OK;
        }
        {
            std::lock_guard<std::mutex> lk(s_.mtx);
            for (int i = 0; i < 7; ++i)
                s_.reset_q[i] = req->q(i);
            s_.reset_speed        = req->speed() > 0 ? req->speed() : 0.3;
            s_.reset_max_duration = req->max_duration() > 0 ? req->max_duration() : 5.0;
            s_.reset_complete     = false;
            s_.reset_error.clear();
            s_.reset_requested    = true;   // atomic — RT callback will see it
        }
        // Wake RT callback (if in wait_for_goal) to start ramp-down.
        s_.goal_cv.notify_all();

        // Block until the main loop finishes the joint move.
        {
            std::unique_lock<std::mutex> lk(s_.mtx);
            s_.reset_cv.wait(lk, [&]{
                return s_.reset_complete || s_.stop.load();
            });
        }

        std::lock_guard<std::mutex> lk(s_.mtx);
        if (s_.reset_error.empty()) {
            rep->set_success(true);
            rep->set_message("Reset complete");
        } else {
            rep->set_success(false);
            rep->set_message(s_.reset_error);
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

    // Resolve per-axis gains (needed if no config file).
    cfg.resolve();

    const double kp_x        = cfg.kp_x;
    const double kp_y        = cfg.kp_y;
    const double kp_z        = cfg.kp_z;
    const double kd_x        = cfg.kd_x;
    const double kd_y        = cfg.kd_y;
    const double kd_z        = cfg.kd_z;
    const double kp_rot      = cfg.kp_rot;
    const double kd_rot      = cfg.kd_rot;
    const double max_lin_vel = cfg.max_lin_vel;
    const double max_rot_vel = cfg.max_rot_vel;
    const uint64_t ramp_ticks = std::max(uint64_t(1),
                                        static_cast<uint64_t>(cfg.ramp_duration * 1000));

    std::cout << "[cartesian_server] PD gains: kp=[" << kp_x << ", " << kp_y << ", " << kp_z << "]"
              << "  kd=[" << kd_x << ", " << kd_y << ", " << kd_z << "]"
              << "  kp_rot=" << kp_rot << "  kd_rot=" << kd_rot
              << "\n[cartesian_server] Velocity limits: lin=" << max_lin_vel
              << " m/s  rot=" << max_rot_vel
              << " rad/s  accel=" << cfg.max_lin_accel << "/" << cfg.max_rot_accel
              << "  cutoff=" << cfg.cutoff_freq << " Hz"
              << "  ramp=" << cfg.ramp_duration << " s" << std::endl;

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

    // ── Move to initial joint configuration (if specified) ──────────────────
    if (cfg.has_init_q) {
        std::cout << "[cartesian_server] init_q: [";
        for (int i = 0; i < 7; ++i) std::cout << (i ? ", " : "") << cfg.init_q[i];
        std::cout << "]  speed=" << cfg.init_speed << std::endl;

        franka::RobotState rs0 = robot.readOnce();
        double max_delta = 0;
        for (int i = 0; i < 7; ++i)
            max_delta = std::max(max_delta, std::abs(cfg.init_q[i] - rs0.q[i]));

        if (max_delta < 1e-3) {
            std::cout << "[cartesian_server] Already at init_q (max delta "
                      << max_delta << " rad), skipping move." << std::endl;
        } else {
            std::cout << "[cartesian_server] Max joint delta: " << max_delta
                      << " rad.  Moving to init_q..." << std::endl;
            try {
                move_to_joint_pose(robot, cfg.init_q, cfg.init_speed, cfg.init_duration);
                std::cout << "[cartesian_server] Reached init_q." << std::endl;
            } catch (const franka::Exception& e) {
                std::cerr << "[cartesian_server] Failed to move to init_q: " << e.what() << std::endl;
                try { robot.automaticErrorRecovery(); } catch (...) {}
                std::cerr << "[cartesian_server] Continuing from current pose." << std::endl;
            }
        }
    }

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

    // Helper: wait for first / new command (also wakes on reset or stop).
    auto wait_for_goal = [&](uint64_t min_seq) -> bool {
        std::unique_lock<std::mutex> lk(state.mtx);
        state.goal_cv.wait(lk, [&]{
            return state.goal_seq > min_seq
                || state.stop.load()
                || state.reset_requested.load();
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
        bool     ramping_down = false;
        uint64_t ramp_down_start = 0;
        std::array<double, 6> prev_cmd = {0, 0, 0, 0, 0, 0};

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
                    // 1 kHz callback steps:
                    //   1. Read goal pose (mutex)
                    //   2. Position error: p_desired - p_current
                    //   3. Orientation error: axis-angle(R_desired * R_current^T)
                    //   4. PD velocity: v = Kp * error - Kd * velocity
                    //   5a/b. Clamp linear / angular velocity norms
                    //   5c. Rate-limit velocity change (acceleration clamp)
                    //   6. Apply cosine ramp (startup / shutdown)
                    //   7. Update telemetry (mutex)

                    // ── 1. Read goal pose under lock ─────────────────────
                    std::array<double, 16> gp;
                    {
                        std::lock_guard<std::mutex> lk(state.mtx);
                        gp = state.goal_pose;
                    }

                    // ── 2. Position error ────────────────────────────────
                    Vec3 p_cur = pose_position(rs.O_T_EE);
                    Vec3 p_des = pose_position(gp);
                    double ex = p_des.x - p_cur.x;
                    double ey = p_des.y - p_cur.y;
                    double ez = p_des.z - p_cur.z;

                    // ── 3. Orientation error ─────────────────────────────
                    //   R_err = R_desired * R_current^T
                    //   rot_err = axis-angle vector of R_err
                    Mat3 R_cur = pose_rotation(rs.O_T_EE);
                    Mat3 R_des = pose_rotation(gp);
                    Mat3 R_err = mat3_mul_Bt(R_des, R_cur);
                    Vec3 rot_err = rotation_error(R_err);

                    // ── 3b. Current velocity (last commanded twist) ──────
                    const auto& v = rs.O_dP_EE_c;
                    // v[0..2] = linear velocity, v[3..5] = angular velocity

                    // ── 4. PD controller (per-axis linear gains) ─────────
                    double vx = kp_x * ex - kd_x * v[0];
                    double vy = kp_y * ey - kd_y * v[1];
                    double vz = kp_z * ez - kd_z * v[2];

                    double wx = kp_rot * rot_err.x - kd_rot * v[3];
                    double wy = kp_rot * rot_err.y - kd_rot * v[4];
                    double wz = kp_rot * rot_err.z - kd_rot * v[5];

                    // ── 5a. Clamp linear velocity ─────────────────────────
                    double lin_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
                    if (lin_norm > max_lin_vel) {
                        double s = max_lin_vel / lin_norm;
                        vx *= s; vy *= s; vz *= s;
                    }

                    // ── 5b. Clamp angular velocity ────────────────────────
                    double rot_norm = std::sqrt(wx*wx + wy*wy + wz*wz);
                    if (rot_norm > max_rot_vel) {
                        double s = max_rot_vel / rot_norm;
                        wx *= s; wy *= s; wz *= s;
                    }

                    // ── 5c. Rate-limit velocity change (acceleration clamp) ─
                    {
                        constexpr double dt = 0.001;  // 1 kHz
                        double max_dv_lin = cfg.max_lin_accel * dt;
                        double max_dv_rot = cfg.max_rot_accel * dt;

                        double dlx = vx - prev_cmd[0], dly = vy - prev_cmd[1], dlz = vz - prev_cmd[2];
                        double dl_norm = std::sqrt(dlx*dlx + dly*dly + dlz*dlz);
                        if (dl_norm > max_dv_lin) {
                            double s = max_dv_lin / dl_norm;
                            vx = prev_cmd[0] + dlx * s;
                            vy = prev_cmd[1] + dly * s;
                            vz = prev_cmd[2] + dlz * s;
                        }

                        double drx = wx - prev_cmd[3], dry = wy - prev_cmd[4], drz = wz - prev_cmd[5];
                        double dr_norm = std::sqrt(drx*drx + dry*dry + drz*drz);
                        if (dr_norm > max_dv_rot) {
                            double s = max_dv_rot / dr_norm;
                            wx = prev_cmd[3] + drx * s;
                            wy = prev_cmd[4] + dry * s;
                            wz = prev_cmd[5] + drz * s;
                        }
                    }

                    // ── 6. Velocity ramp ─────────────────────────────────
                    // Ramp up from zero on startup so the velocity doesn't
                    // jump discontinuously.  Ramp down to zero on stop so
                    // motion_finished is only set once velocity ≈ 0.
                    double ramp = 1.0;

                    if (!ramping_down && (state.stop.load() || state.reset_requested.load())) {
                        ramping_down    = true;
                        ramp_down_start = tick;
                    }

                    if (ramping_down) {
                        uint64_t dt = tick - ramp_down_start;
                        if (dt >= ramp_ticks) {
                            std::array<double, 6> zero_vel = {0, 0, 0, 0, 0, 0};
                            franka::CartesianVelocities cv(zero_vel);
                            cv.motion_finished = true;
                            return cv;
                        }
                        // Cosine ramp down: 1 → 0
                        ramp = 0.5 * (1.0 + std::cos(M_PI * dt / ramp_ticks));
                    } else if (tick < ramp_ticks) {
                        // Cosine ramp up: 0 → 1
                        ramp = 0.5 * (1.0 - std::cos(M_PI * tick / ramp_ticks));
                    }

                    vx *= ramp; vy *= ramp; vz *= ramp;
                    wx *= ramp; wy *= ramp; wz *= ramp;

                    prev_cmd = {vx, vy, vz, wx, wy, wz};

                    // ── 7. Update telemetry under lock ───────────────────
                    {
                        std::lock_guard<std::mutex> lk(state.mtx);
                        state.current_pose     = rs.O_T_EE;
                        state.current_q        = rs.q;
                        state.target_pose      = gp;
                        state.cmd_success_rate = rs.control_command_success_rate;
                    }
                    ++tick;

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

        // ── Handle pending ResetToJoints request ─────────────────────────────
        //
        // robot.control() returned normally (ramp-down completed).
        // If a reset was requested, execute the joint-space move and then
        // re-enter the Cartesian velocity loop with goal_pose = new EE pose.
        if (state.reset_requested.load()) {
            std::array<double, 7> rq;
            double rspeed, rdur;
            {
                std::lock_guard<std::mutex> lk(state.mtx);
                rq     = state.reset_q;
                rspeed = state.reset_speed;
                rdur   = state.reset_max_duration;
                state.ready = false;
            }

            std::cout << "[cartesian_server] Reset: moving to joint pose (speed="
                      << rspeed << ", max_dur=" << rdur << " s)..." << std::endl;

            try {
                move_to_joint_pose(robot, rq, rspeed, rdur);

                // Re-seed shared state from new pose.
                franka::RobotState rs0 = robot.readOnce();
                {
                    std::lock_guard<std::mutex> lk(state.mtx);
                    state.current_pose = rs0.O_T_EE;
                    state.current_q    = rs0.q;
                    state.goal_pose    = rs0.O_T_EE;
                    state.target_pose  = rs0.O_T_EE;
                    state.ready        = true;
                    state.reset_requested = false;
                    state.reset_complete  = true;
                }
                std::cout << "[cartesian_server] Reset complete." << std::endl;
            } catch (const franka::Exception& e) {
                std::cerr << "[cartesian_server] Reset failed: " << e.what() << std::endl;
                try { robot.automaticErrorRecovery(); } catch (...) {}
                {
                    std::lock_guard<std::mutex> lk(state.mtx);
                    state.reset_error     = e.what();
                    state.reset_requested = false;
                    state.reset_complete  = true;
                    state.ready           = true;
                }
            }
            state.reset_cv.notify_all();
            // Re-enter the control loop immediately (goal_pose = current pose,
            // so PD error ≈ 0 and robot holds position).
            continue;
        }
    }

    std::cout << "[cartesian_server] Control loop exited." << std::endl;
    if (g_server) g_server->Shutdown();
    return 0;
}
