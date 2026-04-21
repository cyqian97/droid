# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Repo Is

DROID is a robot manipulation platform for controlling a Franka FR3 arm, collecting teleoperated demonstration data, and uploading it to AWS S3.

**Original DROID codebase** lives under `droid/` — the Python package with Polymetis-based robot control, ZeroRPC server interface, IK solver, camera utilities, GUI, and data collection pipeline.

**Chengyuan's additions** are `franka_direct/` and the scripts below — a new control backend built to fix the ~50% `communication_constraints_violation` rate caused by blocking gRPC inside Polymetis's 1 kHz RT callback. These are intended to eventually move to a separate repo.

The two backends:
1. **Polymetis (original DROID)**: Python → ZeroRPC → NUC `run_server.py` → Polymetis gRPC → `franka_panda_client` C++ → 1 kHz RT loop. ~50% `communication_constraints_violation` failures.
2. **franka_direct (Chengyuan's)**: Python → gRPC → C++ server → libfranka 1 kHz RT loop. gRPC is entirely outside the RT loop; 0% violations.

## Hardware

- **NUC** (192.168.1.6): Real-time Ubuntu, runs the robot control Docker container.
- **Laptop** (192.168.1.1): User-facing machine, runs client Docker container.
- **Franka FR3** (192.168.1.11): EtherCAT connected to NUC.
- **Oculus Quest 3**: Connected to laptop via ADB for VR teleoperation.

## Build Commands

### Python Package
```bash
pip install -e .                   # Install droid package
pip install -e ".[dev]"            # With black, ruff, ipython, pre-commit
pip install -e ".[postprocessing]" # With scikit-image for data postprocessing
```

### franka_direct C++ Servers (inside NUC Docker container)
```bash
docker exec <container> bash /app/droid/franka_direct/build.sh
```
Produces `franka_direct/build/franka_server` and `franka_direct/build/franka_server_cartesian`. Must rebuild inside Docker after any C++ or `.proto` changes.

### Python gRPC Stubs (after any .proto change)
```bash
bash franka_direct/python/generate_stubs.sh
```

### Linting
```bash
make check       # Check (black + ruff) without modifying
make autoformat  # Auto-fix (black + ruff)
```
Line length 121, target Python 3.8+.

## Running the System

### franka_direct — Chengyuan's Backend (Preferred)

**Launch server on NUC Docker container** — pick one:
```bash
docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh  # Cartesian velocity
docker exec <container> bash /app/droid/franka_direct/launch_server.sh             # Joint torque
```
Do NOT run Polymetis `launch_robot.sh` at the same time. Launch scripts `pkill` Polymetis automatically.

**Teleoperation from laptop:**
```bash
python scripts/simple_teleop_direct.py [--left] [--no_reset] [--hz 15]         # Cartesian
python scripts/simple_teleop_direct_torque.py [--left] [--no_reset] [--hz 15]  # Joint + IK
```

Optional ZED camera recording with `simple_teleop_direct_torque.py`:
```bash
python scripts/simple_teleop_direct_torque.py --cam0 <serial> --cam1 <serial> [--cam_fps 60] [--resolution HD720] [--out_dir recordings]
```
A/B (or X/Y on left controller) start/stop recording. Omit `--cam0`/`--cam1` to run without cameras.

**Test scripts:**
```bash
python scripts/simple_pose_direct.py --z_mm -50      # Move EE 50mm down
python scripts/simple_joint_direct.py --joints 3     # Sinusoidal joint 3 motion
```

Server env vars: `ROBOT_IP` (default `192.168.1.11`), `GRPC_ADDR` (default `0.0.0.0:50052`), `CONFIG_FILE`, `POLICY_HZ` (default `25`, joint server only).

### Full DROID Pipeline — Original Codebase (Polymetis)

NUC Docker must be running `run_server.py` (ZeroRPC port 4242). From laptop:
```bash
python scripts/main.py [--left_controller | --right_controller]  # Full GUI + data collection
python scripts/simple_teleop.py [--left] [--no_reset] [--hz 15]  # Simple teleop
python test_nuc_connection.py [NUC_IP]                            # Test ZeroRPC connection
```

## Architecture

### Network Layout
```
Laptop (192.168.1.1) ──── ZeroRPC :4242 ──── NUC (192.168.1.6)
Laptop               ──── gRPC    :50052 ──── NUC (franka_direct)
NUC                  ──── EtherCAT       ──── FR3 (192.168.1.11)
```

### franka_direct Control Path — Chengyuan's (Cartesian)
```
Quest 3 → VRController → pos_delta, rot_delta
  → absolute T_target (4×4) → FrankaDirectClient.set_ee_target()
  → gRPC → franka_server_cartesian.cpp SharedState
  → 1 kHz RT callback → franka::CartesianVelocities → FR3
```

### Polymetis Control Path — Original DROID
```
Quest 3 → VRPolicy → cartesian velocity [-1, 1]
  → ServerInterface.update_command() → ZeroRPC → NUC run_server.py
  → FrankaRobot → RobotIKSolver (dm_robotics MuJoCo) → joint targets
  → Polymetis RobotInterface → franka_panda_client C++ → FR3
```

### Key Design Details

**Pose encoding (franka_direct):** All poses are 4×4 homogeneous transforms stored as 16 doubles in **column-major** order (matching libfranka `O_T_EE`):
```python
pose16_to_mat = lambda p: np.array(p).reshape(4, 4, order='F')
mat_to_pose16 = lambda T: T.flatten(order='F').tolist()
```

**Joint torque formula** (franka_direct joint server, matching Polymetis DefaultController):
```
τ[i] = Kp[i]×(interp_q[i] − q[i]) − Kd[i]×dq[i] + coriolis[i]
```
- Kp = [40, 30, 50, 25, 35, 25, 10], Kd = [4, 6, 5, 5, 3, 2, 1]
- `model.coriolis()` is used (not gravity) — libfranka adds gravity internally
- Clamp to ±[87, 87, 87, 87, 12, 12, 12] Nm

**Why `franka::Torques` instead of `franka::JointPositions`:** `JointPositions` always triggers `joint_motion_generator_velocity/acceleration_discontinuity` at startup on FR3. Torque control bypasses the motion generator entirely.

**VR controller difference:**
- `VRPolicy` (Polymetis path): normalized velocity vectors `[-1, 1]` with axis remapping `rmat_reorder=[-2, -1, -3, 4]`.
- `VRController` (`scripts/vr_controller.py`, franka_direct path): raw `pos_delta` (metres) and `rot_delta` (SO(3)), applied to absolute robot origin pose.

**VRController yaw calibration (joystick press):** Projects the controller's body Y axis onto the global XY plane to extract yaw angle `θ = atan2(R[0,1], R[1,1])`, then builds `vr_to_global_mat = Rz(-θ)`. This cancels only the yaw offset (aligns body Y → global Y) without affecting pitch or roll. Full `inv(raw)` was intentionally avoided.

**Startup/recovery pattern** (franka_direct servers): Wait for first `SetJointTarget` before calling `robot.control()`. After fault recovery, wait for new target before re-entering. Seed `interp_q` from `robot.readOnce()` before each `robot.control()` call.

## Key Files

### Original DROID (not Chengyuan's)

| File | Purpose |
|------|---------|
| `droid/robot_env.py` | Top-level `RobotEnv` (gym.Env) |
| `droid/franka/robot.py` | `FrankaRobot` — Polymetis control |
| `droid/controllers/oculus_controller.py` | `VRPolicy` — VR controller outputting normalized velocity |
| `droid/misc/parameters.py` | All IPs, robot config, camera serial numbers |
| `droid/misc/server_interface.py` | ZeroRPC client → NUC `run_server.py` |
| `droid/robot_ik/robot_ik_solver.py` | dm_robotics Jacobian IK (MuJoCo FR3 model) |
| `scripts/server/run_server.py` | ZeroRPC server (runs on NUC) |
| `scripts/main.py` | Full data collection with GUI |
| `scripts/simple_teleop.py` | Polymetis-based teleoperation |

### Chengyuan's Code (franka_direct + scripts)

| File | Purpose |
|------|---------|
| `franka_direct/src/franka_server.cpp` | Joint torque gRPC server |
| `franka_direct/src/franka_server_cartesian.cpp` | Cartesian velocity gRPC server |
| `franka_direct/proto/franka_control.proto` | gRPC interface definition |
| `franka_direct/python/franka_direct_client.py` | Python gRPC client |
| `franka_direct/config/controller.yaml` | Joint torque gains (edit without rebuild) |
| `franka_direct/config/controller_cartesian.yaml` | Cartesian gains (edit without rebuild) |
| `franka_direct/build.sh` | CMake build script (run inside Docker) |
| `franka_direct/launch_server.sh` | Launch joint torque server |
| `franka_direct/launch_server_cartesian.sh` | Launch Cartesian velocity server |
| `scripts/vr_controller.py` | `VRController` — outputs raw pose deltas; joystick press resets yaw only |
| `scripts/zed_utils.py` | ZED camera helpers: `list_cameras()`, `open_camera()`, `CameraRecorder` (two-camera MP4 recording) |
| `scripts/simple_teleop_direct.py` | Teleoperation via Cartesian server |
| `scripts/simple_teleop_direct_torque.py` | Teleoperation via joint torque server + IK; optional ZED recording via A/B buttons |
| `scripts/simple_joint_direct.py` | Test: sinusoidal joint commands |
| `scripts/simple_downward_direct.py` | Test: move EE downward |
| `scripts/test_vr_readout.py` | Test: print raw VR controller output |
| `test_nuc_connection.py` | Test ZeroRPC connection to NUC |

## Configuration

`droid/misc/parameters.py` contains all hardware IPs, robot serial number, camera serial numbers, and sudo password. Edit here to adapt to different hardware.

Controller gains in `franka_direct/config/` can be edited and take effect on server restart (no rebuild needed).

## Submodules
```bash
git submodule update --init --recursive
```
- `droid/fairo/` — Polymetis and perception libraries
- `droid/oculus_reader/` — Oculus Quest ADB reader

## Docker

Two container types (configs in `.docker/`):
- **NUC**: Ubuntu Bionic, Polymetis conda env (`polymetis-local`), libfranka built from source. Runs `--privileged`, `--network=host`, `rtprio=99`.
- **Laptop**: Camera access, GUI, client code.

libfranka (in Docker): `/app/droid/fairo/polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka/build/`
