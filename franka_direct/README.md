# franka_direct -- Direct libfranka Robot Control

Bypass Polymetis and drive a Franka FR3 directly via **libfranka + gRPC**.

Two C++ servers implement 1 kHz real-time control loops; a Python client
sends commands and reads state over gRPC from any machine on the network.

---

## Why This Exists

The original DROID control path routes through **Polymetis**, whose C++
backend (`franka_panda_client`) makes a blocking gRPC `ControlUpdate` call
**inside** the 1 kHz real-time callback:

```
Laptop  ──ZeroRPC(:4242)──▶  NUC run_server.py (FrankaRobot)
                                    │
                              Polymetis RobotInterface
                                    │  gRPC(:50051)
                                    ▼
                              franka_panda_client  (C++ 1 kHz RT loop)
                                    │
                              ┌─────┴─────┐
                              │ gRPC call │ ← ~0.53 ms
                              │ Jacobian  │ ← ~0.35 ms
                              └─────┬─────┘
                                    │  total ≈ 0.88 ms / 1.0 ms budget
                                    ▼
                              ~50 % communication_constraints_violation
```

`franka_direct` eliminates this by keeping **gRPC entirely outside** the
1 kHz loop.  The real-time callback reads shared memory (mutex-protected),
and gRPC threads write to it asynchronously:

```
Laptop  ──gRPC(:50052)──▶  franka_server*.cpp
                                │
                          ┌─────┴─────────────────┐
                          │  gRPC threads         │  ← any rate
                          │  (SetEETarget, etc.)  │
                          └─────┬─────────────────┘
                                │  mutex write
                                ▼
                          ┌──────────────────────┐
                          │  SharedState struct  │
                          └─────┬────────────────┘
                                │  mutex read (< 1 µs)
                                ▼
                          ┌──────────────────────┐
                          │  1 kHz RT callback   │  ← all local math
                          │  robot.control(...)  │
                          └──────────────────────┘
```

Result: **0 % constraint violations**, deterministic 1 kHz control.

---

## Quick Start

### 1. Build (inside Docker)

```bash
docker exec <container> bash /app/droid/franka_direct/build.sh
```

Produces `build/franka_server` and `build/franka_server_cartesian`.

### 2. Generate Python stubs (on the laptop)

```bash
bash franka_direct/python/generate_stubs.sh
```

### 3. Launch a server

**Joint torque server** (for teleoperation with external IK):

```bash
docker exec <container> bash /app/droid/franka_direct/launch_server.sh
```

**Cartesian velocity server** (for direct EE pose control):

```bash
docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
```

> Do **not** run `launch_robot.sh` (Polymetis) at the same time.

### 4. Run a demo

```bash
# Joint space: slowly rotate joint 3
python scripts/simple_joint_direct.py --joints 3 --delta_deg 0.1

# Cartesian: move 50 mm down in z
python scripts/simple_pose_direct.py --z_mm -50

# VR teleoperation (requires Oculus Quest 3 + IK solver)
python scripts/simple_teleop_direct.py
```

---

## Communication Architecture

The diagram below shows the end-to-end communication pipeline and which
files participate at each layer.

```
┌─────────────────────────────────────────────────────────────────────┐
│  LAPTOP  (Python, any machine on the network)                       │
│                                                                     │
│  scripts/simple_pose_direct.py      Application logic: builds a     │
│  scripts/simple_joint_direct.py     target (pose or joint), calls   │
│  scripts/simple_teleop_direct.py    FrankaDirectClient methods.     │
│          │                                                          │
│          │  import                                                  │
│          ▼                                                          │
│  python/franka_direct_client.py     Thin gRPC wrapper: serialises   │
│          │                          Python lists into protobuf      │
│          │                          messages and sends RPCs.        │
│          │                                                          │
│          │  uses generated stubs                                    │
│          ▼                                                          │
│  python/franka_control_pb2.py       Auto-generated from the .proto  │
│  python/franka_control_pb2_grpc.py  by generate_stubs.sh.           │
│                                                                     │
└──────────┬──────────────────────────────────────────────────────────┘
           │
           │  gRPC over TCP  (:50052)
           │
┌──────────▼──────────────────────────────────────────────────────────┐
│  NUC  (Docker container, C++ servers)                               │
│                                                                     │
│  proto/franka_control.proto         Single source of truth for the  │
│          |                          gRPC service: defines all 6     │
│          |                          RPCs and message types.         │
│          │                                                          │
│          │  compiled into server binary by CMake                    │
│          ▼                                                          │
│  src/franka_server.cpp              Joint torque server: receives   │
│     OR                              SetJointTarget RPCs, runs a PD  │
│  src/franka_server_cartesian.cpp    torque controller at 1 kHz.     │
│          |                          Cartesian velocity server:      │
│          |                          receives SetEETarget RPCs, runs │
│          |                          a PD velocity controller at     │
│          |                          1 kHz.  Only one server runs    │
│          |                          at a time.                      │
│          │                                                          │
│          │  libfranka  robot.control()  at 1 kHz                    │
│          ▼                                                          │
│  Franka FR3 robot hardware                                          │
└─────────────────────────────────────────────────────────────────────┘
```

**Code generation from `franka_control.proto`:**
The `.proto` file is the single source of truth, but each side generates its
own language-specific code independently:
- **C++ (NUC):** `CMakeLists.txt` invokes `protoc` at **build time** —
  the generated `.grpc.pb.cc` / `.pb.cc` files are compiled directly into
  the server binary.  This happens automatically when you run `build.sh`.
- **Python (laptop):** `generate_stubs.sh` invokes `grpc_tools.protoc` to
  produce `franka_control_pb2.py` and `franka_control_pb2_grpc.py`.  You
  must run this manually after any `.proto` change.

**Data path for a single `set_ee_target()` call:**

1. **Python script** constructs a 4x4 pose matrix and calls
   `client.set_ee_target(pose_16)`.
2. **`franka_direct_client.py`** packs the 16 doubles into an `EETarget`
   protobuf message and sends a `SetEETarget` RPC over TCP to port 50052.
3. **`franka_server_cartesian.cpp`** receives the RPC on a gRPC thread,
   writes the new goal pose into `SharedState` (mutex), and returns
   `CommandResult` immediately.
4. The **1 kHz RT callback** (same `.cpp` file, inside `robot.control()`)
   reads `SharedState`, computes a PD velocity command, and sends it to the
   robot via libfranka.

---

## File Details

### `proto/franka_control.proto` — gRPC Interface Contract

Defines the `FrankaControl` gRPC service and all message types.  Both the
C++ servers and the Python stubs are generated from this single file.
Edit this when adding new RPCs or fields.

**RPCs:**

| RPC | Request | Response | Description |
|-----|---------|----------|-------------|
| `SetJointTarget` | `JointTarget` | `CommandResult` | Set 7 joint positions (rad) |
| `SetEETarget` | `EETarget` | `CommandResult` | Set 4x4 EE pose (col-major) |
| `SetGripperTarget` | `GripperTarget` | `CommandResult` | Set gripper width (m) |
| `GetRobotState` | `Empty` | `RobotState` | Read all robot + gripper state |
| `ResetToJoints` | `JointResetTarget` | `CommandResult` | Blocking joint move (cosine profile) |
| `Stop` | `Empty` | `CommandResult` | Graceful shutdown |

**Message types:**

**`JointTarget`** -- 7 joint angles in radians.
```
repeated double q = 1;
```

**`EETarget`** -- 4x4 homogeneous transform, column-major (same as libfranka `O_T_EE`).
```
repeated double pose = 1;   // 16 doubles
```

Column-major layout:
```
Index:  [0] R00   [4] R01   [8] R02   [12] tx
        [1] R10   [5] R11   [9] R12   [13] ty
        [2] R20   [6] R21  [10] R22   [14] tz
        [3]  0    [7]  0   [11]  0    [15]  1
```

**`GripperTarget`**
```
double width = 1;   // [0, 0.08] m -- 0 = closed, 0.08 = open
double speed = 2;   // [0.01, 0.2] m/s -- 0 uses default 0.1
```

**`JointResetTarget`**
```
repeated double q = 1;      // 7 joint angles (rad)
double speed = 2;            // fraction of max velocity [0..1], default 0.3
double max_duration = 3;     // seconds, default 5.0
```

**`RobotState`**
```
repeated double pose        = 1;   // O_T_EE col-major (16 doubles)
repeated double q           = 2;   // actual joint positions (7 doubles)
double cmd_success_rate     = 3;   // libfranka control_command_success_rate
bool   ready                = 4;   // true once control loop is running
string error                = 5;   // non-empty on fatal error
repeated double target_q    = 6;   // interpolated joint command (joint server)
double gripper_width        = 7;   // actual gripper opening [m]
bool   gripper_grasping     = 8;   // true while gripping
repeated double target_pose = 9;   // commanded EE pose (Cartesian server)
```

---

### `python/franka_direct_client.py` — Python gRPC Client

A thin class (`FrankaDirectClient`) that wraps every RPC in a method
returning native Python types (`list`, `dict`, `bool`).  Works with either
server variant.  Imported by all demo scripts.

```python
from franka_direct_client import FrankaDirectClient

client = FrankaDirectClient(host="192.168.1.6", port=50052, timeout=5.0)
```

| Method | Parameters | Returns | Notes |
|--------|-----------|---------|-------|
| `get_robot_state()` | -- | `dict` | See keys below |
| `wait_until_ready(timeout, poll_hz)` | 10.0, 5.0 | `dict` | Blocks until `ready=True` |
| `set_joint_target(q_7)` | `list[7]` rad | `(bool, str)` | Joint server only |
| `set_ee_target(pose_16)` | `list[16]` col-major | `(bool, str)` | Cartesian server only |
| `set_gripper_target(width_m, speed_mps)` | m, m/s | `(bool, str)` | 0.0 = close, 0.08 = open |
| `reset_to_joints(q_7, speed, max_duration, timeout)` | rad, [0..1], s, s | `(bool, str)` | Blocking joint move |
| `stop()` | -- | `bool` | Graceful shutdown |
| `close()` | -- | -- | Close gRPC channel |

**State dict keys** (returned by `get_robot_state()` and `wait_until_ready()`):

| Key | Type | Description |
|-----|------|-------------|
| `pose` | `list[16]` | Actual measured O_T_EE (col-major) |
| `q` | `list[7]` | Actual measured joint positions (rad) |
| `target_q` | `list[7]` | Interpolated joint command (joint server) |
| `target_pose` | `list[16]` | Commanded EE pose (Cartesian server) |
| `cmd_success_rate` | `float` | libfranka success rate [0..1] |
| `ready` | `bool` | True once control loop is running |
| `error` | `str` | Non-empty on fatal error |
| `gripper_width` | `float` | Actual gripper opening (m) |
| `gripper_grasping` | `bool` | True while gripping |

**Context manager:**
```python
with FrankaDirectClient(host="192.168.1.6") as client:
    state = client.wait_until_ready()
    client.set_ee_target(pose_16)
# Automatically calls stop() and close()
```

---

### `python/franka_control_pb2.py`, `franka_control_pb2_grpc.py` — Generated Stubs

Auto-generated by `generate_stubs.sh` (which runs `grpc_tools.protoc`).
Provides protobuf message classes and the `FrankaControlStub` gRPC class
used by `franka_direct_client.py`.  Do not edit by hand.

---

### `src/franka_server.cpp` — Joint Torque Server

Connects to the robot via libfranka, runs a 1 kHz `robot.control()` callback
that returns `franka::Torques`.  Receives `SetJointTarget` RPCs on gRPC
threads, interpolates toward the goal at 1 kHz, and applies a PD torque
controller with Coriolis feedforward.  Also handles gripper commands on a
dedicated thread.

**Thread architecture:**

```
┌─────────────────────────────┐
│  Main thread                │
│  robot.control() at 1 kHz   │ → franka::Torques
│  (torque PD controller)     │
└──────────┬──────────────────┘
           │ SharedState (mutex)
┌──────────┴──────────────────┐
│  gRPC threads               │
│  SetJointTarget             │
│  GetRobotState              │
└─────────────────────────────┘
┌─────────────────────────────┐
│  Gripper thread             │
│  franka::Gripper (blocking) │
└─────────────────────────────┘
```

**Joint interpolation:**

The server receives joint position targets at an external rate (e.g. 25 Hz)
but must command the robot at 1 kHz.  A linear interpolator bridges the gap:

```
At each 1 kHz tick:
    step = goal_q[i] - interp_q[i]
    step = clamp(step, -max_step, +max_step)   // default: 0.001 rad/tick = 1.0 rad/s
    interp_q[i] += step
```

**PD torque controller:**

Matches the Polymetis `DefaultController` (joint impedance) algorithm:

```
tau[i] = Kp[i] * (interp_q[i] - q[i])     // position error
       - Kd[i] * dq[i]                     // velocity damping
       + coriolis[i]                        // Coriolis + centrifugal feedforward
tau[i] = clamp(tau[i], -tau_limit[i], +tau_limit[i])
```

**Important:** libfranka adds gravity compensation automatically in torque
control mode.  Do **not** add a gravity term -- this matches Polymetis
`JointImpedanceControl`.

A first-order low-pass filter smooths the torque output:

```
alpha = dt / (dt + 1 / (2*pi*f_cutoff))     // dt = 0.001 s, f_cutoff = 100 Hz
tau_filtered = alpha * tau_raw + (1 - alpha) * tau_prev
```

**Startup / recovery:**

1. gRPC server starts immediately.
2. Main thread connects to robot, calls `readOnce()`, sets `ready=true`.
3. Main thread **waits** for the first `SetJointTarget` before entering `robot.control()`.
4. On `ControlException`: call `automaticErrorRecovery()`, wait for new command.
5. Seed `interp_q` from `robot.readOnce()` before each `robot.control()` entry.

**Configuration** (`config/controller.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp` | `[10, 7.5, 12.5, 6.25, 8.25, 6.25, 2.5]` | Stiffness gains [N*m/rad] |
| `kd` | `[2, 3, 2.5, 5, 1.5, 1, 0.5]` | Damping gains [N*m*s/rad] |
| `tau_limit` | `[87, 87, 87, 87, 12, 12, 12]` | Torque clamp [N*m] |
| `max_step` | `0.001` | Max interpolation step [rad/tick] |
| `lpf_cutoff` | `100.0` | Torque LPF cutoff [Hz] |
| `gripper_force` | `20.0` | Grasp force [N] |
| `gripper_eps_in` | `0.08` | Grasp inner tolerance [m] |
| `gripper_eps_out` | `0.08` | Grasp outer tolerance [m] |

---

### `src/franka_server_cartesian.cpp` — Cartesian Velocity Server

Same architecture as the joint server, but the RT callback returns
`franka::CartesianVelocities` instead of `franka::Torques`.  libfranka's
internal joint impedance controller converts velocity commands to torques.

**PD velocity controller:**

At each 1 kHz tick, the callback computes a Cartesian velocity command from
the pose error between the desired and current end-effector frames:

*Position (per-axis gains):*

```
vx = kp_x * (x_desired - x_current) - kd_x * vx_current
vy = kp_y * (y_desired - y_current) - kd_y * vy_current
vz = kp_z * (z_desired - z_current) - kd_z * vz_current
```

Per-axis gains `kp_x`, `kp_y`, `kp_z` (and `kd_*`) can be set individually
in the YAML config.  If omitted, they fall back to `kp_lin` / `kd_lin`.

*Orientation:*

```
wx = kp_rot * rot_err.x - kd_rot * omega_x
wy = kp_rot * rot_err.y - kd_rot * omega_y
wz = kp_rot * rot_err.z - kd_rot * omega_z
```

**Rotation error:**

The rotation error is the **axis-angle vector** extracted from
`R_err = R_desired * R_current^T`:

```
cos(angle) = (trace(R_err) - 1) / 2
sin(angle) = sin(acos(cos(angle)))

k = angle / (2 * sin(angle))        // for angle -> 0, k -> 0.5

rot_err = [ k * (R_err[2][1] - R_err[1][2]),
            k * (R_err[0][2] - R_err[2][0]),
            k * (R_err[1][0] - R_err[0][1]) ]
```

The result is a 3D vector whose **direction** is the rotation axis and
**magnitude** is the rotation angle in radians.

**Velocity clamping:**

Linear and angular velocities are clamped independently by norm:

```
if ||v_lin|| > max_lin_vel:
    v_lin *= max_lin_vel / ||v_lin||

if ||v_rot|| > max_rot_vel:
    v_rot *= max_rot_vel / ||v_rot||
```

This preserves direction while limiting speed.

**Velocity rate limiting (acceleration clamp):**

After absolute clamping, the per-tick velocity *change* is also limited.
This prevents large jumps when a new `SetEETarget` arrives suddenly:

```
delta = v_cmd - v_prev
if ||delta_lin|| > max_lin_accel * dt:
    delta_lin *= max_lin_accel * dt / ||delta_lin||
    v_cmd_lin = v_prev_lin + delta_lin

(same for angular with max_rot_accel)
```

**Velocity ramp (cosine profile):**

To avoid acceleration discontinuities that trigger libfranka reflexes, the
PD output is multiplied by a cosine ramp factor:

*Startup* (ramps 0 -> 1 over `ramp_duration` seconds):

```
ramp = 0.5 * (1 - cos(pi * tick / ramp_ticks))
```

*Shutdown* (ramps 1 -> 0 over `ramp_duration` seconds):

```
ramp = 0.5 * (1 + cos(pi * (tick - start) / ramp_ticks))
```

All 6 velocity components are multiplied by `ramp`.  The `motion_finished`
flag is set only after the ramp reaches zero.

*Why cosine?*  The cosine function has zero derivative at both endpoints,
so velocity starts and stops with zero acceleration.  A linear ramp would
produce a step in acceleration at t=0 and t=T.

**ResetToJoints (joint-space motion generator):**

The `ResetToJoints` RPC (and `init_q` at startup) uses a cosine-interpolated
joint position motion:

```
q(t) = q_start + (q_goal - q_start) * 0.5 * (1 - cos(pi * t / T))
```

Duration `T` is computed per-joint and synchronized to the slowest:

```
T_j = pi * |q_goal[j] - q_start[j]| / (2 * speed * dq_max[j])
T   = max(T_j for all j)
T   = clamp(T, 0.5, max_duration)
```

where `dq_max` is the FR3 maximum joint velocity `[2.175, 2.175, 2.175,
2.175, 2.61, 2.61, 2.61]` rad/s.

Velocity and acceleration are exactly zero at both endpoints, preventing
motion generator discontinuity reflexes.

*Flow when called via RPC:*
1. gRPC handler sets `reset_requested` (atomic) and blocks on `reset_cv`.
2. The RT callback sees the flag and ramps down to zero velocity.
3. `robot.control()` returns normally.
4. The main loop calls `move_to_joint_pose()` to execute the joint move.
5. On completion, re-seeds `goal_pose = current O_T_EE` and re-enters the
   Cartesian velocity loop (PD error ~ 0, robot holds position).
6. `reset_cv` is signaled; the RPC returns to the client.

**Startup / recovery:**

Same pattern as the joint server:
1. Optional `init_q` joint move before Cartesian control.
2. Wait for first `SetEETarget` before entering `robot.control()`.
3. On `ControlException`: recover, wait for new command.
4. On `ResetToJoints`: ramp down, joint move, re-enter.

**Configuration** (`config/controller_cartesian.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_lin` | `12.0` | Fallback linear P gain [1/s] |
| `kd_lin` | `0.5` | Fallback linear D gain |
| `kp_x/y/z` | (use `kp_lin`) | Per-axis P gain overrides |
| `kd_x/y/z` | (use `kd_lin`) | Per-axis D gain overrides |
| `kp_rot` | `2.0` | Orientation P gain [1/s] |
| `kd_rot` | `0.3` | Angular velocity damping |
| `max_lin_vel` | `0.5` | Linear velocity clamp [m/s] |
| `max_rot_vel` | `2.5` | Angular velocity clamp [rad/s] |
| `max_lin_accel` | `5.0` | Max linear acceleration [m/s²] |
| `max_rot_accel` | `10.0` | Max angular acceleration [rad/s²] |
| `ramp_duration` | `0.5` | Cosine ramp duration [s] |
| `cutoff_freq` | `100.0` | libfranka internal LPF [Hz] |
| `init_q` | (disabled) | Initial joint pose [7 angles, rad] |
| `init_speed` | `0.3` | Init move speed fraction [0..1] |
| `init_duration` | `5.0` | Init move max duration [s] |
| `gripper_force` | `20.0` | Grasp force [N] |
| `gripper_eps_in` | `0.08` | Grasp inner tolerance [m] |
| `gripper_eps_out` | `0.08` | Grasp outer tolerance [m] |

Edit the YAML and restart the server to apply changes -- no rebuild needed.

---

### Build & Launch Scripts

| File | Role |
|------|------|
| `CMakeLists.txt` | CMake build configuration.  Links against gRPC, protobuf, libfranka, and yaml-cpp.  Compiles the `.proto` into C++ sources and builds both server binaries. |
| `build.sh` | Runs CMake + make inside the Docker container.  Activates the conda environment and configures `ldconfig` so libfranka's `.so` is discoverable.  Produces `build/franka_server` and `build/franka_server_cartesian`. |
| `launch_server.sh` | Launches the **joint torque server** inside Docker.  Kills any lingering Polymetis processes, activates conda, and `exec`s the binary.  Configurable via `ROBOT_IP`, `GRPC_ADDR`, `CONFIG_FILE` environment variables. |
| `launch_server_cartesian.sh` | Same as above, but launches the **Cartesian velocity server**. |
| `python/generate_stubs.sh` | Runs `grpc_tools.protoc` on `franka_control.proto` to regenerate the Python `_pb2` files.  Run on the laptop after any `.proto` change. |

---

### Demo Scripts (in `scripts/`)

| Script | Server | What It Does |
|--------|--------|--------------|
| `simple_joint_direct.py` | Joint | Sends sinusoidal joint position targets at 25 Hz.  Measures round-trip timing and prints command success rate.  Good for verifying the joint server works. |
| `simple_pose_direct.py` | Cartesian | Resets robot to a home joint pose (`ResetToJoints`), optionally tests the gripper, then sends a single EE pose offset (translation + rotation) and monitors tracking error until convergence.  Good for verifying the Cartesian server and tuning gains. |
| `simple_teleop_direct.py` | Joint | Full VR teleoperation pipeline: reads Oculus Quest 3 controller poses via `VRPolicy`, converts Cartesian velocity to joint targets via `RobotIKSolver`, and streams them to the joint server at ~25 Hz.  Handles gripper open/close from the VR trigger. |
