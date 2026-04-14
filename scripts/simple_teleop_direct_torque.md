# simple_teleop_direct_torque.py — Reference

Teleoperation script that maps Meta Quest 3 controller motion to Franka FR3 joint targets, using a Python-side IK solver and a C++ joint torque controller. The IK runs at 15 Hz in Python; the torque PD loop runs at 1 kHz in C++.

---

## Pipeline

```
Meta Quest 3
    │  raw 4×4 pose matrices (ADB logcat)
    ▼
VRController  (50 Hz background thread)
    │  pos_delta ∈ ℝ³ (metres), rot_delta ∈ SO(3)
    ▼
simple_teleop_direct_torque.py  (15 Hz main loop)
    │
    ├─ compose delta + robot origin → T_target ∈ SE(3)
    │
    ├─ pose_to_cartesian_velocity()
    │    position error    → lin_vel ∈ [-1, 1]³  (normalised by 75 mm/step)
    │    rotation error    → rot_vel ∈ [-1, 1]³  (normalised by 0.15 rad/step)
    │
    ├─ RobotIKSolver.cartesian_velocity_to_joint_velocity()
    │    dm_robotics Cartesian6dVelocityEffector
    │    Jacobian-based velocity IK on MuJoCo FR3 model
    │    → joint_velocity ∈ ℝ⁷
    │
    ├─ RobotIKSolver.joint_velocity_to_delta()
    │    scales joint velocity → joint_delta (max 0.2 rad/step)
    │
    └─ q_target = q_current + joint_delta
    │
    ▼
FrankaDirectClient  (gRPC)
    │  SetJointTarget  →  7 joint positions in radians
    ▼
franka_server.cpp  (1 kHz torque PD loop)
    │  τ[i] = Kp[i]·(interp_q[i] − q[i]) − Kd[i]·dq[i] + coriolis[i]
    ▼
Franka FR3 joints
```

---

## Comparison with simple_teleop_direct.py

| | simple_teleop_direct | simple_teleop_direct_torque |
|---|---|---|
| **C++ server** | franka_server_cartesian | franka_server |
| **Command type** | EEF pose (16 floats, SE(3)) | Joint positions (7 floats) |
| **IK** | Built into C++ (Jacobian Cartesian velocity) | Python-side dm_robotics (Jacobian velocity IK) |
| **Joint limit avoidance** | Via libfranka internal controller | dm_robotics (0.3 rad margin) |
| **Nullspace control** | None | Weak pull toward q=0 (gain=0.025) |
| **Gripper** | Binary open/close at 0.5 threshold | Proportional: trigger → width |
| **Control rate** | 15 Hz Python → 1 kHz C++ | 15 Hz Python → 1 kHz C++ |

---

## IK Details

### Library

`droid/robot_ik/robot_ik_solver.py` — `RobotIKSolver`, which wraps:
- `dm_robotics.moma.effectors.cartesian_6d_velocity_effector.Cartesian6dVelocityEffector`
- MuJoCo FR3 model (`droid/robot_ik/franka/fr3.xml`)

### Algorithm

At each 15 Hz step:

1. **Pose error** (in robot base frame):
   ```
   Δp   = t_target − t_current          (position error, metres)
   R_e  = R_target · R_current^T        (rotation error matrix)
   ω    = axis_angle(R_e)               (rotation error vector, radians)
   ```

2. **Normalise to [-1, 1]** (RobotIKSolver convention):
   ```
   lin_vel = Δp / max_lin_delta          max_lin_delta = 0.075 m
   rot_vel = ω  / max_rot_delta          max_rot_delta = 0.15 rad
   ```
   Clip each norm to 1.0.

3. **Jacobian IK** (inside `cartesian_velocity_to_joint_velocity`):
   ```
   Δq_cart = [lin_vel · max_lin_delta; rot_vel · max_rot_delta]   (6D)
   J       = MuJoCo Jacobian at current joint state               (6×7)
   Δq      = (J^T J + λI)^{-1} J^T Δq_cart                       (damped pseudoinverse)
   ```
   With Tikhonov regularisation λ=0.01, up to 300 iterations, joint limits enforced.

4. **Scale to joint delta** (max 0.2 rad/step):
   ```
   joint_delta = Δq / max(|Δq_i| / max_joint_delta_i)   (rescaled if needed)
   ```

5. **New joint target**:
   ```
   q_target = q_current + joint_delta
   ```

### IK parameters (from RobotIKSolver)

| Parameter | Value |
|-----------|-------|
| `max_lin_delta` | 75 mm / step |
| `max_rot_delta` | 0.15 rad / step |
| `max_joint_delta` | 0.2 rad / step |
| `regularization_weight` | 0.01 (Tikhonov) |
| `nullspace_gain` | 0.025 (toward q=0) |
| `minimum_distance_from_joint_position_limit` | 0.3 rad |
| `joint_position_limit_velocity_scale` | 0.95 |
| `control_timestep_seconds` | 1/15 s |

---

## Torque controller (franka_server.cpp)

The C++ server runs a joint impedance PD controller at 1 kHz:

```
τ[i] = Kp[i] · (interp_q[i] − q[i])   ← position error
     − Kd[i] · dq[i]                   ← velocity damping
     + coriolis[i]                      ← feedforward (Coriolis + centrifugal)
τ[i] = clamp(τ[i], −tau_limit[i], +tau_limit[i])
```

`interp_q` chases `q_target` (received from Python via gRPC) at `max_step = 0.001 rad/tick`, providing smooth interpolation. libfranka adds gravity compensation automatically.

**Gains** (from `franka_direct/config/controller.yaml`):

| | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|---|---|---|---|---|---|---|---|
| Kp [N·m/rad] | 10 | 7.5 | 12.5 | 6.25 | 8.25 | 6.25 | 2.5 |
| Kd [N·m·s/rad] | 2 | 3 | 2.5 | 5 | 1.5 | 1 | 0.5 |

---

## Gripper control

Proportional mapping from index trigger:

```
gripper_width = (1 − trigger) × 0.08 m
```

- Trigger released (0) → 80 mm (fully open)
- Trigger fully pressed (1) → 0 mm (fully closed)

Commands are sent only when the target changes by more than 2 mm (deadband) to avoid constant gripper activity.

---

## Coordinate frames

All pose math is in the **robot base frame** (libfranka O frame):

| Quantity | Frame | Source |
|----------|-------|--------|
| `T_current` | Robot base | libfranka `O_T_EE` via gRPC |
| `T_target` | Robot base | Composed from VRController delta + robot origin |
| `Δp`, `ω` | Robot base | Computed from `T_target` and `T_current` |
| MuJoCo FK | Robot base | Matches libfranka (same FR3 model) |
| `q_target` | — | Joint space (7 angles in radians) |

The VRController calibration (`vr_to_global_mat`) ensures the Cartesian delta is expressed in the robot's workspace frame before it is applied to `T_robot_origin`. See [vr_controller_math.md](vr_controller_math.md) for the full derivation.

---

## Prerequisites

1. Build the torque server inside Docker:
   ```
   docker exec <container> bash /app/droid/franka_direct/build.sh
   ```
2. Generate Python gRPC stubs:
   ```
   bash franka_direct/python/generate_stubs.sh
   ```
3. Launch the torque server:
   ```
   docker exec <container> bash /app/droid/franka_direct/launch_server.sh
   ```
4. Connect Quest 3 via USB and start the teleop APK manually.

---

## Arguments

| Flag | Default | Description |
|------|---------|-------------|
| `--left` | false | Use left controller |
| `--no_reset` | false | Skip home reset |
| `--hz` | 15 | Control loop frequency (Hz) |
| `--host` | `192.168.1.6` | franka_server hostname/IP |
| `--port` | 50052 | gRPC port |
