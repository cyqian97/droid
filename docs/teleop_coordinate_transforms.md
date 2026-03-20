# Teleoperation Coordinate Transforms

This document traces every coordinate transform in the pipeline from the Meta Quest 3 VR controller to the Franka FR3 end-effector target pose.

---

## Pipeline Overview

```
Quest 3 APK
    │  raw 4×4 pose (VR tracking frame, row-major text stream)
    ▼
OculusReader.process_data()
    │  numpy 4×4 matrix  T_vr  (VR world frame)
    ▼
[Transform 1] vr_to_global_mat @ T_vr
    │  4×4 matrix  T_global  (orientation-calibrated VR frame)
    ▼
[Transform 2] global_to_env_mat @ T_global
    │  4×4 matrix  T_env  (robot workspace frame)
    ▼
[Transform 3] subtract VR origin  →  pos_delta, rot_delta
    │  Δpos ∈ ℝ³,  ΔR ∈ SO(3)
    ▼
[Transform 4] apply delta to robot origin  →  T_target
    │  4×4 SE(3) target pose (robot base frame)
    ▼
franka_server_cartesian (gRPC)
    │  Cartesian impedance PD loop @ 1 kHz
    ▼
Franka FR3 end-effector
```

All pose matrices are 4×4 homogeneous transforms `[R | t; 0 0 0 1]` where `R` is a 3×3 rotation matrix and `t` is a 3-vector translation in metres.

---

## Stage 0 — OculusReader: raw VR pose

**File:** [droid/oculus_reader/oculus_reader/reader.py](droid/oculus_reader/oculus_reader/reader.py#L139-L170)

The Quest 3 APK streams controller poses over ADB logcat as a space-separated list of 16 floats. `process_data()` parses them **row by row** into a standard numpy matrix:

```python
transform[r][c] = float(value)   # r increments after every 4 values
```

Result: `T_vr` is a 4×4 matrix in the **Oculus tracking frame** — the Quest 3's internal world coordinate system, which is roughly:
- **+X** right
- **+Y** up
- **+Z** toward the user (out of screen / OpenGL convention)

The pose represents the controller's position and orientation relative to the Quest 3 headset's tracking origin (the room-scale origin set at headset startup).

---

## Transform 1 — Orientation calibration: VR world → global

**File:** [scripts/vr_controller.py](scripts/vr_controller.py#L128-L138)

**Trigger:** Joystick press (`RJ` / `LJ` button), or automatically at startup until grip is first held.

```python
self.vr_to_global_mat = np.linalg.inv(raw)   # raw = T_vr at calibration moment
```

**Applied as:**
```python
T_global = vr_to_global_mat @ T_vr
```

**What it does:** `vr_to_global_mat` is the inverse of the controller's pose at the moment of calibration. Left-multiplying by it moves the reference frame to the controller's pose at that instant, making `T_global = I` (identity) at the moment of calibration and expressing all subsequent poses *relative to that reference orientation*.

**Why it's needed:** The Oculus tracking frame is anchored to the physical room, so the absolute orientation depends on where the headset was placed. This calibration removes that room-frame dependency, so the robot's response to "tilt the controller forward" is consistent regardless of how the operator is standing.

---

## Transform 2 — Axis reordering: global → robot env frame

**File:** [scripts/vr_controller.py](scripts/vr_controller.py#L27-L58)

```python
rmat_reorder = [-2, -1, -3, 4]          # default
global_to_env_mat = _reorder_mat(rmat_reorder)
```

`_reorder_mat([-2, -1, -3, 4])` builds the 4×4 permutation+sign matrix:

```
global_to_env_mat =
[ 0  -1   0   0 ]
[-1   0   0   0 ]
[ 0   0  -1   0 ]
[ 0   0   0   1 ]
```

Meaning (by row → output axis gets which input column, with sign):

| Output axis | Source       | Effect              |
|-------------|--------------|---------------------|
| env X       | −global Y    | swap X↔Y, negate    |
| env Y       | −global X    | swap X↔Y, negate    |
| env Z       | −global Z    | negate Z            |
| homogeneous | +homogeneous | unchanged           |

**Applied as:**
```python
T_env = global_to_env_mat @ T_global
       = global_to_env_mat @ vr_to_global_mat @ T_vr
```

This left-multiplies both the rotation and translation parts of the pose simultaneously:
- `env_pos = global_to_env_mat[:3,:3] @ global_pos`
- `env_R   = global_to_env_mat[:3,:3] @ global_R`

**Why it's needed:** The Oculus tracking frame's axes don't align with the robot's base frame. This fixed permutation/sign map aligns the coordinate conventions so that "move controller forward" maps to "move robot EEF forward" in robot space.

---

## Transform 3 — Origin capture & delta computation

**File:** [scripts/vr_controller.py](scripts/vr_controller.py#L192-L205)

### 3a. Origin capture

**Trigger:** Grip trigger is pressed or released (any toggle resets origin on next `get_pose_delta()` call).

```python
if self._reset_origin:
    self._vr_origin_pos = cur_pos    # T_env[:3, 3]
    self._vr_origin_rot = cur_rot    # T_env[:3, :3]
    self._reset_origin = False
    self._origin_just_reset = True
    return None, None, None          # caller captures robot origin this frame
```

The origin is the **env-frame position and rotation at the moment grip is first held.**

### 3b. Delta computation

```python
pos_delta = cur_pos - self._vr_origin_pos          # ℝ³, in robot env frame (metres)
rot_delta = cur_rot @ self._vr_origin_rot.T        # SO(3), relative rotation matrix
```

- **`pos_delta`**: straight vector subtraction — displacement of the controller from its origin position, already in the robot env frame.
- **`rot_delta`**: `R_cur @ R_origin⁻¹` — the rotation that takes the orientation at origin to the current orientation. Since `R` is orthogonal, `R⁻¹ = Rᵀ`.

Both quantities are **incremental**: they represent how much the controller has moved/rotated since the grip was pressed, not absolute poses.

---

## Transform 4 — Apply delta to robot origin: env frame → robot base frame

**File:** [scripts/simple_teleop_direct.py](scripts/simple_teleop_direct.py#L229-L237)

### 4a. Robot origin capture

When `vr.origin_just_reset` is true (fires once per grip-press), the current robot EEF pose is captured:

```python
robot_origin = pose16_to_mat(state["pose"])    # 4×4, robot base frame
```

`pose16_to_mat` decodes the 16-float column-major list from the gRPC server:

```python
def pose16_to_mat(pose16):
    return np.array(pose16).reshape(4, 4, order='F')   # Fortran/column-major
```

Note: the robot server stores poses **column-major** (matching Eigen/libfranka convention), so `order='F'` is required to reconstruct the correct matrix.

### 4b. Target pose construction

```python
T_target = np.eye(4)
T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta
```

- **Translation**: `t_target = t_robot_origin + Δpos` — the robot EEF target is the robot's position at capture time plus the VR displacement. The delta is already in robot base frame units (metres) due to Transform 2.
- **Rotation**: `R_target = ΔR @ R_robot_origin` — the target orientation is the robot's orientation at capture time rotated by the VR rotation delta. The left-multiply applies the delta in the current env frame (equivalent to post-multiplying in the body frame from the robot's perspective).

The resulting `T_target` is a 4×4 SE(3) pose in the **robot base frame**, and is sent to the server as a column-major 16-float list.

---

## Coordinate Frame Summary

| Frame name        | Origin & orientation                                          | When used                          |
|-------------------|---------------------------------------------------------------|------------------------------------|
| **VR tracking**   | Quest 3 room-scale origin; axes set at headset startup        | Raw APK output                     |
| **VR global**     | Controller pose at joystick-press moment (= identity there)   | After Transform 1                  |
| **Robot env**     | Axis-remapped global frame (X↔Y swapped, Z negated)           | After Transform 2; pos/rot deltas  |
| **Robot base**    | Franka FR3 base flange; libfranka/Eigen convention            | Robot state & target poses         |

---

## Key Implementation Notes

1. **Column-major vs row-major**: OculusReader outputs row-major numpy arrays (`transform[row, col]`). The gRPC server uses column-major 16-float lists. The `pose16_to_mat`/`mat_to_pose16` helpers handle the conversion with `order='F'`.

2. **Orientation calibration is continuous until grip is held**: `_reset_orientation` keeps updating `vr_to_global_mat` on every poll tick until the grip trigger is pressed *or* the joystick is released. This means the calibration "settles" on the last pose before the operator grips.

3. **Grip toggle resets origin each time**: Every grip press/release sets `_reset_origin = True`, which causes the next `get_pose_delta()` call to capture a new VR origin and signal the main loop to capture a new robot origin. This prevents accumulated drift between sessions.

4. **`rot_delta` left-multiply semantics**: `R_target = ΔR @ R_robot_origin` applies the VR rotation change in the **env frame** (extrinsic rotation). If it were `R_robot_origin @ ΔR`, it would apply the rotation in the robot's **body frame** (intrinsic). The current convention means tilting the controller forward always moves the EEF in the robot-base-frame's forward direction, regardless of the EEF's current orientation.
