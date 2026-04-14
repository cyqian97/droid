# VRController — Pose Math Reference

This document explains every matrix operation inside `scripts/vr_controller.py`, in the order they are applied.

---

## What the class outputs

`get_pose_delta()` returns:

| Output | Type | Meaning |
|--------|------|---------|
| `pos_delta` | `np.ndarray[3]` | Controller displacement from origin, in robot env frame (metres) |
| `rot_delta` | `np.ndarray[3,3]` | Rotation relative to origin, in robot env frame |
| `gripper` | `float` | Raw index trigger value [0, 1] |

These are **incremental** quantities — they describe how much the controller has moved since the grip trigger was last pressed, not where it is in the world.

---

## Step 1 — Raw pose from OculusReader

`OculusReader` delivers a `4×4` homogeneous transform for each controller, keyed `'r'` / `'l'`:

```
T_vr ∈ SE(3),  shape (4, 4)

T_vr = [ R_vr | t_vr ]
        [  0   |  1   ]
```

- **Frame:** Oculus tracking space — the room-scale coordinate system established when the headset was powered on. Axes are approximately: +X right, +Y up, +Z toward user.
- **Parsing:** The APK streams 16 numbers row-by-row, so `OculusReader.process_data()` fills `transform[row, col]` directly. The result is a standard numpy row-major array.

---

## Step 2 — Orientation calibration: VR world → "global"

**When:** On startup (`_reset_orientation = True`), and again each time the joystick is pressed (`RJ` / `LJ`).

```python
self.vr_to_global_mat = np.linalg.inv(raw)   # raw = T_vr at calibration moment
```

```
vr_to_global_mat = T_vr_cal⁻¹ ∈ SE(3)
```

**Applied as:**

```
T_global = vr_to_global_mat @ T_vr
         = T_vr_cal⁻¹ @ T_vr
```

**What it does:** Expresses the controller's current pose *relative to its pose at calibration time*. At the moment of calibration `T_global = I`. As the controller moves, `T_global` grows away from identity.

**Why it matters:** Without this step, the absolute VR room orientation leaks into the robot commands. The operator's physical position in the room would affect which direction "forward" maps to on the robot. After calibration, only motion relative to the reference pose matters.

**Liveness:** `vr_to_global_mat` keeps updating on every poll tick while the joystick is held *and* grip is not held. It freezes once grip is pressed or the joystick is released:

```python
stop = cur_enabled or buttons.get(joy_key, False)
if stop:
    self._reset_orientation = False
```

This lets the operator hold the joystick, rotate to a comfortable position, then press grip — the last joystick-held pose becomes the permanent reference until the next recalibration.

---

## Step 3 — Axis reordering: "global" → robot env frame

**Configured at construction:**

```python
rmat_reorder = [-2, -1, -3, 4]   # default
self.global_to_env_mat = _reorder_mat(rmat_reorder)
```

`_reorder_mat(vec)` builds a permutation matrix with signs. For the default `[-2, -1, -3, 4]`:

```
Entry i of vec:  sign × (1-based source column index)

  i=0: -2  →  row 0 ←  −col 1
  i=1: -1  →  row 1 ←  −col 0
  i=2: -3  →  row 2 ←  −col 2
  i=3: +4  →  row 3 ←  +col 3

global_to_env_mat =
[ 0  -1   0   0 ]
[-1   0   0   0 ]
[ 0   0  -1   0 ]
[ 0   0   0   1 ]
```

This is a signed axis-permutation matrix `P ∈ O(3)` (embedded in 4D). Its effect on a vector:

```
env_x = −global_y
env_y = −global_x
env_z = −global_z
```

**Applied as:**

```
T_env = global_to_env_mat @ T_global
      = global_to_env_mat @ vr_to_global_mat @ T_vr
```

The full transform chain (left-multiplied, applied right-to-left):

```
T_env = P · T_vr_cal⁻¹ · T_vr
```

For the rotation and translation components separately:

```
R_env = P[:3,:3] · R_vr_cal⁻¹ · R_vr
t_env = P[:3,:3] · (T_vr_cal⁻¹ · t_vr + ...)
```

*(Because the full 4×4 left-multiply applies P to both the rotation columns and the translation column simultaneously.)*

**Why this ordering matters:** `P` is applied after `T_vr_cal⁻¹`, so the axis swap happens in the calibrated frame, not the raw VR frame. The calibration step first removes room-orientation dependency; then the axis swap aligns the resulting frame with robot base conventions.

---

## Step 4 — Origin capture

**Trigger:** Every time the grip trigger toggles (press or release), `_reset_origin` is set `True`. On the next call to `get_pose_delta()`:

```python
if self._reset_origin:
    self._vr_origin_pos = cur_pos    # T_env[:3, 3]
    self._vr_origin_rot = cur_rot    # T_env[:3, :3]
    self._reset_origin = False
    self._origin_just_reset = True
    return None, None, None          # caller must capture robot origin this frame
```

`cur_pos` and `cur_rot` are the translation and rotation extracted from `T_env` at the moment of grip-press. The caller (`simple_teleop_direct.py`) uses the `origin_just_reset` flag to simultaneously capture the robot EEF pose as the robot origin.

---

## Step 5 — Delta computation

On every subsequent call while the origin is valid:

### Position delta

```python
pos_delta = cur_pos - self._vr_origin_pos
```

```
Δt = t_env(now) − t_env(origin)  ∈ ℝ³
```

Straight vector subtraction in the robot env frame. Units are metres (inherited from the Oculus tracking system). No scaling is applied.

### Rotation delta

```python
rot_delta = cur_rot @ self._vr_origin_rot.T
```

```
ΔR = R_env(now) · R_env(origin)ᵀ
   = R_env(now) · R_env(origin)⁻¹
```

This is the rotation that maps the orientation at origin to the current orientation, expressed in the env frame. It satisfies:

```
R_env(now) = ΔR · R_env(origin)
```

`ΔR = I` when the controller has not rotated from the origin, and grows as the operator tilts/twists the controller.

---

## How the caller uses the delta (simple_teleop_direct.py)

The robot EEF target is assembled from the captured robot origin `T_robot` and the VR deltas:

```python
T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta
```

**Translation:** additive in base frame — `Δt` from VR directly offsets the robot's position at capture time. This works because `P` was chosen so that VR env-frame metres correspond to robot base-frame metres (same scale, aligned axes).

**Rotation:** `ΔR · R_robot_origin` — left-multiply applies the VR rotation change in the **base frame** (extrinsic convention). The effect is that "tilt the controller forward" always moves the EEF in the robot's base-frame forward direction, regardless of the EEF's current wrist orientation.

---

## Summary of matrix chain

```
T_vr          — raw Oculus tracking pose (room frame)
   ↓  vr_to_global_mat = T_vr_cal⁻¹
T_global      — pose relative to calibration reference
   ↓  global_to_env_mat = P  (axis permutation)
T_env         — pose in robot env frame
   ↓  subtract origin (captured at grip-press)
Δt, ΔR        — incremental position and rotation
   ↓  add to robot origin (captured simultaneously)
T_target      — absolute EEF target in robot base frame
```
