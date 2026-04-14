# VRController — Pose Math Reference

This document explains every matrix operation inside `scripts/vr_controller.py`, in the order they are applied.

---

## What the class outputs

`get_pose_delta()` returns:

| Output | Type | Meaning |
|--------|------|---------|
| `pos_delta` | `np.ndarray[3]` | Controller displacement from origin, in calibrated VR frame (metres) |
| `rot_delta` | `np.ndarray[3,3]` | Rotation relative to origin, in calibrated VR frame |
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

- **Frame:** Oculus tracking space — the room-scale coordinate system established when the headset was powered on.
- **Parsing:** The APK streams 16 numbers row-by-row, so `OculusReader.process_data()` fills `transform[row, col]` directly into a standard numpy row-major array. No transformation is applied.

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

## Step 3 — Origin capture

**Trigger:** Every time the grip trigger toggles (press or release), `_reset_origin` is set `True`. On the next call to `get_pose_delta()`:

```python
if self._reset_origin:
    self._vr_origin_pos = cur_pos    # T_global[:3, 3]
    self._vr_origin_rot = cur_rot    # T_global[:3, :3]
    self._reset_origin = False
    self._origin_just_reset = True
    return None, None, None          # caller must capture robot origin this frame
```

`cur_pos` and `cur_rot` are the translation and rotation extracted from `T_global` at the moment of grip-press. The caller (`simple_teleop_direct.py`) uses the `origin_just_reset` flag to simultaneously capture the robot EEF pose as the robot origin.

---

## Step 4 — Delta computation

On every subsequent call while the origin is valid:

### Position delta

```python
pos_delta = cur_pos - self._vr_origin_pos
```

```
Δt = t_global(now) − t_global(origin)  ∈ ℝ³
```

Straight vector subtraction in the calibrated VR frame. Units are metres (inherited from the Oculus tracking system). No scaling is applied.

### Rotation delta

```python
rot_delta = cur_rot @ self._vr_origin_rot.T
```

```
ΔR = R_global(now) · R_global(origin)ᵀ
   = R_global(now) · R_global(origin)⁻¹
```

This is the rotation that maps the orientation at origin to the current orientation. It satisfies:

```
R_global(now) = ΔR · R_global(origin)
```

`ΔR = I` when the controller has not rotated from the origin, and grows as the operator tilts/twists the controller.

---

## How the caller uses the delta (simple_teleop_direct.py)

The robot EEF target is assembled from the captured robot origin `T_robot` and the VR deltas:

```python
T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta
```

**Translation:** additive in base frame — `Δt` from VR directly offsets the robot's position at capture time.

**Rotation:** `ΔR · R_robot_origin` — left-multiply applies the VR rotation change in the **base frame** (extrinsic convention). The effect is that "tilt the controller forward" always moves the EEF in the robot's base-frame forward direction, regardless of the EEF's current wrist orientation.

---

## Summary of matrix chain

```
T_vr          — raw Oculus tracking pose (room frame)
   ↓  vr_to_global_mat = T_vr_cal⁻¹
T_global      — pose relative to calibration reference
   ↓  subtract origin (captured at grip-press)
Δt, ΔR        — incremental position and rotation
   ↓  add to robot origin (captured simultaneously)
T_target      — absolute EEF target in robot base frame
```
