# simple_teleop_direct.py — Script Reference

Teleoperation script that maps Meta Quest 3 controller motion to Franka FR3 end-effector motion, using direct Cartesian pose control over gRPC. No Polymetis, no ZeroRPC, no IK solver.

---

## Pipeline

```
Meta Quest 3
    │  pose matrices via ADB logcat
    ▼
VRController
    │  pos_delta (ℝ³, metres), rot_delta (SO(3))
    ▼
simple_teleop_direct.py
    │  absolute target pose T_target (SE(3), 4×4)
    ▼
FrankaDirectClient  (gRPC, ~15 Hz)
    │  column-major 16-float pose list
    ▼
franka_server_cartesian.cpp  (1 kHz PD loop)
    ▼
Franka FR3 end-effector
```

---

## Prerequisites

1. Build the Cartesian server inside Docker:
   ```
   docker exec <container> bash /app/droid/franka_direct/build.sh
   ```
2. Generate Python gRPC stubs:
   ```
   bash franka_direct/python/generate_stubs.sh
   ```
3. Launch the server (do **not** run `launch_robot.sh` at the same time):
   ```
   docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
   ```
4. Connect Oculus Quest 3 via USB and start the teleop APK manually.

---

## Arguments

| Flag | Default | Description |
|------|---------|-------------|
| `--left` | false | Use left controller instead of right |
| `--no_reset` | false | Skip robot reset to home position |
| `--hz` | 15 | Control loop frequency (Hz) |
| `--host` | `192.168.1.6` | franka_server hostname/IP |
| `--port` | 50052 | franka_server gRPC port |

---

## Controls

| Input | Action |
|-------|--------|
| Grip trigger (hold) | Enable robot movement |
| Index trigger | Close gripper (release to open) |
| Joystick press | Recalibrate controller forward direction |
| A / X button | Stop — mark trajectory as **success** |
| B / Y button | Stop — mark trajectory as **failure** |
| `r` + Enter | Reset VR state (re-calibrate + new origin) |
| `q` + Enter | Quit |
| Ctrl+C | Emergency stop |

---

## Startup sequence

### Step 1 — Connect to franka_server
```python
client = FrankaDirectClient(host=args.host, port=args.port)
state = client.wait_until_ready(timeout=20.0)
```
Blocks until the gRPC server reports ready. Prints `cmd_success_rate` from the server as a health indicator.

### Step 2 — Initialize VRController
```python
vr = VRController(right_controller=right_controller)
```
Starts the background ADB polling thread inside `VRController`. The thread reads controller poses at ~50 Hz and handles orientation calibration automatically.

### Step 3 — Reset robot to home
```python
HOME_Q = [0.0, -π/5, 0.0, -4π/5, 0.0, 3π/5, 0.0]
client.reset_to_joints(HOME_Q, speed=0.2)
```
Moves the robot to a known safe joint configuration before teleoperation begins. Skipped with `--no_reset`.

---

## Main control loop (~15 Hz)

Each iteration:

### 1. Poll VR state
```python
info = vr.get_info()
```
Returns: `movement_enabled` (grip held), `controller_on`, `success`, `failure`.

### 2. Get current robot state
```python
state = client.get_robot_state()
```
Returns the current EEF pose as a column-major 16-float list, plus gripper width and error flag.

### 3. Compute and send arm target

```python
pos_delta, rot_delta, _ = vr.get_pose_delta()
```

**On grip toggle** (`vr.origin_just_reset == True`): capture the robot's current pose as the robot origin.
```python
robot_origin = pose16_to_mat(state["pose"])   # 4×4, robot base frame
```

**On subsequent frames** with valid delta:
```python
T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta
client.set_ee_target(mat_to_pose16(T_target))
```

- `pos_delta` is added directly to the robot origin's position (metres, in robot base frame).
- `rot_delta` is left-multiplied onto the robot origin's rotation — extrinsic rotation about fixed base frame axes.

See [vr_controller_math.md](vr_controller_math.md) for the full derivation of `pos_delta` and `rot_delta`.

### 4. Gripper control

```python
index_trig = vr._state["buttons"].get("rightTrig", (0.0,))[0]
want_closed = index_trig > 0.5
```

Threshold at 0.5 — binary open/close, not proportional. Commands are only sent on state change to avoid spamming the gripper:

```python
if want_closed and gripper_open:
    client.set_gripper_target(0.0, speed=0.1)     # close
elif not want_closed and not gripper_open:
    client.set_gripper_target(0.08, speed=0.1)    # open (80 mm max width)
```

### 5. Frequency regulation
```python
sleep_t = loop_period - elapsed
if sleep_t > 0:
    time.sleep(sleep_t)
```
Targets the Hz set by `--hz`. The actual achieved frequency is printed in the status line.

---

## Pose encoding helpers

```python
def pose16_to_mat(pose16):
    return np.array(pose16).reshape(4, 4, order='F')

def mat_to_pose16(T):
    return T.flatten(order='F').tolist()
```

The gRPC server (C++/Eigen) stores poses **column-major**. `order='F'` (Fortran order) handles the conversion in both directions. Using the wrong order would silently transpose the matrix.

---

## Comparison with VRPolicy (original DROID)

| | VRPolicy | simple_teleop_direct |
|---|---|---|
| Command type | Velocity delta (clipped to [-1,1]) | Absolute target pose |
| Rotation repr. | Quaternion / Euler | Rotation matrix |
| Action normalization | `pos_action_gain`, `_limit_velocity`, `.clip(-1,1)` | None — server PD loop handles limits |
| Axis reordering | `global_to_env_mat` ([-2,-1,-3,4]) | Removed |
| Backend | Polymetis + ZeroRPC | franka_server_cartesian (gRPC direct) |
| Control frequency | ~15 Hz (same) | ~15 Hz |
