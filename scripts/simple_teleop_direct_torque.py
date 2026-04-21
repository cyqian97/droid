#!/usr/bin/env python3
"""
Teleoperation using VR controller + IK + joint torque control.

Pipeline:
  Quest 3 → VRController → pose delta → absolute EEF target T_target (SE(3))
  → pose_to_cartesian_velocity() → RobotIKSolver (dm_robotics, 15 Hz)
  → joint delta Δq → q_target → FrankaDirectClient (gRPC)
  → franka_server.cpp (1 kHz joint impedance torque loop)

IK details:
  - Uses dm_robotics Cartesian6dVelocityEffector with a MuJoCo FR3 model.
  - Jacobian-based velocity IK with Tikhonov regularisation (λ=0.01),
    joint position limit avoidance (0.3 rad margin), and weak nullspace
    control toward q=0 (gain=0.025).
  - Position error is capped at 75 mm/step; rotation at 0.15 rad/step.
  - Runs at exactly 15 Hz (same rate as the original DROID VRPolicy).

Gripper:
  Proportional — index trigger value maps linearly to opening width.
  Fully squeezed = 0 mm (closed), fully released = 80 mm (open).

Prerequisites:
  1. Build inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python gRPC stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch the torque server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server.sh
  4. Connect Oculus Quest 3 via USB; start the teleop APK manually.
  5. Run this script:
       python scripts/simple_teleop_direct_torque.py

Controls:
  Hold GRIP TRIGGER    → enable robot movement
  INDEX TRIGGER        → proportional gripper (squeeze = close)
  JOYSTICK press       → recalibrate controller orientation
  A (right) / X (left) → stop (success)
  B (right) / Y (left) → stop (failure)
  r + Enter            → reset VR state
  q + Enter            → quit
  Ctrl+C               → emergency stop
"""

import argparse
import os
import select
import signal
import sys
import time

import numpy as np

# ── Path setup ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "franka_direct", "python"))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

try:
    from franka_direct_client import FrankaDirectClient
except ImportError as e:
    print(f"[ERROR] Could not import FrankaDirectClient: {e}")
    print("Did you run:  bash franka_direct/python/generate_stubs.sh ?")
    sys.exit(1)

try:
    from droid.robot_ik.robot_ik_solver import RobotIKSolver
except ImportError as e:
    print(f"[ERROR] Could not import RobotIKSolver: {e}")
    print("Make sure dm_robotics and dm_control are installed.")
    sys.exit(1)

from zed_utils import CameraRecorder, ZED_AVAILABLE

from vr_controller import VRController


# ── Pose math helpers ────────────────────────────────────────────────────────

def pose16_to_mat(pose16):
    """Column-major 16 floats → 4×4 numpy array."""
    return np.array(pose16).reshape(4, 4, order='F')


def mat_to_pose16(T):
    """4×4 numpy array → column-major 16 floats list."""
    return T.flatten(order='F').tolist()


def rotation_error_vec(R_target, R_current):
    """Rotation error as axis-angle vector in robot base frame.

    Returns ω ∈ ℝ³ such that exp([ω]×) ≈ R_target @ R_current.T.
    ||ω|| = rotation angle in radians, ω/||ω|| = rotation axis.

    Parameters
    ----------
    R_target, R_current : np.ndarray[3, 3]
        Rotation matrices in robot base frame.
    """
    R_err = R_target @ R_current.T
    cos_a = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-10:
        return np.zeros(3)
    k = angle / (2.0 * np.sin(angle))
    return k * np.array([R_err[2, 1] - R_err[1, 2],
                          R_err[0, 2] - R_err[2, 0],
                          R_err[1, 0] - R_err[0, 1]])


def pose_to_cartesian_velocity(T_target, T_current, ik):
    """Convert pose error to normalized 6D Cartesian velocity for RobotIKSolver.

    RobotIKSolver expects velocity components in [-1, 1]:
      - lin_vel = 1.0 means "move at max_lin_delta (75 mm) this step"
      - rot_vel = 1.0 means "rotate at max_rot_delta (0.15 rad) this step"

    Position and rotation errors are scaled by these limits. The norm of
    each component is independently clipped to 1.0 before passing to the
    IK solver (which performs the same clipping internally).

    Parameters
    ----------
    T_target, T_current : np.ndarray[4, 4]
        Target and current EEF poses in robot base frame.
    ik : RobotIKSolver

    Returns
    -------
    np.ndarray[6]
        Normalized Cartesian velocity [lin_vel (3), rot_vel (3)].
    """
    # Position error in base frame (metres)
    p_err = T_target[:3, 3] - T_current[:3, 3]

    # Rotation error: axis-angle vector in base frame (radians)
    rot_err = rotation_error_vec(T_target[:3, :3], T_current[:3, :3])

    # Normalize to [-1, 1] by the per-step limits
    lin_vel = p_err   / ik.max_lin_delta
    rot_vel = rot_err / ik.max_rot_delta

    # Clip norms to 1.0 so a large error doesn't violate IK velocity limits
    lin_norm = np.linalg.norm(lin_vel)
    if lin_norm > 1.0:
        lin_vel /= lin_norm

    rot_norm = np.linalg.norm(rot_vel)
    if rot_norm > 1.0:
        rot_vel /= rot_norm

    return np.concatenate([lin_vel, rot_vel])


# ── Keyboard helper ──────────────────────────────────────────────────────────

def check_keyboard():
    """Non-blocking keyboard read. Returns stripped line or None."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


# ── Status printer ───────────────────────────────────────────────────────────

def print_status(step, enabled, pos_err_mm, rot_err_deg, hz, gripper):
    sys.stdout.write(
        f"\r[Step {step:>6}] "
        f"{'MOVING' if enabled else 'PAUSED':<7} | "
        f"pos_err={pos_err_mm:>6.1f}mm  "
        f"rot_err={rot_err_deg:>5.1f}deg | "
        f"Hz={hz:>5.1f} | "
        f"gripper={gripper*1000:>4.0f}mm    "
    )
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Franka FR3 teleoperation via VR + IK + joint torque control")
    parser.add_argument("--left",     action="store_true",
                        help="Use left controller (default: right)")
    parser.add_argument("--no_reset", action="store_true",
                        help="Skip robot reset to home position")
    parser.add_argument("--hz",       type=int, default=15,
                        help="Control loop frequency in Hz (default: 15)")
    parser.add_argument("--host",     type=str, default="192.168.1.6",
                        help="franka_server host (default: 192.168.1.6)")
    parser.add_argument("--port",     type=int, default=50052,
                        help="franka_server gRPC port (default: 50052)")

    g = parser.add_argument_group("camera recording (optional)")
    g.add_argument("--cam0",       type=int, default=None,
                   help="Serial number of first ZED camera")
    g.add_argument("--cam1",       type=int, default=None,
                   help="Serial number of second ZED camera")
    g.add_argument("--cam_fps",    type=int, default=15,
                   help="Camera capture and video FPS (default: 15)")
    g.add_argument("--resolution", type=str, default="HD720",
                   choices=["HD2K", "HD1080", "HD720", "VGA"])
    g.add_argument("--out_dir",    type=str, default="recordings",
                   help="Root directory for saved videos (default: recordings/)")
    args = parser.parse_args()

    right_controller = not args.left
    loop_period      = 1.0 / args.hz

    # ── Ctrl+C handler ────────────────────────────────────────────────────────
    running = True

    def _sigint(sig, frame):
        nonlocal running
        if not running:
            os._exit(1)
        print("\n\nCtrl+C detected. Stopping...")
        running = False

    signal.signal(signal.SIGINT, _sigint)

    print("=" * 58)
    print("Teleoperation  —  VR + IK + joint torque control")
    print("=" * 58)

    # === Step 1: Connect to franka_server =====================================
    print(f"\nConnecting to franka_server at {args.host}:{args.port} ...")
    client = FrankaDirectClient(host=args.host, port=args.port)
    try:
        state = client.wait_until_ready(timeout=20.0)
        print("[OK] franka_server ready")
        print(f"     cmd_success_rate = {state['cmd_success_rate']:.3f}")
    except (TimeoutError, RuntimeError) as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    # === Step 2: Initialize IK solver =========================================
    print("Initializing IK solver (dm_robotics Cartesian6dVelocityEffector) ...")
    try:
        ik = RobotIKSolver()
        print(f"[OK] IK solver ready  "
              f"(max_lin={ik.max_lin_delta*1000:.0f} mm/step, "
              f"max_rot={np.degrees(ik.max_rot_delta):.1f} deg/step, "
              f"max_joint={np.degrees(ik.max_joint_delta):.1f} deg/step)")
    except Exception as e:
        print(f"[FAIL] Could not initialize IK solver: {e}")
        sys.exit(1)

    # === Step 3: Initialize VR controller =====================================
    print("Initializing VR controller ...")
    try:
        vr = VRController(right_controller=right_controller)
        side = "right" if right_controller else "left"
        print(f"[OK] VR controller initialized ({side} hand)")
    except Exception as e:
        print(f"[FAIL] Could not initialize VR controller: {e}")
        sys.exit(1)

    # === Step 3.5: Initialize cameras (optional) ==============================
    recorder = None
    if args.cam0 is not None and args.cam1 is not None:
        if not ZED_AVAILABLE:
            print("[WARN] pyzed not installed — camera recording disabled.")
        else:
            try:
                recorder = CameraRecorder(
                    serial0=args.cam0, serial1=args.cam1,
                    fps=args.cam_fps, resolution=args.resolution,
                    out_dir=args.out_dir,
                )
                recorder.open()
            except RuntimeError as e:
                print(f"[WARN] Camera init failed: {e} — recording disabled.")
                recorder = None

    # === Step 4: Optionally reset robot to home ================================
    HOME_Q = [0.0, -np.pi / 5, 0.0, -4 * np.pi / 5, 0.0, 3 * np.pi / 5, 0.0]

    if not args.no_reset:
        print("Resetting robot to home position ...")
        ok, msg = client.reset_to_joints(HOME_Q, speed=0.2)
        if ok:
            print("[OK] Robot at home position")
        else:
            print(f"[WARN] Reset: {msg}")
    else:
        print("[SKIP] Robot reset skipped (--no_reset)")

    # === Step 5: Print controls ================================================
    btn_a = "A" if right_controller else "X"
    btn_b = "B" if right_controller else "Y"
    print()
    print("=" * 58)
    print("TELEOPERATION READY")
    print("=" * 58)
    print(f"  Hold GRIP TRIGGER    → move robot")
    print(f"  INDEX TRIGGER        → proportional gripper (squeeze = close)")
    print(f"  JOYSTICK press       → recalibrate orientation")
    if recorder is not None:
        print(f"  '{btn_a}'                  → start recording")
        print(f"  '{btn_b}'                  → stop and save recording")
    print(f"  Ctrl+C               → emergency stop")
    print(f"  r + Enter            → reset VR state")
    print(f"  q + Enter            → quit")
    print(f"  Control frequency:   {args.hz} Hz")
    print("=" * 58)
    print()

    # === Step 6: Main control loop ============================================
    step_count   = 0
    robot_origin = None    # 4×4, captured when VR origin resets
    prev_btn_a   = False
    prev_btn_b   = False

    GRIPPER_OPEN     = 0.08   # Franka Hand max width [m]
    GRIPPER_SPEED    = 0.1    # finger speed [m/s]
    GRIPPER_DEADBAND = 0.002  # only send command if change > 2 mm
    last_gripper_cmd = GRIPPER_OPEN

    while running:
        loop_start = time.time()
        step_count += 1

        # ── Camera: grab frame (frame-locked to control loop) ───────────────
        if recorder is not None:
            recorder.grab()

        # ── VR controller info ──────────────────────────────────────────────
        info = vr.get_info()

        # ── Recording buttons (edge-detect to avoid repeat triggers) ────────
        cur_a = info["success"]
        cur_b = info["failure"]
        if cur_a and not prev_btn_a:
            print(f"\n[BTN] {btn_a} pressed", end="")
            if recorder is not None:
                if not recorder.is_recording:
                    print(" → starting recording ...")
                    recorder.start()
                else:
                    print(" (already recording)")
            else:
                print(" (no recorder active)")
        if cur_b and not prev_btn_b:
            print(f"\n[BTN] {btn_b} pressed", end="")
            if recorder is not None:
                if recorder.is_recording:
                    print(" → stopping recording ...")
                    recorder.stop()
                else:
                    print(" (not recording)")
            else:
                print(" (no recorder active)")
        prev_btn_a = cur_a
        prev_btn_b = cur_b

        if not info["controller_on"]:
            sys.stdout.write("\r[WARN] VR controller lost. Waiting...          ")
            sys.stdout.flush()
            time.sleep(1)
            continue

        # ── Keyboard input ──────────────────────────────────────────────────
        key = check_keyboard()
        if key == "q":
            print("\n\n[DONE] 'q' pressed — quitting")
            break
        elif key == "r":
            print("\n\nResetting VR controller state ...")
            vr.reset_state()
            robot_origin = None
            print("VR state reset. Hold grip trigger to start again.")
            continue

        # ── Get current robot state ─────────────────────────────────────────
        state = client.get_robot_state()
        if state["error"]:
            print(f"\n[ERROR] Robot error: {state['error']}")
            break

        T_current = pose16_to_mat(state["pose"])

        # ── Arm: move when grip trigger held ─────────────────────────────────
        pos_err_mm  = 0.0
        rot_err_deg = 0.0

        if info["movement_enabled"]:
            pos_delta, rot_delta, _ = vr.get_pose_delta()

            # Capture robot EEF origin when VR origin resets
            if vr.origin_just_reset:
                robot_origin = T_current.copy()

            elif pos_delta is not None and robot_origin is not None:
                # ── Compose VR delta with robot origin → absolute EEF target ──
                # T_target.R = ΔR @ R_robot_origin  (extrinsic rotation, base frame)
                # T_target.t = t_robot_origin + Δt   (additive in base frame)
                T_target = np.eye(4)
                T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
                T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta

                # ── IK: pose error → Cartesian velocity → joint delta ──────────
                # 1. Compute normalised 6D Cartesian velocity from pose error.
                cart_vel = pose_to_cartesian_velocity(T_target, T_current, ik)

                # 2. dm_robotics IK: Cartesian velocity → joint velocity.
                #    Uses the MuJoCo FR3 model at current joint state.
                robot_state_dict = {
                    "joint_positions": state["q"],
                    "joint_velocities": state["dq"] if state["dq"] else [0.0] * 7,
                }
                joint_vel   = ik.cartesian_velocity_to_joint_velocity(cart_vel, robot_state_dict)

                # 3. Scale joint velocity → joint delta (max 0.2 rad/step).
                joint_delta = ik.joint_velocity_to_delta(joint_vel)

                # 4. New joint target = current q + IK delta.
                q_target = (np.array(state["q"]) + joint_delta).tolist()
                client.set_joint_target(q_target)

                # Tracking errors for status display
                pos_err_mm  = np.linalg.norm(T_target[:3, 3] - T_current[:3, 3]) * 1000.0
                rot_err_deg = np.degrees(np.arccos(np.clip(
                    (np.trace(T_target[:3, :3] @ T_current[:3, :3].T) - 1.0) / 2.0,
                    -1.0, 1.0)))

        # ── Gripper: proportional from index trigger ─────────────────────────
        # Trigger value ∈ [0, 1]: 0 = released (open), 1 = fully squeezed (close).
        # Gripper width = (1 - trigger) * GRIPPER_OPEN, sent only on > 2 mm change.
        trig_key   = "rightTrig" if right_controller else "leftTrig"
        index_trig = vr._state["buttons"].get(trig_key, (0.0,))
        if isinstance(index_trig, (tuple, list)):
            index_trig = index_trig[0]

        gripper_target = (1.0 - float(index_trig)) * GRIPPER_OPEN
        if abs(gripper_target - last_gripper_cmd) > GRIPPER_DEADBAND:
            client.set_gripper_target(gripper_target, GRIPPER_SPEED)
            last_gripper_cmd = gripper_target

        # ── Regulate frequency ──────────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = loop_period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

        actual_hz = 1.0 / max(time.time() - loop_start, 1e-6)
        print_status(step_count, info["movement_enabled"],
                     pos_err_mm, rot_err_deg, actual_hz, state["gripper_width"])

    # === Cleanup ==============================================================
    print("\nTeleoperation ended.")
    print(f"Total steps: {step_count}")
    if recorder is not None:
        recorder.close()   # saves any in-progress recording, closes cameras
    try:
        client.stop()
        client.close()
    except Exception:
        pass
    os._exit(0)


if __name__ == "__main__":
    main()
