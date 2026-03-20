#!/usr/bin/env python3
"""
Teleoperation using VR controller + direct Cartesian pose control.

No Polymetis, no ZeroRPC, no IK solver.  The pipeline is:
  Oculus Quest 3 → VRController → pose delta → absolute target pose
  → FrankaDirectClient (gRPC) → franka_server_cartesian.cpp (1 kHz PD loop)

Prerequisites:
  1. Build servers inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch the Cartesian server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
  4. Connect Oculus Quest 3 via ADB.
  5. Run this script:
       python scripts/simple_teleop_direct.py

Controls:
    Grip trigger (side)  : Hold to enable robot movement
    Index trigger         : Close/open gripper
    Joystick press       : Recalibrate forward direction
    A button (right) / X (left) : Stop (success)
    B button (right) / Y (left) : Stop (failure)
    Ctrl+C               : Emergency stop
    r + Enter            : Reset VR state
    q + Enter            : Quit
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

try:
    from franka_direct_client import FrankaDirectClient
except ImportError as e:
    print(f"[ERROR] Could not import FrankaDirectClient: {e}")
    print("Did you run:  bash franka_direct/python/generate_stubs.sh ?")
    sys.exit(1)

from vr_controller import VRController


# ── Helpers ───────────────────────────────────────────────────────────────────

def pose16_to_mat(pose16):
    """Column-major 16 floats → 4×4 numpy array."""
    return np.array(pose16).reshape(4, 4, order='F')


def mat_to_pose16(T):
    """4×4 numpy array → column-major 16 floats list."""
    return T.flatten(order='F').tolist()


def check_keyboard():
    """Non-blocking keyboard read. Returns stripped line or None."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def print_status(step, enabled, pos_delta, hz, pos, gripper):
    sys.stdout.write(
        f"\r[Step {step:>6}] "
        f"{'MOVING' if enabled else 'PAUSED':<7} | "
        f"{'Δpos=({:.3f}, {:.3f}, {:.3f})'.format(*pos_delta) if pos_delta is not None else 'Δpos=(---, ---, ---)' } | "
        f"Hz: {hz:>5.1f} | "
        f"xyz=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) | "
        f"gripper={gripper*100:.0f}mm    "
    )
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Franka FR3 teleoperation via VR + Cartesian server")
    parser.add_argument("--left",     action="store_true",
                        help="Use left controller (default: right)")
    parser.add_argument("--no_reset", action="store_true",
                        help="Skip robot reset to home position")
    parser.add_argument("--hz",       type=int,   default=15,
                        help="Control loop frequency in Hz (default: 15)")
    parser.add_argument("--host",     type=str,   default="192.168.1.6",
                        help="franka_server host (default: 192.168.1.6)")
    parser.add_argument("--port",     type=int,   default=50052,
                        help="franka_server gRPC port (default: 50052)")
    args = parser.parse_args()

    right_controller = not args.left
    loop_period      = 1.0 / args.hz

    # ── Ctrl+C handler ────────────────────────────────────────────────────────
    running = True

    def _sigint(sig, frame):
        nonlocal running
        if not running:
            # Second Ctrl+C → force exit immediately
            os._exit(1)
        print("\n\nCtrl+C detected. Stopping...")
        running = False

    signal.signal(signal.SIGINT, _sigint)

    print("=" * 55)
    print("Teleoperation  —  VR + Cartesian direct control")
    print("=" * 55)

    # === Step 1: Connect to franka_server_cartesian ============================
    print(f"\nConnecting to franka_server at {args.host}:{args.port} ...")
    client = FrankaDirectClient(host=args.host, port=args.port)
    try:
        state = client.wait_until_ready(timeout=20.0)
        print("[OK] franka_server ready")
        print(f"     cmd_success_rate = {state['cmd_success_rate']:.3f}")
    except (TimeoutError, RuntimeError) as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    # === Step 2: Initialize VR controller =====================================
    print("Initializing VR controller ...")
    try:
        vr = VRController(right_controller=right_controller)
        side = "right" if right_controller else "left"
        print(f"[OK] VR controller initialized ({side} hand)")
    except Exception as e:
        print(f"[FAIL] Could not initialize VR controller: {e}")
        sys.exit(1)

    # === Step 3: Optionally reset robot to home ================================
    HOME_Q = [0.0, -np.pi / 5, 0.0, -4 * np.pi / 5, 0.0, 3 * np.pi / 5, 0.0]

    if not args.no_reset:
        print("Resetting robot to home position ...")
        ok, msg = client.reset_to_joints(HOME_Q, speed=0.2)
        if ok:
            print(f"[OK] Robot at home position")
        else:
            print(f"[WARN] Reset: {msg}")
    else:
        print("[SKIP] Robot reset skipped (--no_reset)")

    # === Step 4: Print controls ================================================
    btn_a = "A" if right_controller else "X"
    btn_b = "B" if right_controller else "Y"
    print()
    print("=" * 55)
    print("TELEOPERATION READY")
    print("=" * 55)
    print(f"  Hold GRIP TRIGGER    → move robot")
    print(f"  INDEX TRIGGER        → close gripper (release to open)")
    print(f"  JOYSTICK press       → recalibrate orientation")
    print(f"  '{btn_a}'                  → stop (success)")
    print(f"  '{btn_b}'                  → stop (failure)")
    print(f"  Ctrl+C               → emergency stop")
    print(f"  r + Enter            → reset VR state")
    print(f"  q + Enter            → quit")
    print(f"  Control frequency:   {args.hz} Hz")
    print("=" * 55)
    print()

    # === Step 5: Main control loop ============================================
    step_count    = 0
    gripper_open  = True
    GRIPPER_OPEN  = 0.08   # Franka Hand max width [m]
    GRIPPER_CLOSE = 0.0
    GRIPPER_SPEED = 0.1    # finger speed [m/s]

    robot_origin = None    # 4×4 matrix, captured when grip trigger toggles

    while running:
        loop_start = time.time()
        step_count += 1

        # ── VR controller info ──────────────────────────────────────────────
        info = vr.get_info()

        # ── Stop signals ────────────────────────────────────────────────────
        if info["success"]:
            print(f"\n\n[DONE] '{btn_a}' pressed — trajectory marked as success")
            break
        if info["failure"]:
            print(f"\n\n[DONE] '{btn_b}' pressed — trajectory marked as failure")
            break
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

        # ── Arm: only move when grip trigger held ───────────────────────────
        pos_delta = None
        if info["movement_enabled"]:
            pos_delta, rot_delta, _ = vr.get_pose_delta()

            # Capture robot origin when VR origin resets
            if vr.origin_just_reset:
                robot_origin = pose16_to_mat(state["pose"])

            elif pos_delta is not None and robot_origin is not None:
                # Compose VR delta with robot origin → absolute target
                T_target = np.eye(4)
                T_target[:3, :3] = rot_delta @ robot_origin[:3, :3]
                T_target[:3, 3]  = robot_origin[:3, 3] + pos_delta
                client.set_ee_target(mat_to_pose16(T_target))

        # ── Gripper: read index trigger directly ────────────────────────────
        trig_key = "rightTrig" if right_controller else "leftTrig"
        index_trig = vr._state["buttons"].get(trig_key, (0.0,))
        if isinstance(index_trig, (tuple, list)):
            index_trig = index_trig[0]
        want_closed = index_trig > 0.5
        if want_closed and gripper_open:
            client.set_gripper_target(GRIPPER_CLOSE, GRIPPER_SPEED)
            gripper_open = False
        elif not want_closed and not gripper_open:
            client.set_gripper_target(GRIPPER_OPEN, GRIPPER_SPEED)
            gripper_open = True

        # ── Regulate frequency ──────────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = loop_period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

        actual_hz = 1.0 / max(time.time() - loop_start, 1e-6)
        T_cur = pose16_to_mat(state["pose"])
        print_status(step_count, info["movement_enabled"], pos_delta, actual_hz,
                     T_cur[:3, 3], state["gripper_width"])

    # === Cleanup ==============================================================
    print("\nTeleoperation ended.")
    print(f"Total steps: {step_count}")
    try:
        client.stop()
        client.close()
    except Exception:
        pass
    # OculusReader.stop() hangs because the ADB logcat socket read is blocking
    # and cannot be interrupted.  Force-exit instead.
    os._exit(0)


if __name__ == "__main__":
    main()
