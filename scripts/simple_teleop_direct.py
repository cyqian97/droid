#!/usr/bin/env python3
"""
Teleoperation using VR controller + IK + direct libfranka control.

No Polymetis, no ZeroRPC.  The pipeline is:
  Oculus Quest 3 → VRPolicy → Cartesian velocity → RobotIKSolver → joint
  positions → FrankaDirectClient (gRPC) → franka_server.cpp (1 kHz torque loop)

Prerequisites:
  1. Build franka_server inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch franka_server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server.sh
  4. Connect Oculus Quest 3 via ADB.
  5. Run this script:
       python scripts/simple_teleop_direct.py

Controls:
    Grip trigger (side)  : Hold to enable robot movement
    Index trigger         : Close/open gripper (display only — no gripper HW)
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
sys.path.insert(0, REPO_ROOT)                                        # makes `droid` importable
sys.path.insert(0, os.path.join(REPO_ROOT, "franka_direct", "python"))  # FrankaDirectClient

try:
    from franka_direct_client import FrankaDirectClient
except ImportError as e:
    print(f"[ERROR] Could not import FrankaDirectClient: {e}")
    print("Did you run:  bash franka_direct/python/generate_stubs.sh ?")
    sys.exit(1)

from droid.controllers.oculus_controller import VRPolicy
from droid.misc.transformations import rmat_to_euler
from droid.robot_ik.robot_ik_solver import RobotIKSolver


# ── Helpers ───────────────────────────────────────────────────────────────────

def pose16_to_cartesian(pose16):
    """Convert a col-major 4×4 homogeneous transform (16 floats) to [x,y,z,roll,pitch,yaw]."""
    T = np.array(pose16).reshape(4, 4, order='F')   # column-major → standard row-major matrix
    xyz   = T[:3, 3]
    euler = rmat_to_euler(T[:3, :3])                 # xyz Euler angles (radians)
    return np.concatenate([xyz, euler])


def check_keyboard():
    """Non-blocking keyboard read. Returns stripped line or None."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def print_status(step, enabled, hz, cartesian_pos, gripper):
    sys.stdout.write(
        f"\r[Step {step:>6}] "
        f"{'MOVING' if enabled else 'PAUSED':<7} | "
        f"Hz: {hz:>5.1f} | "
        f"z={cartesian_pos[2]:.3f}m | "
        f"gripper={gripper*100:.0f}mm    "
    )
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Franka FR3 teleoperation via VR + IK + franka_server")
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
    parser.add_argument("--pos_gain", type=float, default=3.0,
                        help="VRPolicy translational gain (default: 3.0)")
    parser.add_argument("--rot_gain", type=float, default=1.0,
                        help="VRPolicy rotational gain (default: 1.0)")
    args = parser.parse_args()

    right_controller = not args.left
    loop_period      = 1.0 / args.hz

    # ── Ctrl+C handler ────────────────────────────────────────────────────────
    running = True

    def _sigint(sig, frame):
        nonlocal running
        print("\n\nCtrl+C detected. Stopping...")
        running = False

    signal.signal(signal.SIGINT, _sigint)

    print("=" * 55)
    print("Teleoperation  —  VR + IK + direct franka_server")
    print("=" * 55)

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
    # RobotIKSolver uses dm_robotics / MuJoCo — takes ~1-2 s to compile the model
    print("Initializing IK solver (loading MuJoCo model) ...")
    ik_solver = RobotIKSolver()
    print("[OK] IK solver ready")

    # === Step 3: Initialize VR controller =====================================
    print("Initializing VR controller ...")
    try:
        controller = VRPolicy(right_controller=right_controller,
                              pos_action_gain=args.pos_gain,
                              rot_action_gain=args.rot_gain)
        side = "right" if right_controller else "left"
        print(f"[OK] VR controller initialized ({side} hand, pos_gain={args.pos_gain}, rot_gain={args.rot_gain})")
    except Exception as e:
        print(f"[FAIL] Could not initialize VR controller: {e}")
        sys.exit(1)

    # === Step 4: Optionally reset robot to home ================================
    # Home position matches simple_teleop.py reset_joints
    HOME_Q = [0.0, -np.pi / 5, 0.0, -4 * np.pi / 5, 0.0, 3 * np.pi / 5, 0.0]

    if not args.no_reset:
        print("Resetting robot to home position ...")
        client.set_joint_target(HOME_Q)
        # Poll until the robot arrives (server now tracks at max_step after
        # interp window, so one send is enough for any distance).
        home_arr = np.array(HOME_Q)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            s = client.get_robot_state()
            if s["error"]:
                print(f"[FAIL] Robot error during reset: {s['error']}")
                sys.exit(1)
            err = np.max(np.abs(np.array(s["q"]) - home_arr))
            sys.stdout.write(f"\r  Moving to home ... max joint error = {np.rad2deg(err):.2f} deg   ")
            sys.stdout.flush()
            if err < np.deg2rad(3):   # within 3 degrees on all joints
                break
            time.sleep(0.1)
        else:
            print("\n[WARN] Reset timed out — robot may not be at home")
        print("\n[OK] Robot at home position")
    else:
        print("[SKIP] Robot reset skipped (--no_reset)")

    # === Step 5: Print controls ================================================
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

    # === Step 6: Main control loop ============================================
    step_count    = 0
    gripper_pos   = 0.0    # integrated VRPolicy gripper signal [0, 1]
    gripper_open  = True   # current commanded state (True = open, False = closed)
    GRIPPER_OPEN  = 0.08   # Franka Hand max width [m]
    GRIPPER_CLOSE = 0.0    # fully closed [m]
    GRIPPER_SPEED = 0.1    # finger speed [m/s]

    # Seed target_q from current server state (after reset, this is near HOME_Q)
    init_state = client.get_robot_state()
    target_q   = np.array(init_state["target_q"])

    while running:
        loop_start = time.time()
        step_count += 1

        # ── VR controller info (local, no blocking) ───────────────────────────
        info = controller.get_info()

        # ── Stop signals ──────────────────────────────────────────────────────
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

        # ── Keyboard input ────────────────────────────────────────────────────
        key = check_keyboard()
        if key == "q":
            print("\n\n[DONE] 'q' pressed — quitting")
            break
        elif key == "r":
            print("\n\nResetting VR controller state ...")
            controller.reset_state()
            print("VR state reset. Hold grip trigger to start again.")
            continue

        # ── Get current robot state ───────────────────────────────────────────
        state = client.get_robot_state()
        if state["error"]:
            print(f"\n[ERROR] Robot error: {state['error']}")
            break

        # current_q and target_q from server
        target_q = np.array(state["target_q"])   # use commanded pose as IK base

        # Build a state_dict compatible with VRPolicy._calculate_action
        cartesian_pos = pose16_to_cartesian(state["pose"])   # actual measured EE pose
        robot_state_dict = {
            "cartesian_position": cartesian_pos,
            "gripper_position":   gripper_pos,
        }

        # ── Compute action (always, so gripper works without grip trigger) ──────
        obs_dict = {"robot_state": robot_state_dict}
        action = controller.forward(obs_dict)
        # action: [lin_vel(3), rot_vel(3), gripper(1)], all in [-1, 1]

        # ── Arm: only move when grip trigger held ─────────────────────────────
        if info["movement_enabled"]:
            ik_robot_state = {
                "joint_positions":  target_q.tolist(),
                "joint_velocities": [0.0] * 7,
            }
            joint_vel   = ik_solver.cartesian_velocity_to_joint_velocity(
                action[:6], ik_robot_state)
            joint_delta = ik_solver.joint_velocity_to_delta(joint_vel)

            new_target_q = target_q + joint_delta
            client.set_joint_target(new_target_q.tolist())
            target_q = new_target_q

        # ── Gripper: always active, independent of grip trigger ───────────────
        # action[6] > 0 → index trigger pressed (close), < 0 → released (open)
        gripper_pos = float(np.clip(gripper_pos + action[6] * 0.1, 0.0, 1.0))
        want_closed = gripper_pos > 0.5
        if want_closed and gripper_open:
            client.set_gripper_target(GRIPPER_CLOSE, GRIPPER_SPEED)
            gripper_open = False
        elif not want_closed and not gripper_open:
            client.set_gripper_target(GRIPPER_OPEN, GRIPPER_SPEED)
            gripper_open = True

        # ── Regulate frequency ────────────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = loop_period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

        actual_hz = 1.0 / max(time.time() - loop_start, 1e-6)
        print_status(step_count, info["movement_enabled"], actual_hz,
                     cartesian_pos, state["gripper_width"])

    # === Cleanup ==============================================================
    print("\nTeleoperation ended.")
    print(f"Total steps: {step_count}")
    try:
        client.stop()
        client.close()
    except Exception:
        pass
    # VRPolicy runs a non-daemon background thread that would otherwise keep
    # the process alive indefinitely after the main loop exits.
    os._exit(0)


if __name__ == "__main__":
    main()
