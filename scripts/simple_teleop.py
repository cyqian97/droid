#!/usr/bin/env python3
"""
Simple teleoperation script for Franka FR3 + Oculus Quest 3.
No GUI, no cameras, no data saving — just pure teleoperation.

Usage:
    python scripts/simple_teleop.py                  # Right controller, with reset
    python scripts/simple_teleop.py --left           # Left controller
    python scripts/simple_teleop.py --no_reset       # Skip robot reset
    python scripts/simple_teleop.py --hz 10          # Custom control frequency

Controls:
    Grip trigger (side)  : Hold to enable robot movement
    Index trigger         : Close/open gripper
    Joystick press       : Recalibrate forward direction
    A button (right) / X (left) : Stop (success)
    B button (right) / Y (left) : Stop (failure)
    Ctrl+C               : Emergency stop
"""

import argparse
import select
import signal
import sys
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError

import numpy as np

from droid.controllers.oculus_controller import VRPolicy
from droid.misc.parameters import nuc_ip
from droid.misc.server_interface import ServerInterface


# Thread pool for non-blocking RPC calls with timeout
_executor = ThreadPoolExecutor(max_workers=2)


def rpc_with_timeout(func, timeout=3.0):
    """Run an RPC call with a timeout. Returns None if it times out."""
    future = _executor.submit(func)
    try:
        return future.result(timeout=timeout)
    except FuturesTimeoutError:
        return None
    except Exception:
        return None


def print_status(info, hz_actual, step_count, rpc_ok=True):
    """Print a single-line status update."""
    enabled = "MOVING" if info["movement_enabled"] else "PAUSED"
    controller = "ON" if info["controller_on"] else "OFF"
    rpc_status = "" if rpc_ok else " | RPC: RETRY"
    sys.stdout.write(
        f"\r[Step {step_count:>6}] "
        f"Robot: {enabled:<7} | "
        f"Controller: {controller:<3} | "
        f"Hz: {hz_actual:>5.1f}"
        f"{rpc_status}    "
    )
    sys.stdout.flush()


def check_keyboard():
    """Non-blocking check if user pressed a key (Enter or q)."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        key = sys.stdin.readline().strip().lower()
        return key
    return None


def main():
    parser = argparse.ArgumentParser(description="Simple Franka FR3 teleoperation")
    parser.add_argument("--left", action="store_true", help="Use left controller")
    parser.add_argument("--no_reset", action="store_true", help="Skip robot reset to home position")
    parser.add_argument("--hz", type=int, default=15, help="Control frequency (default: 15)")
    parser.add_argument("--ip", type=str, default=None, help="NUC IP address (default: from parameters.py)")
    parser.add_argument("--rpc_timeout", type=float, default=3.0, help="Per-call RPC timeout in seconds (default: 3)")
    args = parser.parse_args()

    right_controller = not args.left
    control_hz = args.hz
    loop_period = 1.0 / control_hz
    ip = args.ip if args.ip else nuc_ip
    rpc_timeout = args.rpc_timeout

    # --- Handle Ctrl+C gracefully ---
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n\nCtrl+C detected. Stopping...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # === Step 1: Connect to NUC server ===
    print("=" * 50)
    print("Simple Teleoperation")
    print("=" * 50)
    print(f"Connecting to NUC at {ip}...")

    try:
        robot = ServerInterface(ip_address=ip)
        print("[OK] Connected to NUC server")
        print("[OK] Launched controller and robot")
    except Exception as e:
        print(f"[FAIL] Could not connect to NUC: {e}")
        print("Make sure run_server.py is running on the NUC.")
        sys.exit(1)

    # === Step 2: Initialize VR controller ===
    print("Initializing VR controller...")
    try:
        controller = VRPolicy(right_controller=right_controller)
        side = "right" if right_controller else "left"
        print(f"[OK] VR controller initialized ({side} hand)")
    except Exception as e:
        print(f"[FAIL] Could not initialize VR controller: {e}")
        print("Make sure the Oculus headset is connected via ADB.")
        sys.exit(1)

    # === Step 3: Optionally reset robot ===
    if not args.no_reset:
        print("Resetting robot to home position...")
        reset_joints = [0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0]
        try:
            robot.update_joints(np.array(reset_joints), velocity=False, blocking=True)
            print("[OK] Robot at home position")
        except Exception as e:
            print(f"[WARN] Reset failed: {e}")
            print("Continuing without reset. Robot stays at current position.")
    else:
        print("[SKIP] Robot reset skipped (--no_reset)")

    # === Step 4: Print instructions ===
    btn_a = "A" if right_controller else "X"
    btn_b = "B" if right_controller else "Y"
    print()
    print("=" * 50)
    print("TELEOPERATION READY")
    print("=" * 50)
    print(f"  Hold GRIP TRIGGER to move robot")
    print(f"  Squeeze INDEX TRIGGER to close gripper")
    print(f"  Press JOYSTICK to recalibrate orientation")
    print(f"  Press '{btn_a}' to stop (success)")
    print(f"  Press '{btn_b}' to stop (failure)")
    print(f"  Press Ctrl+C for emergency stop")
    print(f"  Type 'r' + Enter to reset VR state")
    print(f"  Type 'q' + Enter to quit")
    print(f"  Control frequency: {control_hz} Hz")
    print(f"  RPC timeout: {rpc_timeout}s")
    print("=" * 50)
    print()

    # === Step 5: Main control loop ===
    step_count = 0
    rpc_failures = 0
    last_state = None
    reconnect_needed = False

    while running:
        loop_start = time.time()
        step_count += 1
        rpc_ok = True

        # --- Get VR controller info (local, never blocks) ---
        info = controller.get_info()

        # --- Check stop signals ---
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

        # --- Check keyboard input ---
        key = check_keyboard()
        if key == "q":
            print("\n\n[DONE] 'q' pressed — quitting")
            break
        elif key == "r":
            print("\n\nResetting VR controller state...")
            controller.reset_state()
            print("VR state reset. Hold grip trigger to start again.")
            continue
        elif key == "c":
            print("\n\nReconnecting to server...")
            reconnect_needed = True

        # --- Reconnect if needed ---
        if reconnect_needed:
            try:
                robot.establish_connection()
                robot.launch_robot()
                rpc_failures = 0
                reconnect_needed = False
                print("[OK] Reconnected. Resuming...")
            except Exception as e:
                print(f"[FAIL] Reconnect failed: {e}")
                print("Retrying in 3 seconds... (type 'q' to quit)")
                time.sleep(3)
                continue

        # --- Get robot state with timeout ---
        result = rpc_with_timeout(lambda: robot.get_robot_state(), timeout=rpc_timeout)
        if result is None:
            rpc_ok = False
            rpc_failures += 1
            if rpc_failures >= 10:
                print(f"\n\n[WARN] {rpc_failures} RPC failures. Server may be down.")
                print("Type 'c' + Enter to reconnect, or 'q' to quit.")
                rpc_failures = 0
            # Use last known state if available
            if last_state is None:
                time.sleep(0.1)
                actual_hz = 1.0 / max(time.time() - loop_start, 0.001)
                print_status(info, actual_hz, step_count, rpc_ok=False)
                continue
            state_dict = last_state
        else:
            state_dict = result[0]
            last_state = state_dict
            rpc_failures = 0

        # --- Calculate and send action ---
        if info["movement_enabled"]:
            obs_dict = {"robot_state": state_dict}
            action = controller.forward(obs_dict)

            # Send command with timeout (don't block if server hangs)
            cmd_result = rpc_with_timeout(
                lambda a=action: robot.update_command(a, action_space="cartesian_velocity"),
                timeout=rpc_timeout,
            )
            if cmd_result is None:
                rpc_ok = False
                rpc_failures += 1

        # --- Regulate frequency ---
        elapsed = time.time() - loop_start
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        # --- Print status ---
        actual_hz = 1.0 / max(time.time() - loop_start, 0.001)
        print_status(info, actual_hz, step_count, rpc_ok=rpc_ok)

    # === Cleanup ===
    _executor.shutdown(wait=False)
    print("\nTeleoperation ended.")
    print(f"Total steps: {step_count}")


if __name__ == "__main__":
    main()
