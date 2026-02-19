#!/usr/bin/env python3
"""
Simple downward motion policy for Franka FR3.
Demonstrates basic robot control: moves gripper down by 1mm per command at 25Hz.

Usage:
    python scripts/simple_downward_motion.py                              # Default: velocity control
    python scripts/simple_downward_motion.py --action_space cartesian_position  # Position control
    python scripts/simple_downward_motion.py --steps 50                   # Custom step count
    python scripts/simple_downward_motion.py --hz 10                      # Custom frequency
    python scripts/simple_downward_motion.py --step_size 2.0              # 2mm per step
    python scripts/simple_downward_motion.py --min_z 0.05                 # Lower safety limit
    python scripts/simple_downward_motion.py --no_reset                   # Skip robot reset

Controls:
    Ctrl+C : Emergency stop
"""

import argparse
import signal
import sys
import time

import numpy as np

from droid.misc.parameters import nuc_ip
from droid.misc.server_interface import ServerInterface


def main():
    parser = argparse.ArgumentParser(description="Simple FR3 downward motion policy")
    parser.add_argument("--ip", type=str, default=None, help="NUC IP address (default: from parameters.py)")
    parser.add_argument("--hz", type=int, default=25, help="Control frequency in Hz (default: 25)")
    parser.add_argument("--steps", type=int, default=200, help="Number of downward steps (default: 100)")
    parser.add_argument("--step_size", type=float, default=0.25, help="Downward movement per step in mm (default: 1.0)")
    parser.add_argument("--min_z", type=float, default=0.1, help="Minimum z-height safety limit in meters (default: 0.1)")
    parser.add_argument("--no_reset", action="store_true", help="Skip robot reset to home position")
    parser.add_argument(
        "--action_space",
        type=str,
        default="cartesian_velocity",
        choices=["cartesian_velocity", "cartesian_position"],
        help="Control mode (default: cartesian_velocity)",
    )
    args = parser.parse_args()

    # Parameters
    ip = args.ip if args.ip else nuc_ip
    control_hz = args.hz
    loop_period = 1.0 / control_hz
    num_steps = args.steps
    step_size_m = args.step_size / 1000.0  # Convert mm to meters
    min_z = args.min_z
    action_space = args.action_space

    # Handle Ctrl+C gracefully
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n\nCtrl+C detected. Stopping...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # === Step 1: Connect to NUC server ===
    print("=" * 60)
    print("Simple Downward Motion Policy")
    print("=" * 60)
    print(f"Connecting to NUC at {ip}...")

    try:
        robot = ServerInterface(ip_address=ip)
        print("[OK] Connected to NUC server")
        print("[OK] Launched controller and robot")
    except Exception as e:
        print(f"[FAIL] Could not connect to NUC: {e}")
        print("Make sure run_server.py is running on the NUC.")
        sys.exit(1)

    # === Step 2: Optionally reset robot ===
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

    # === Step 3: Get initial end-effector pose ===
    print("Getting initial end-effector pose...")
    try:
        state_dict, _ = robot.get_robot_state()
        initial_pose = np.array(state_dict["cartesian_position"])
        print(f"[OK] Initial pose: x={initial_pose[0]:.3f}, y={initial_pose[1]:.3f}, z={initial_pose[2]:.3f}")
    except Exception as e:
        print(f"[FAIL] Could not get robot state: {e}")
        sys.exit(1)

    # Check if initial z is above safety limit
    if initial_pose[2] <= min_z:
        print(f"[ERROR] Initial z-height ({initial_pose[2]:.3f}m) is at or below safety limit ({min_z}m)")
        print("Please move the robot higher or adjust --min_z parameter.")
        sys.exit(1)

    # Calculate total movement
    total_movement_m = num_steps * step_size_m
    estimated_time = num_steps / control_hz
    final_z = initial_pose[2] - total_movement_m

    print()
    print("=" * 60)
    print("MOTION PLAN")
    print("=" * 60)
    print(f"  Action space: {action_space}")
    print(f"  Control frequency: {control_hz} Hz")
    print(f"  Number of steps: {num_steps}")
    print(f"  Step size: {args.step_size} mm")
    print(f"  Total downward movement: {total_movement_m * 1000:.1f} mm ({total_movement_m:.3f} m)")
    print(f"  Estimated duration: {estimated_time:.2f} seconds")
    print(f"  Initial z-height: {initial_pose[2]:.3f} m")
    print(f"  Expected final z-height: {final_z:.3f} m")
    print(f"  Safety limit (min z): {min_z:.3f} m")
    print("=" * 60)
    print()

    # Safety check on expected final position
    if final_z < min_z:
        print(f"[WARN] Expected final z ({final_z:.3f}m) is below safety limit ({min_z}m)")
        print(f"Motion will stop early at step ~{int((initial_pose[2] - min_z) / step_size_m)}")
        print()

    print("Press Ctrl+C to stop at any time.")
    print("Starting in 2 seconds...")
    time.sleep(2)
    print()

    # === Step 4: Main control loop ===
    step_count = 0
    estimated_z = initial_pose[2]

    if action_space == "cartesian_velocity":
        # Velocity action: [vx, vy, vz, wx, wy, wz, gripper] normalized to [-1, 1]
        # IK solver scales velocity by max_lin_delta (0.075m) to get delta per step.
        # vz = step_size_m / max_lin_delta gives exactly step_size_m per step.
        max_lin_delta = 0.075  # from RobotIKSolver
        vz = -step_size_m * control_hz / max_lin_delta  # negative = downward
        
        action = np.array([0.0, 0.0, vz, 0.0, 0.0, 0.0, 0.0])
    else:
        # Position action: [x, y, z, roll, pitch, yaw, gripper]
        # Append gripper=0 so action[:-1] gives the full 6D pose.
        action = np.append(initial_pose.copy(), 0.0)

    print("=" * 60)
    print("EXECUTING DOWNWARD MOTION")
    if action_space == "cartesian_velocity":
        print(f"Z velocity: {vz:.3f} m/s (velocity control)" )
    print("=" * 60)

    rpc_times = []
    slow_steps = []

    while running and step_count < num_steps:
        loop_start = time.time()

        # Update estimated z and safety check
        estimated_z -= step_size_m
        if estimated_z < min_z:
            print(f"\n[SAFETY] z-height limit reached ({estimated_z:.3f}m < {min_z}m)")
            print(f"Stopping at step {step_count} of {num_steps}")
            break

        # For position control, update target z each step
        if action_space == "cartesian_position":
            action[2] -= step_size_m

        # Send command (IK happens server-side in create_action_dict)
        try:
            rpc_start = time.time()
            robot.update_command(action, action_space=action_space, blocking=False)
            rpc_elapsed = time.time() - rpc_start
            rpc_times.append(rpc_elapsed)
            if rpc_elapsed > loop_period * 2:
                slow_steps.append((step_count, rpc_elapsed))
        except Exception as e:
            print(f"\n[ERROR] Failed to send command: {e}")
            print("Stopping motion.")
            break

        step_count += 1

        # Regulate frequency
        elapsed = time.time() - loop_start
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Calculate actual frequency
        actual_hz = 1.0 / max(time.time() - loop_start, 0.001)

        # Print status (update same line)
        sys.stdout.write(
            f"\r[Step {step_count:>4}/{num_steps}] "
            f"z={estimated_z:>.4f}m | "
            f"Hz={actual_hz:>5.1f} | "
            f"RPC: {rpc_elapsed*1000:>6.1f}ms    "
        )
        sys.stdout.flush()

    # === Step 5: Summary ===
    print("\n")
    print("=" * 60)
    print("MOTION COMPLETE")
    print("=" * 60)
    print(f"  Action space: {action_space}")
    print(f"  Steps completed: {step_count} / {num_steps}")
    print(f"  Total downward movement: {step_count * args.step_size:.1f} mm ({step_count * step_size_m:.4f} m)")
    print(f"  Final z-height (estimated): {estimated_z:.4f} m")
    print(f"  Initial z-height: {initial_pose[2]:.4f} m")
    print(f"  Duration: {(step_count / control_hz):.2f} seconds")
    print()
    if rpc_times:
        rpc_arr = np.array(rpc_times) * 1000  # convert to ms
        print("  RPC TIMING DIAGNOSTICS:")
        print(f"    Mean:   {rpc_arr.mean():>8.1f} ms")
        print(f"    Median: {np.median(rpc_arr):>8.1f} ms")
        print(f"    Min:    {rpc_arr.min():>8.1f} ms")
        print(f"    Max:    {rpc_arr.max():>8.1f} ms")
        print(f"    Std:    {rpc_arr.std():>8.1f} ms")
        print(f"    Target: {loop_period*1000:>8.1f} ms ({control_hz} Hz)")
        if slow_steps:
            print(f"    Slow steps (>{loop_period*2*1000:.0f}ms): {len(slow_steps)}/{len(rpc_times)}")
            for step, t in slow_steps[:10]:
                print(f"      Step {step}: {t*1000:.1f} ms")
            if len(slow_steps) > 10:
                print(f"      ... and {len(slow_steps)-10} more")
        else:
            print(f"    Slow steps: 0 (all within budget)")
    print("=" * 60)


if __name__ == "__main__":
    main()
