#!/usr/bin/env python3
"""
Simple Downward Motion using direct libfranka Cartesian pose control.

Bypasses Polymetis entirely — sends O_T_EE targets directly to franka_server
(C++ gRPC server) at 25 Hz.  The robot's built-in Cartesian impedance
controller (kJointImpedance mode) handles the 1 kHz servo loop with no gRPC
inside the RT callback.

Prerequisites:
  1. Build franka_server inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch franka_server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server.sh
  4. Run this script on the laptop:
       python scripts/simple_downward_direct.py
"""

import argparse
import sys
import os
import time

import numpy as np

# ── Import FrankaDirectClient ─────────────────────────────────────────────────
REPO_ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(REPO_ROOT, "franka_direct", "python"))

try:
    from franka_direct_client import FrankaDirectClient
except ImportError as e:
    print(f"[ERROR] Could not import FrankaDirectClient: {e}")
    print("Did you run:  bash franka_direct/python/generate_stubs.sh ?")
    sys.exit(1)


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Move Franka EE downward using direct libfranka Cartesian control")
    p.add_argument("--host",     default="192.168.1.6",
                   help="NUC IP running franka_server (default: 192.168.1.6)")
    p.add_argument("--port",     type=int, default=50052)
    p.add_argument("--hz",       type=float, default=25.0,
                   help="Control frequency in Hz (default: 25)")
    p.add_argument("--steps",    type=int,   default=200,
                   help="Number of control steps (default: 200)")
    p.add_argument("--step_mm",  type=float, default=0.25,
                   help="Downward step size per command in mm (default: 0.25 → 50 mm total)")
    p.add_argument("--min_z",    type=float, default=0.10,
                   help="Safety lower limit for z in metres (default: 0.10)")
    return p.parse_args()


# ── Helpers ───────────────────────────────────────────────────────────────────

def print_banner(title: str):
    print("=" * 60)
    print(title)
    print("=" * 60)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args   = parse_args()
    step_m = args.step_mm / 1000.0
    period = 1.0 / args.hz
    total_m = args.steps * step_m

    print_banner("Simple Downward Motion — Direct libfranka")
    print(f"  Server:    {args.host}:{args.port}")
    print(f"  Frequency: {args.hz} Hz   Period: {period*1000:.1f} ms")
    print(f"  Steps:     {args.steps}")
    print(f"  Step size: {args.step_mm} mm")
    print(f"  Total:     {total_m*1000:.1f} mm")
    print(f"  Min z:     {args.min_z} m")

    # ── Connect ───────────────────────────────────────────────────────────────
    print(f"\nConnecting to franka_server at {args.host}:{args.port} ...")
    client = FrankaDirectClient(host=args.host, port=args.port)

    print("Waiting for robot ready ...")
    try:
        state = client.wait_until_ready(timeout=15.0)
    except (TimeoutError, RuntimeError) as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # ── Get initial pose ──────────────────────────────────────────────────────
    pose = state["pose"]            # list[16], col-major O_T_EE
    if len(pose) != 16:
        print(f"[ERROR] Expected 16 pose values, got {len(pose)}")
        sys.exit(1)

    initial_z = pose[14]
    print(f"[OK] Initial EE pose:  x={pose[12]:.3f}  y={pose[13]:.3f}  z={initial_z:.4f} m")
    print(f"     cmd_success_rate = {state['cmd_success_rate']:.2f}")

    if initial_z <= args.min_z:
        print(f"[ERROR] z={initial_z:.3f} m is at or below safety limit ({args.min_z} m)")
        sys.exit(1)

    expected_final_z = initial_z - total_m
    print(f"\nExpected final z = {expected_final_z:.4f} m")
    input("Press Enter to start ...")

    # ── Motion loop ───────────────────────────────────────────────────────────
    print_banner("EXECUTING DOWNWARD MOTION")

    target_pose = list(pose)        # mutable copy
    cmd_success_rates = []
    rpc_times         = []

    step = 0
    try:
        for step in range(args.steps):
            loop_start = time.monotonic()

            # Move down
            target_pose[14] -= step_m

            # Safety
            if target_pose[14] < args.min_z:
                print(f"\n[STOP] Safety limit reached at z={target_pose[14]:.4f} m")
                break

            # Send target
            t0 = time.monotonic()
            ok, msg = client.set_cartesian_target(target_pose)
            rpc_times.append(time.monotonic() - t0)

            if not ok:
                print(f"\n[ERROR] SetCartesianTarget failed: {msg}")
                break

            # Read back for diagnostics
            state = client.get_robot_state()
            if state["error"]:
                print(f"\n[ERROR] Robot error: {state['error']}")
                break

            rate = state["cmd_success_rate"]
            cmd_success_rates.append(rate)

            # Status
            sys.stdout.write(
                f"\r[{step+1:>4}/{args.steps}] "
                f"z_target={target_pose[14]:.4f} m | "
                f"z_actual={state['pose'][14] if state['pose'] else 0:.4f} m | "
                f"success_rate={rate:.2f} | "
                f"rpc={rpc_times[-1]*1000:.1f}ms    "
            )
            sys.stdout.flush()

            # Regulate frequency
            elapsed = time.monotonic() - loop_start
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # ── Summary ───────────────────────────────────────────────────────────────
    print_banner("MOTION COMPLETE")
    print(f"  Steps completed:    {step + 1} / {args.steps}")
    print(f"  Total movement:     {(step + 1) * step_m * 1000:.1f} mm")
    print(f"  Target final z:     {target_pose[14]:.4f} m")
    print(f"  Initial z:          {initial_z:.4f} m")

    if rpc_times:
        print(f"\n  RPC TIMING (set_cartesian_target):")
        print(f"    Mean:   {np.mean(rpc_times)*1000:.2f} ms")
        print(f"    Max:    {np.max(rpc_times)*1000:.2f} ms")
        print(f"    Target: {period*1000:.1f} ms ({args.hz} Hz)")

    if cmd_success_rates:
        print(f"\n  CMD SUCCESS RATE (libfranka, from robot):")
        print(f"    Mean:  {np.mean(cmd_success_rates):.3f}")
        print(f"    Min:   {np.min(cmd_success_rates):.3f}")
        print(f"    (should be > 0.95 for smooth motion)")

    client.stop()
    client.close()


if __name__ == "__main__":
    main()
