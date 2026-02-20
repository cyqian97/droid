#!/usr/bin/env python3
"""
Simple Joint Motion using direct libfranka joint position control.

Bypasses Polymetis entirely — sends joint position targets directly to
franka_server (C++ gRPC server) at 25 Hz.  The server linearly interpolates
between waypoints at 1 kHz.

Prerequisites:
  1. Build franka_server inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Regenerate Python stubs (after proto change):
       bash franka_direct/python/generate_stubs.sh
  3. Launch franka_server:
       docker exec <container> bash /app/droid/franka_direct/launch_server.sh
  4. Run this script on the laptop:
       python scripts/simple_joint_direct.py
"""

import argparse
import sys
import os
import time

import numpy as np

# ── Import FrankaDirectClient ─────────────────────────────────────────────────
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
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
        description="Slowly rotate Franka joints using direct libfranka joint position control")
    p.add_argument("--host",      default="192.168.1.6",
                   help="NUC IP running franka_server (default: 192.168.1.6)")
    p.add_argument("--port",      type=int,   default=50052)
    p.add_argument("--hz",        type=float, default=25.0,
                   help="Control frequency in Hz (default: 25)")
    p.add_argument("--steps",     type=int,   default=200,
                   help="Number of control steps (default: 200)")
    p.add_argument("--delta_deg", type=float, default=0.1,
                   help="Joint rotation per step in degrees (default: 0.1 → 20 deg total at 200 steps)")
    p.add_argument("--joints",    type=str,   default="3",
                   help="Comma-separated joint indices to rotate, 0-based (default: 3)")
    return p.parse_args()


# ── Helpers ───────────────────────────────────────────────────────────────────

def print_banner(title: str):
    print("=" * 60)
    print(title)
    print("=" * 60)

def fmt_q(q):
    return "[" + ", ".join(f"{v:+.4f}" for v in q) + "]"


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args       = parse_args()
    delta_rad  = np.deg2rad(args.delta_deg)
    period     = 1.0 / args.hz
    joint_idxs = [int(j) for j in args.joints.split(",")]

    for j in joint_idxs:
        if j < 0 or j > 6:
            print(f"[ERROR] Joint index {j} out of range [0, 6]")
            sys.exit(1)

    print_banner("Simple Joint Motion — Direct libfranka")
    print(f"  Server:     {args.host}:{args.port}")
    print(f"  Frequency:  {args.hz} Hz   Period: {period*1000:.1f} ms")
    print(f"  Steps:      {args.steps}")
    print(f"  Delta/step: {args.delta_deg:.3f} deg  ({delta_rad:.5f} rad)")
    print(f"  Total:      {args.steps * args.delta_deg:.1f} deg per joint")
    print(f"  Joints:     {joint_idxs}")

    # ── Connect ───────────────────────────────────────────────────────────────
    print(f"\nConnecting to franka_server at {args.host}:{args.port} ...")
    client = FrankaDirectClient(host=args.host, port=args.port)

    print("Waiting for robot ready ...")
    try:
        state = client.wait_until_ready(timeout=15.0)
    except (TimeoutError, RuntimeError) as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # ── Get initial joint positions ───────────────────────────────────────────
    # Use target_q (server's current interpolated command) as baseline to avoid
    # any velocity discontinuity on the first step.
    target_q = list(state["target_q"])
    if len(target_q) != 7:
        print(f"[ERROR] Expected 7 joint positions, got {len(target_q)}")
        sys.exit(1)

    print(f"\n[OK] Initial joint target:  {fmt_q(target_q)} rad")
    print(f"     actual joint positions: {fmt_q(state['q'])} rad")
    print(f"     cmd_success_rate = {state['cmd_success_rate']:.3f}")
    print(f"\nExpected final q[{joint_idxs}] += {args.steps * args.delta_deg:.1f} deg")
    input("Press Enter to start ...")

    # ── Motion loop ───────────────────────────────────────────────────────────
    print_banner("EXECUTING JOINT MOTION")

    cmd_success_rates = []
    rpc_times         = []
    get_state_times   = []
    slow_steps        = []
    rate_dip_steps    = []

    step = 0
    try:
        for step in range(args.steps):
            loop_start = time.monotonic()

            # Increment selected joints
            for j in joint_idxs:
                target_q[j] += delta_rad

            # Send target
            t0 = time.monotonic()
            ok, msg = client.set_joint_target(target_q)
            rpc_ms = (time.monotonic() - t0) * 1000
            rpc_times.append(rpc_ms)
            if rpc_ms > period * 1000 * 2:
                slow_steps.append((step + 1, rpc_ms))

            if not ok:
                print(f"\n[ERROR] SetJointTarget failed: {msg}")
                break

            # Read back for diagnostics
            t1 = time.monotonic()
            state = client.get_robot_state()
            get_state_times.append((time.monotonic() - t1) * 1000)

            if state["error"]:
                print(f"\n[ERROR] Robot error: {state['error']}")
                break

            rate = state["cmd_success_rate"]
            cmd_success_rates.append(rate)
            if rate < 0.99:
                rate_dip_steps.append((step + 1, rate))

            # Status
            actual_q = state["q"]
            sys.stdout.write(
                f"\r[{step+1:>4}/{args.steps}] "
                f"target_q[{joint_idxs[0]}]={target_q[joint_idxs[0]]:+.4f} rad | "
                f"actual_q[{joint_idxs[0]}]={actual_q[joint_idxs[0]] if actual_q else 0:+.4f} rad | "
                f"rate={rate:.3f} | "
                f"rpc={rpc_ms:.1f}ms    "
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
    print(f"  Steps completed: {step + 1} / {args.steps}")
    print(f"  Total rotation:  {(step + 1) * args.delta_deg:.2f} deg per joint")

    if rpc_times:
        rt = np.array(rpc_times)
        print(f"\n  SET_JOINT_TARGET RPC (laptop→franka_server gRPC):")
        print(f"    Mean:   {rt.mean():>7.2f} ms")
        print(f"    Median: {np.median(rt):>7.2f} ms")
        print(f"    p90:    {np.percentile(rt, 90):>7.2f} ms")
        print(f"    p95:    {np.percentile(rt, 95):>7.2f} ms")
        print(f"    p99:    {np.percentile(rt, 99):>7.2f} ms")
        print(f"    Max:    {rt.max():>7.2f} ms")
        print(f"    Target: {period*1000:>7.1f} ms ({args.hz} Hz)")
        if slow_steps:
            print(f"    Slow (>{period*2*1000:.0f}ms): {len(slow_steps)} steps")
            for s, t in slow_steps[:10]:
                print(f"      Step {s}: {t:.1f} ms")
        else:
            print(f"    Slow steps: 0")

    if get_state_times:
        gs = np.array(get_state_times)
        print(f"\n  GET_ROBOT_STATE RPC:")
        print(f"    Mean:   {gs.mean():>7.2f} ms")
        print(f"    Median: {np.median(gs):>7.2f} ms")
        print(f"    p95:    {np.percentile(gs, 95):>7.2f} ms")
        print(f"    Max:    {gs.max():>7.2f} ms")

    if cmd_success_rates:
        cr = np.array(cmd_success_rates)
        print(f"\n  1kHz RT LOOP — control_command_success_rate:")
        print(f"    Mean:   {cr.mean():>7.3f}   (1.000 = no deadline misses)")
        print(f"    Median: {np.median(cr):>7.3f}")
        print(f"    p10:    {np.percentile(cr, 10):>7.3f}")
        print(f"    Min:    {cr.min():>7.3f}")
        if rate_dip_steps:
            print(f"    Dips <0.99: {len(rate_dip_steps)} steps")
            for s, r in rate_dip_steps[:10]:
                slow_near = [t for st, t in slow_steps if abs(st - s) <= 2]
                note = f"  ← near slow RPC ({slow_near[0]:.0f}ms)" if slow_near else ""
                print(f"      Step {s}: rate={r:.3f}{note}")
            if len(rate_dip_steps) > 10:
                print(f"      ... and {len(rate_dip_steps)-10} more")
        else:
            print(f"    Dips <0.99: 0  (RT loop ran clean throughout)")

    client.stop()
    client.close()


if __name__ == "__main__":
    main()
