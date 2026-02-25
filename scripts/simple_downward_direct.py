#!/usr/bin/env python3
"""
Set an EE pose target (translation + rotation offsets) and monitor tracking error.

Bypasses Polymetis entirely — sends O_T_EE pose targets to
franka_server_cartesian (C++ gRPC server).  The server runs a PD controller
at 1 kHz converting pose error to Cartesian velocity commands.

Usage examples:
  # Move 50 mm down in z
  python scripts/simple_downward_direct.py --z_mm -50

  # Move 30 mm in x, rotate 10 deg about z
  python scripts/simple_downward_direct.py --x_mm 30 --z_deg 10

  # Combined translation + rotation, 15 second timeout
  python scripts/simple_downward_direct.py --x_mm 20 --y_mm -10 --z_mm -30 --x_deg 5 --duration 15

Prerequisites:
  1. Build inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch the Cartesian server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
  4. Run this script on the laptop:
       python scripts/simple_downward_direct.py --z_mm -50
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
        description="Set EE pose target and monitor tracking error (direct libfranka)")
    p.add_argument("--host",     default="192.168.1.6",
                   help="NUC IP running franka_server (default: 192.168.1.6)")
    p.add_argument("--port",     type=int, default=50052)
    p.add_argument("--hz",       type=float, default=25.0,
                   help="Monitoring frequency in Hz (default: 25)")
    p.add_argument("--duration", type=float, default=10.0,
                   help="Max duration in seconds (default: 10)")

    g = p.add_argument_group("translation offsets (mm, in base frame)")
    g.add_argument("--x_mm", type=float, default=0.0, help="X displacement in mm")
    g.add_argument("--y_mm", type=float, default=0.0, help="Y displacement in mm")
    g.add_argument("--z_mm", type=float, default=0.0, help="Z displacement in mm")

    g = p.add_argument_group("rotation offsets (degrees, extrinsic XYZ in base frame)")
    g.add_argument("--x_deg", type=float, default=0.0, help="Rotation about X axis in degrees")
    g.add_argument("--y_deg", type=float, default=0.0, help="Rotation about Y axis in degrees")
    g.add_argument("--z_deg", type=float, default=0.0, help="Rotation about Z axis in degrees")

    return p.parse_args()


# ── Rotation helpers ──────────────────────────────────────────────────────────

def rot_x(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rot_y(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot_z(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose16_to_mat(pose16):
    """Column-major 16 floats -> 4x4 numpy array."""
    return np.array(pose16).reshape(4, 4, order='F')

def mat_to_pose16(T):
    """4x4 numpy array -> column-major 16 floats list."""
    return T.flatten(order='F').tolist()

def rotation_error_angle(R_target, R_actual):
    """Angle (radians) between two rotation matrices."""
    R_err = R_target @ R_actual.T
    trace = np.trace(R_err)
    # clamp for numerical safety
    cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(cos_angle)


# ── Helpers ───────────────────────────────────────────────────────────────────

def print_banner(title: str):
    print("=" * 60)
    print(title)
    print("=" * 60)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args   = parse_args()
    period = 1.0 / args.hz
    max_steps = int(args.duration * args.hz)

    dp = np.array([args.x_mm, args.y_mm, args.z_mm])      # mm
    dr = np.array([args.x_deg, args.y_deg, args.z_deg])    # deg

    if np.allclose(dp, 0) and np.allclose(dr, 0):
        print("[ERROR] No displacement specified. Use --x_mm, --y_mm, --z_mm, --x_deg, --y_deg, --z_deg.")
        sys.exit(1)

    print_banner("Pose Target — Direct libfranka")
    print(f"  Server:      {args.host}:{args.port}")
    print(f"  Monitor Hz:  {args.hz}")
    print(f"  Duration:    {args.duration} s  ({max_steps} steps)")
    print(f"  Translation: x={args.x_mm:+.1f}  y={args.y_mm:+.1f}  z={args.z_mm:+.1f} mm")
    print(f"  Rotation:    x={args.x_deg:+.1f}  y={args.y_deg:+.1f}  z={args.z_deg:+.1f} deg")

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
    pose16 = state["target_pose"]
    if len(pose16) != 16:
        print(f"[ERROR] Expected 16 pose values, got {len(pose16)}")
        sys.exit(1)

    T_init = pose16_to_mat(pose16)
    p_init = T_init[:3, 3].copy()
    R_init = T_init[:3, :3].copy()

    print(f"[OK] Initial EE target: x={p_init[0]:.3f}  y={p_init[1]:.3f}  z={p_init[2]:.4f} m")
    act = state["pose"]
    print(f"     actual EE pose:    x={act[12]:.3f}  y={act[13]:.3f}  z={act[14]:.4f} m")
    print(f"     cmd_success_rate = {state['cmd_success_rate']:.2f}")

    # ── Compute target pose ───────────────────────────────────────────────────
    # Translation offset (mm -> m)
    p_target = p_init + dp / 1000.0

    print(f"\n  Target EE:  x={p_target[0]:.3f}  y={p_target[1]:.3f}  z={p_target[2]:.4f} m")
    print(f"  Ramp:       {max_steps} steps over {args.duration:.1f} s at {args.hz} Hz")
    input("Press Enter to start ...")

    # ── Incremental ramp toward target ────────────────────────────────────────
    # Send linearly interpolated poses so the PD controller only sees small
    # errors each tick.  Measure error against the FINAL target throughout.
    print_banner("RAMPING")

    # Storage for time-series
    timestamps     = []
    pos_errors_mm  = []   # (N, 3)
    rot_errors_deg = []   # (N,)
    rpc_times      = []
    cmd_rates      = []

    step = 0
    t_start = time.monotonic()
    try:
        for step in range(max_steps):
            loop_start = time.monotonic()

            # Interpolated target for this step
            frac = (step + 1) / max_steps
            p_interp = p_init + (p_target - p_init) * frac

            R_interp_delta = (rot_z(np.radians(dr[2] * frac))
                              @ rot_y(np.radians(dr[1] * frac))
                              @ rot_x(np.radians(dr[0] * frac)))
            R_interp = R_interp_delta @ R_init

            T_interp = np.eye(4)
            T_interp[:3, :3] = R_interp
            T_interp[:3, 3]  = p_interp
            interp_pose16 = mat_to_pose16(T_interp)

            # Send interpolated target
            t0 = time.monotonic()
            ok, msg = client.set_ee_target(interp_pose16)
            rpc_ms = (time.monotonic() - t0) * 1000
            rpc_times.append(rpc_ms)

            if not ok:
                print(f"\n[ERROR] SetEETarget failed: {msg}")
                break

            # Read current state
            state = client.get_robot_state()
            if state["error"]:
                print(f"\n[ERROR] Robot error: {state['error']}")
                break

            # Compute errors: commanded pose this step vs actual read-back
            T_actual = pose16_to_mat(state["pose"])
            p_actual = T_actual[:3, 3]
            R_actual = T_actual[:3, :3]

            pe = (p_interp - p_actual) * 1000.0   # mm
            re = np.degrees(rotation_error_angle(R_interp, R_actual))

            t_elapsed = time.monotonic() - t_start
            timestamps.append(t_elapsed)
            pos_errors_mm.append(pe.copy())
            rot_errors_deg.append(re)
            cmd_rates.append(state["cmd_success_rate"])

            pos_norm = np.linalg.norm(pe)
            sys.stdout.write(
                f"\r[{step+1:>4}/{max_steps}  {t_elapsed:>5.1f}s] "
                f"err: x={pe[0]:+6.2f} y={pe[1]:+6.2f} z={pe[2]:+6.2f} mm "
                f"(|{pos_norm:5.2f}|) | "
                f"rot={re:5.2f}deg | "
                f"rate={state['cmd_success_rate']:.3f}    "
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
    print()
    print_banner("SUMMARY")
    total_time = timestamps[-1] if timestamps else 0
    print(f"  Duration:        {total_time:.1f} s  ({step + 1} steps)")

    if pos_errors_mm:
        pe_arr = np.array(pos_errors_mm)       # (N, 3)
        pe_norm = np.linalg.norm(pe_arr, axis=1)  # (N,)
        re_arr = np.array(rot_errors_deg)      # (N,)

        print(f"\n  POSITION TRACKING ERROR  commanded - actual (mm):")
        print(f"    {'':>10}  {'x':>8}  {'y':>8}  {'z':>8}  {'|norm|':>8}")
        print(f"    {'Mean':>10}  {pe_arr[:,0].mean():>+8.2f}  {pe_arr[:,1].mean():>+8.2f}  {pe_arr[:,2].mean():>+8.2f}  {pe_norm.mean():>8.2f}")
        print(f"    {'Std':>10}  {pe_arr[:,0].std():>8.3f}  {pe_arr[:,1].std():>8.3f}  {pe_arr[:,2].std():>8.3f}  {pe_norm.std():>8.3f}")
        print(f"    {'Max norm':>10}  {pe_arr[np.argmax(pe_norm),0]:>+8.2f}  {pe_arr[np.argmax(pe_norm),1]:>+8.2f}  {pe_arr[np.argmax(pe_norm),2]:>+8.2f}  {pe_norm.max():>8.2f}  (t={timestamps[np.argmax(pe_norm)]:.1f}s)")
        print(f"    {'Min norm':>10}  {pe_arr[np.argmin(pe_norm),0]:>+8.2f}  {pe_arr[np.argmin(pe_norm),1]:>+8.2f}  {pe_arr[np.argmin(pe_norm),2]:>+8.2f}  {pe_norm.min():>8.2f}  (t={timestamps[np.argmin(pe_norm)]:.1f}s)")

        print(f"\n  ROTATION TRACKING ERROR  commanded @ actual^T (deg):")
        print(f"    Mean:      {re_arr.mean():.3f}")
        print(f"    Std:       {re_arr.std():.4f}")
        print(f"    Max:       {re_arr.max():.3f}  (t={timestamps[np.argmax(re_arr)]:.1f}s)")
        print(f"    Min:       {re_arr.min():.3f}  (t={timestamps[np.argmin(re_arr)]:.1f}s)")

    if rpc_times:
        rt = np.array(rpc_times)
        print(f"\n  SET_EE_TARGET RPC:")
        print(f"    Mean: {rt.mean():.2f} ms   Median: {np.median(rt):.2f} ms   p95: {np.percentile(rt, 95):.2f} ms   Max: {rt.max():.2f} ms")

    if cmd_rates:
        cr = np.array(cmd_rates)
        print(f"\n  1kHz RT LOOP cmd_success_rate:")
        print(f"    Mean: {cr.mean():.3f}   Min: {cr.min():.3f}")

    client.stop()
    client.close()


if __name__ == "__main__":
    main()
