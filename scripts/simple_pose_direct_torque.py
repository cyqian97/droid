#!/usr/bin/env python3
"""
Move the EE by a fixed pose delta per step using IK + joint torque control.

At each step the commanded target advances by (dx, dy, dz, drx, dry, drz),
so the total displacement after N steps is N × delta.

Pipeline:
  per-step delta → accumulated T_cmd
  → pose_to_cartesian_velocity() → RobotIKSolver (dm_robotics)
  → joint delta Δq → q_target → FrankaDirectClient (gRPC)
  → franka_server.cpp (1 kHz joint impedance torque loop)

Usage examples:
  # Move -2 mm/step in z at 15 Hz for 50 steps (total -100 mm over 3.3 s)
  python scripts/simple_pose_direct_torque.py --dz -2 --hz 15 --steps 50

  # Rotate 0.5 deg/step about z at 10 Hz for 30 steps
  python scripts/simple_pose_direct_torque.py --drz 0.5 --hz 10 --steps 30

Prerequisites:
  1. Build inside Docker:
       docker exec <container> bash /app/droid/franka_direct/build.sh
  2. Generate Python gRPC stubs (on the laptop):
       bash franka_direct/python/generate_stubs.sh
  3. Launch the torque server (do NOT run launch_robot.sh at the same time):
       docker exec <container> bash /app/droid/franka_direct/launch_server.sh
"""

import argparse
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "franka_direct", "python"))

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
    return np.array(pose16).reshape(4, 4, order='F')

def rotation_error_vec(R_target, R_current):
    R_err = R_target @ R_current.T
    cos_a = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-10:
        return np.zeros(3)
    k = angle / (2.0 * np.sin(angle))
    return k * np.array([R_err[2, 1] - R_err[1, 2],
                          R_err[0, 2] - R_err[2, 0],
                          R_err[1, 0] - R_err[0, 1]])

def rotation_error_angle(R_target, R_actual):
    R_err = R_target @ R_actual.T
    return np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))


# ── IK helper ─────────────────────────────────────────────────────────────────

def pose_to_cartesian_velocity(T_target, T_current, ik):
    p_err   = T_target[:3, 3] - T_current[:3, 3]
    rot_err = rotation_error_vec(T_target[:3, :3], T_current[:3, :3])

    lin_vel = p_err   / ik.max_lin_delta
    rot_vel = rot_err / ik.max_rot_delta

    lin_norm = np.linalg.norm(lin_vel)
    if lin_norm > 1.0:
        lin_vel /= lin_norm

    rot_norm = np.linalg.norm(rot_vel)
    if rot_norm > 1.0:
        rot_vel /= rot_norm

    return np.concatenate([lin_vel, rot_vel])


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Move EE by a fixed delta per step via IK + joint torque control")
    p.add_argument("--host",        default="192.168.1.6")
    p.add_argument("--port",        type=int,   default=50052)
    p.add_argument("--hz",          type=float, default=15.0,
                   help="Control frequency in Hz (default: 15)")
    p.add_argument("--steps",       type=int,   default=50,
                   help="Total number of steps to run (default: 50)")
    p.add_argument("--no-reset",    action="store_true",
                   help="Skip reset to home position")
    p.add_argument("--reset-speed", type=float, default=0.2)

    g = p.add_argument_group("per-step translation delta (mm, base frame)")
    g.add_argument("--dx", type=float, default=0.0, help="X delta per step in mm")
    g.add_argument("--dy", type=float, default=0.0, help="Y delta per step in mm")
    g.add_argument("--dz", type=float, default=0.0, help="Z delta per step in mm")

    g = p.add_argument_group("per-step rotation delta (degrees, extrinsic XYZ base frame)")
    g.add_argument("--drx", type=float, default=0.0, help="Rotation about X per step in degrees")
    g.add_argument("--dry", type=float, default=0.0, help="Rotation about Y per step in degrees")
    g.add_argument("--drz", type=float, default=0.0, help="Rotation about Z per step in degrees")

    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    dp_step = np.array([args.dx, args.dy, args.dz]) / 1000.0  # m per step
    dr_step = np.array([args.drx, args.dry, args.drz])         # deg per step

    if np.allclose(dp_step, 0) and np.allclose(dr_step, 0):
        print("[ERROR] No delta specified. Use --dx, --dy, --dz, --drx, --dry, --drz.")
        sys.exit(1)

    HOME_Q = [0.0, -np.pi / 5, 0.0, -4 * np.pi / 5, 0.0, 3 * np.pi / 5, 0.0]
    period = 1.0 / args.hz

    total_dp = dp_step * args.steps * 1000.0  # mm
    total_dr = dr_step * args.steps            # deg

    print("=" * 60)
    print("Pose Delta — IK + Joint Torque Control")
    print("=" * 60)
    print(f"  Server:        {args.host}:{args.port}")
    print(f"  Frequency:     {args.hz} Hz")
    print(f"  Steps:         {args.steps}  ({args.steps / args.hz:.1f} s)")
    print(f"  Delta/step:    dx={args.dx:+.3f}  dy={args.dy:+.3f}  dz={args.dz:+.3f} mm")
    print(f"                 drx={args.drx:+.3f}  dry={args.dry:+.3f}  drz={args.drz:+.3f} deg")
    print(f"  Total motion:  x={total_dp[0]:+.1f}  y={total_dp[1]:+.1f}  z={total_dp[2]:+.1f} mm")
    print(f"                 rx={total_dr[0]:+.1f}  ry={total_dr[1]:+.1f}  rz={total_dr[2]:+.1f} deg")

    # ── Connect ───────────────────────────────────────────────────────────────
    print(f"\nConnecting to franka_server at {args.host}:{args.port} ...")
    client = FrankaDirectClient(host=args.host, port=args.port)
    try:
        state = client.wait_until_ready(timeout=15.0)
        print(f"[OK] franka_server ready  (cmd_success_rate={state['cmd_success_rate']:.3f})")
    except (TimeoutError, RuntimeError) as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # ── IK solver ─────────────────────────────────────────────────────────────
    print("Initializing IK solver ...")
    try:
        ik = RobotIKSolver()
        print(f"[OK] IK solver ready  "
              f"(max_lin={ik.max_lin_delta*1000:.0f} mm/step, "
              f"max_rot={np.degrees(ik.max_rot_delta):.1f} deg/step)")
    except Exception as e:
        print(f"[ERROR] Could not initialize IK solver: {e}")
        sys.exit(1)

    # ── Reset to home ─────────────────────────────────────────────────────────
    if not args.no_reset:
        print(f"\nResetting to home joints (speed={args.reset_speed}) ...")
        ok, msg = client.reset_to_joints(HOME_Q, speed=args.reset_speed)
        print("[OK] Reset complete." if ok else f"[WARN] Reset: {msg}")
    else:
        print("Skipping home reset.")

    # ── Record initial pose ───────────────────────────────────────────────────
    state  = client.get_robot_state()
    T_init = pose16_to_mat(state["pose"])
    p_init = T_init[:3, 3].copy()
    R_init = T_init[:3, :3].copy()

    print(f"\n  Initial EE:  x={p_init[0]:.3f}  y={p_init[1]:.3f}  z={p_init[2]:.4f} m")
    print(f"  Final  EE:   x={p_init[0]+total_dp[0]/1000:.3f}  "
          f"y={p_init[1]+total_dp[1]/1000:.3f}  "
          f"z={p_init[2]+total_dp[2]/1000:.4f} m")

    input("\nPress Enter to start ...")

    # ── Control loop ──────────────────────────────────────────────────────────
    # T_cmd advances by (dp_step, dR_step) each iteration.
    # The accumulated rotation is applied extrinsically (base frame).

    timestamps      = []
    pos_errors_mm   = []
    rot_errors_deg  = []
    cmd_rates       = []
    joint_positions = []   # (N, 7) — q read at command-send time

    # Running accumulated command pose
    p_cmd = p_init.copy()
    R_cmd = R_init.copy()
    dR_step = (rot_z(np.radians(dr_step[2]))
               @ rot_y(np.radians(dr_step[1]))
               @ rot_x(np.radians(dr_step[0])))

    print("\n" + "=" * 60)
    print("RUNNING")
    print("=" * 60)

    t_start = time.monotonic()
    try:
        for step in range(args.steps):
            loop_start = time.monotonic()

            # Advance command target by one delta
            p_cmd = p_cmd + dp_step
            R_cmd = dR_step @ R_cmd

            T_cmd = np.eye(4)
            T_cmd[:3, :3] = R_cmd
            T_cmd[:3, 3]  = p_cmd

            # Current robot state
            state = client.get_robot_state()
            if state["error"]:
                print(f"\n[ERROR] Robot error: {state['error']}")
                break

            T_current = pose16_to_mat(state["pose"])

            # IK: pose error → Cartesian velocity → joint delta → q_target
            cart_vel = pose_to_cartesian_velocity(T_cmd, T_current, ik)
            robot_state_dict = {
                "joint_positions":  state["q"],
                "joint_velocities": state["dq"] if state["dq"] else [0.0] * 7,
            }
            joint_vel   = ik.cartesian_velocity_to_joint_velocity(cart_vel, robot_state_dict)
            joint_delta = ik.joint_velocity_to_delta(joint_vel)
            q_target    = (np.array(state["q"]) + joint_delta).tolist()
            client.set_joint_target(q_target)

            # Tracking error vs current command target
            pe = (p_cmd - T_current[:3, 3]) * 1000.0
            re = np.degrees(rotation_error_angle(R_cmd, T_current[:3, :3]))
            t_elapsed = time.monotonic() - t_start

            timestamps.append(t_elapsed)
            pos_errors_mm.append(pe.copy())
            rot_errors_deg.append(re)
            cmd_rates.append(state["cmd_success_rate"])
            joint_positions.append(list(state["q"]))

            sys.stdout.write(
                f"\r[{step+1:>4}/{args.steps}  {t_elapsed:>5.1f}s]  "
                f"pos_err: x={pe[0]:+6.2f} y={pe[1]:+6.2f} z={pe[2]:+6.2f} mm "
                f"(|{np.linalg.norm(pe):5.2f}|)  "
                f"rot={re:5.2f}deg  "
                f"rate={state['cmd_success_rate']:.3f}    "
            )
            sys.stdout.flush()

            elapsed = time.monotonic() - loop_start
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)

    if pos_errors_mm:
        pe_arr  = np.array(pos_errors_mm)
        pe_norm = np.linalg.norm(pe_arr, axis=1)
        re_arr  = np.array(rot_errors_deg)

        print(f"  Duration:  {timestamps[-1]:.1f} s  ({len(timestamps)} steps)")

        print(f"\n  POSITION TRACKING ERROR  (cmd - actual, mm):")
        print(f"    {'':>8}  {'x':>8}  {'y':>8}  {'z':>8}  {'|norm|':>8}")
        print(f"    {'Mean':>8}  {pe_arr[:,0].mean():>+8.2f}  {pe_arr[:,1].mean():>+8.2f}  {pe_arr[:,2].mean():>+8.2f}  {pe_norm.mean():>8.2f}")
        print(f"    {'Std':>8}  {pe_arr[:,0].std():>8.3f}  {pe_arr[:,1].std():>8.3f}  {pe_arr[:,2].std():>8.3f}  {pe_norm.std():>8.3f}")
        print(f"    {'Max':>8}  {pe_norm.max():>8.2f} mm  (t={timestamps[int(np.argmax(pe_norm))]:.1f}s)")

        print(f"\n  ROTATION TRACKING ERROR  (deg):")
        print(f"    Mean: {re_arr.mean():.3f}   Std: {re_arr.std():.4f}   Max: {re_arr.max():.3f}")

        cr = np.array(cmd_rates)
        print(f"\n  1kHz RT LOOP cmd_success_rate:")
        print(f"    Mean: {cr.mean():.3f}   Min: {cr.min():.3f}")

    # ── Plot ──────────────────────────────────────────────────────────────────
    if joint_positions:
        q_arr = np.degrees(np.array(joint_positions))  # (N, 7)
        t_arr = np.array(timestamps)

        _, ax = plt.subplots(figsize=(10, 5))
        for j in range(7):
            ax.plot(t_arr, q_arr[:, j], label=f"Joint {j+1}", linestyle=None, marker="+", markersize=3)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Joint position (deg)")
        ax.set_title("Joint positions vs time (command sent)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, linewidth=0.4)
        plt.tight_layout()
        plt.show()

    client.stop()
    client.close()


if __name__ == "__main__":
    main()
