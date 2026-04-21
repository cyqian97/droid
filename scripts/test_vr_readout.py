#!/usr/bin/env python3
"""
Test script to debug Meta Quest controller readouts.

Reads raw data from OculusReader and:
  1. Logs all button press/release events to a timestamped log file.
  2. Shows a live 3D matplotlib plot of the right controller's position
     and orientation (as an XYZ triad).

Usage:
  python scripts/test_vr_readout.py
  python scripts/test_vr_readout.py --left          # track left controller
  python scripts/test_vr_readout.py --hz 30         # faster polling
  python scripts/test_vr_readout.py --no-plot        # log only, no GUI

Press Ctrl+C to stop.  The log file is saved to logs/vr_readout_<timestamp>.log.
"""

import argparse
import datetime
import os
import select
import signal
import sys
import termios
import time
import tty

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)
from scipy.spatial.transform import Rotation as R

# ── Path setup ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "droid", "oculus_reader"))

from oculus_reader.reader import OculusReader


# ── Helpers ──────────────────────────────────────────────────────────────────

def mat4_to_pos_euler(mat4):
    """Extract position [x,y,z] and euler [rx,ry,rz] from a 4x4 transform."""
    pos = mat4[:3, 3].copy()
    euler = R.from_matrix(mat4[:3, :3]).as_euler("xyz", degrees=True)
    return pos, euler


class ButtonLogger:
    """Track button state changes and write press/release events to a log file."""

    # All boolean buttons we track
    BOOL_BUTTONS = ["A", "B", "X", "Y", "RThU", "RJ", "RG", "RTr", "LThU", "LJ", "LG", "LTr"]
    # Analog axes we track (log when crossing thresholds)
    ANALOG_AXES = ["rightTrig", "leftTrig", "rightGrip", "leftGrip"]

    def __init__(self, log_path):
        self.log_file = open(log_path, "w")
        self.prev_bool = {}
        self.prev_analog = {}
        self.t0 = time.time()
        self._write(f"# VR button log started at {datetime.datetime.now().isoformat()}")
        self._write(f"# {'time_s':>8s}  {'event':<12s}  {'button':<12s}  {'value'}")

    def _write(self, line):
        self.log_file.write(line + "\n")
        self.log_file.flush()

    def update(self, buttons):
        t = time.time() - self.t0

        # Boolean buttons: detect press / release
        for key in self.BOOL_BUTTONS:
            if key not in buttons:
                continue
            cur = bool(buttons[key])
            prev = self.prev_bool.get(key)
            if prev is not None and cur != prev:
                event = "PRESS" if cur else "RELEASE"
                self._write(f"  {t:>8.3f}  {event:<12s}  {key:<12s}")
                print(f"  [{t:7.3f}s] {event:<8s} {key}")
            self.prev_bool[key] = cur

        # Analog axes: log value when crossing 0.1 or 0.5 thresholds
        for key in self.ANALOG_AXES:
            if key not in buttons:
                continue
            val = buttons[key]
            if isinstance(val, (tuple, list)):
                val = val[0]
            prev_val = self.prev_analog.get(key, 0.0)
            for thresh in [0.1, 0.5, 0.9]:
                if (prev_val < thresh) != (val < thresh):
                    direction = "ABOVE" if val >= thresh else "BELOW"
                    self._write(f"  {t:>8.3f}  {'ANALOG':<12s}  {key:<12s}  {direction} {thresh} (val={val:.3f})")
                    print(f"  [{t:7.3f}s] ANALOG   {key} {direction} {thresh} (val={val:.3f})")
            self.prev_analog[key] = val

        # Joystick: log when magnitude crosses 0.3
        for key in ["rightJS", "leftJS"]:
            if key not in buttons:
                continue
            val = buttons[key]
            if isinstance(val, (tuple, list)) and len(val) >= 2:
                mag = np.sqrt(val[0] ** 2 + val[1] ** 2)
                prev_mag = self.prev_analog.get(key + "_mag", 0.0)
                if (prev_mag < 0.3) != (mag < 0.3):
                    direction = "ACTIVE" if mag >= 0.3 else "IDLE"
                    self._write(f"  {t:>8.3f}  {'JOYSTICK':<12s}  {key:<12s}  {direction} (x={val[0]:.2f}, y={val[1]:.2f})")
                    print(f"  [{t:7.3f}s] JOYSTICK {key} {direction} (x={val[0]:.2f}, y={val[1]:.2f})")
                self.prev_analog[key + "_mag"] = mag

    def close(self):
        self._write(f"# Log ended at {datetime.datetime.now().isoformat()}")
        self.log_file.close()


class LivePlot3D:
    """Live 3D plot showing controller position and orientation triad."""

    def __init__(self, trail_len=1000):
        self.trail_len = trail_len
        self.positions = []

        plt.ion()
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_title("VR Controller Pose (raw)")

        # Trail line
        self.trail_line, = self.ax.plot([], [], [], "b-", alpha=0.4, linewidth=1)
        # Current position marker
        self.pos_marker, = self.ax.plot([], [], [], "ko", markersize=6)
        # Orientation triad (X=red, Y=green, Z=blue)
        self.triad_lines = [
            self.ax.plot([], [], [], color=c, linewidth=2)[0]
            for c in ["red", "green", "blue"]
        ]
        self.triad_length = 0.05  # 5 cm arrows

        # Text annotations
        self.pos_text = self.ax.text2D(0.02, 0.95, "", transform=self.ax.transAxes, fontsize=9, family="monospace")
        self.rot_text = self.ax.text2D(0.02, 0.90, "", transform=self.ax.transAxes, fontsize=9, family="monospace")

        self.fig.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)

    def update(self, pos, euler, rot_mat):
        self.positions.append(pos.copy())
        if len(self.positions) > self.trail_len:
            self.positions = self.positions[-self.trail_len:]

        trail = np.array(self.positions)
        self.trail_line.set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])
        self.pos_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])

        # Draw orientation triad
        for i, line in enumerate(self.triad_lines):
            axis = rot_mat[:3, i] * self.triad_length
            line.set_data_3d(
                [pos[0], pos[0] + axis[0]],
                [pos[1], pos[1] + axis[1]],
                [pos[2], pos[2] + axis[2]],
            )

        # Update text
        self.pos_text.set_text(f"pos: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f} m")
        self.rot_text.set_text(f"rot: r={euler[0]:+.1f}  p={euler[1]:+.1f}  y={euler[2]:+.1f} deg")

        # Auto-scale axes
        if len(self.positions) > 5:
            margin = 0.1
            mins = trail.min(axis=0) - margin
            maxs = trail.max(axis=0) + margin
            # Keep axes equal aspect
            center = (mins + maxs) / 2
            half_range = max((maxs - mins).max() / 2, 0.05)
            self.ax.set_xlim(center[0] - half_range, center[0] + half_range)
            self.ax.set_ylim(center[1] - half_range, center[1] + half_range)
            self.ax.set_zlim(center[2] - half_range, center[2] + half_range)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Debug Meta Quest controller readouts")
    parser.add_argument("--left", action="store_true", help="Track left controller (default: right)")
    parser.add_argument("--hz", type=int, default=20, help="Polling frequency (default: 20)")
    parser.add_argument("--no-plot", action="store_true", help="Disable 3D plot (log only)")
    parser.add_argument("--log-dir", type=str, default="logs", help="Log output directory")
    args = parser.parse_args()

    controller_id = "l" if args.left else "r"
    side_name = "left" if args.left else "right"
    period = 1.0 / args.hz

    # Ctrl+C handler
    running = True
    def _sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _sigint)

    # Create log directory
    os.makedirs(args.log_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(args.log_dir, f"vr_readout_{timestamp}.log")

    print("=" * 55)
    print(f"VR Controller Debug  ({side_name} hand)")
    print("=" * 55)

    # Initialize OculusReader
    print("Connecting to Oculus Quest via ADB ...")
    reader = OculusReader()
    print("[OK] OculusReader started")

    # Wait for first data
    print("Waiting for controller data ...")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        transforms, buttons = reader.get_transformations_and_buttons()
        if controller_id in transforms:
            print(f"[OK] {side_name} controller detected")
            break
        time.sleep(0.1)
    else:
        print(f"[FAIL] No data from {side_name} controller after 10s")
        reader.stop()
        sys.exit(1)

    # Initialize logger and plot
    logger = ButtonLogger(log_path)
    print(f"Logging button events to: {log_path}")

    plot = None
    if not args.no_plot:
        plot = LivePlot3D()
        print("[OK] 3D plot window opened")

    print()
    print("Recording ... press SPACE to pause/resume, Ctrl+C to stop")
    print("-" * 55)

    count = 0
    t0 = time.time()
    paused = False

    # Put terminal in raw mode so we can detect single key presses
    old_term = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    def check_key():
        """Non-blocking single-key read (cbreak mode)."""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    try:
        while running:
            loop_start = time.time()

            # Check for space key to toggle pause
            key = check_key()
            if key == " ":
                paused = not paused
                if paused:
                    print("\n  ** PAUSED ** — rotate/zoom the plot, press SPACE to resume")
                else:
                    print("  ** RESUMED **")
            elif key == "q":
                break

            if paused:
                # Keep matplotlib responsive for interaction
                if plot is not None:
                    plot.fig.canvas.flush_events()
                time.sleep(0.05)
                continue

            transforms, buttons = reader.get_transformations_and_buttons()

            # Log button events
            if buttons:
                logger.update(buttons)

            # Update plot with controller pose
            if controller_id in transforms:
                mat4 = np.asarray(transforms[controller_id])
                pos, euler = mat4_to_pos_euler(mat4)

                if plot is not None:
                    plot.update(pos, euler, mat4)

                count += 1
                if count % (args.hz * 2) == 0:  # print status every 2s
                    elapsed = time.time() - t0
                    sys.stdout.write(
                        f"\r  [{elapsed:.0f}s] "
                        f"pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) "
                        f"rot=({euler[0]:+.0f}, {euler[1]:+.0f}, {euler[2]:+.0f})   "
                    )
                    sys.stdout.flush()

            # Regulate frequency
            sleep_t = period - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal to normal mode
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)

    print(f"\n\nStopping after {count} samples ({time.time() - t0:.1f}s)")
    logger.close()
    print(f"Log saved to: {log_path}")
    reader.stop()
    os._exit(0)


if __name__ == "__main__":
    main()
