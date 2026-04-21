"""
Minimal VR controller for Franka teleoperation via Meta Quest.

Reads controller poses and buttons from OculusReader, handles coordinate
frame alignment (orientation reset + origin capture), and outputs raw
pose deltas (position + rotation matrix) instead of velocities.

Usage:
    from vr_controller import VRController

    vr = VRController(right_controller=True)
    # Hold grip trigger to enable, press joystick to calibrate orientation
    info = vr.get_info()
    pos_delta, rot_delta, gripper = vr.get_pose_delta()
"""

import threading
import time

import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "droid", "oculus_reader"))
from oculus_reader.reader import OculusReader


class VRController:
    """Minimal VR controller that outputs pose deltas.

    Parameters
    ----------
    right_controller : bool
        True for right hand, False for left.
    """

    def __init__(self, right_controller=True):
        self.controller_id = "r" if right_controller else "l"
        self.vr_to_global_mat = np.eye(4)

        # Internal state (written by background thread, read by main thread)
        self._lock = threading.Lock()
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }

        # Alignment flags
        self._reset_orientation = True
        self._origin_just_reset = False   # signals script to capture robot origin
        self._reset_origin = True

        # VR origin (captured when grip trigger is pressed)
        self._vr_origin_pos = None
        self._vr_origin_rot = None

        # Start OculusReader and background polling thread
        self.oculus_reader = OculusReader()
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self.oculus_reader.stop()

    # ── Background polling (same logic as VRPolicy._update_internal_state) ──

    def _poll_loop(self, hz=-1, timeout_sec=5):
        last_read = time.time()
        btn_id = self.controller_id.upper()  # "R" or "L"

        while self._running:
            if hz > 0:
                time.sleep(1.0 / hz)

            poses, buttons = self.oculus_reader.get_transformations_and_buttons()

            # Controller connectivity check
            dt = time.time() - last_read
            with self._lock:
                self._state["controller_on"] = dt < timeout_sec

            if not poses:
                continue

            grip_key = btn_id + "G"          # "RG" or "LG"
            joy_key = btn_id + "J"           # "RJ" or "LJ"

            prev_enabled = self._state["movement_enabled"]
            cur_enabled = buttons.get(grip_key, False)
            toggled = prev_enabled != cur_enabled

            with self._lock:
                self._state["poses"] = poses
                self._state["buttons"] = buttons
                self._state["movement_enabled"] = cur_enabled
                self._state["controller_on"] = True
            last_read = time.time()

            # Origin reset on grip toggle
            if toggled:
                self._reset_origin = True

            # Orientation reset on joystick press
            # # Temporally disable this reset
            # should_reset_orient = buttons.get(joy_key, False)
            # if should_reset_orient or self._reset_orientation:
            #     if self.controller_id in poses:
            #         raw = np.asarray(poses[self.controller_id])
            #         try:
            #             self.vr_to_global_mat = np.linalg.inv(raw)
            #         except np.linalg.LinAlgError:
            #             self.vr_to_global_mat = np.eye(4)
            #         # Stop updating once grip is held or joystick released
            #         stop = cur_enabled or buttons.get(joy_key, False)
            #         if stop:
            #             self._reset_orientation = False

    # ── Public API ──────────────────────────────────────────────────────────

    def get_info(self):
        """Return button/status dict (same keys as VRPolicy.get_info)."""
        with self._lock:
            b = self._state["buttons"]
            if self.controller_id == "r":
                success, failure = b.get("A", False), b.get("B", False)
            else:
                success, failure = b.get("X", False), b.get("Y", False)
            return {
                "success": success,
                "failure": failure,
                "movement_enabled": self._state["movement_enabled"],
                "controller_on": self._state["controller_on"],
            }

    def get_pose_delta(self):
        """Compute VR controller pose delta from origin.

        Returns
        -------
        pos_delta : np.ndarray[3] or None
            Translation offset from VR origin in robot frame (metres).
        rot_delta : np.ndarray[3,3] or None
            Rotation offset from VR origin (rotation matrix).
        gripper : float or None
            Raw index trigger value [0, 1].

        Returns (None, None, None) if controller data is not available or
        if the origin was just reset (signals the caller to capture robot origin).
        """
        with self._lock:
            poses = self._state["poses"]
            buttons = self._state["buttons"]

        if self.controller_id not in poses:
            return None, None, None

        # Apply coordinate transforms
        raw = np.asarray(poses[self.controller_id])
        transformed = self.vr_to_global_mat @ raw

        cur_pos = transformed[:3, 3].copy()
        cur_rot = transformed[:3, :3].copy()

        # Read gripper trigger
        trig_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
        trig_val = buttons.get(trig_key, (0.0,))
        gripper = trig_val[0] if isinstance(trig_val, (tuple, list)) else float(trig_val)

        # Origin reset: capture VR origin, signal caller
        if self._reset_origin:
            self._vr_origin_pos = cur_pos
            self._vr_origin_rot = cur_rot
            self._reset_origin = False
            self._origin_just_reset = True
            return None, None, None

        if self._vr_origin_pos is None:
            return None, None, None

        # Pose delta relative to VR origin
        pos_delta = cur_pos - self._vr_origin_pos
        rot_delta = cur_rot @ self._vr_origin_rot.T

        return pos_delta, rot_delta, gripper

    @property
    def origin_just_reset(self):
        """True once after each origin reset. Cleared after reading."""
        if self._origin_just_reset:
            self._origin_just_reset = False
            return True
        return False

    def reset_state(self):
        """Full reset: re-calibrate orientation + origin on next reading."""
        with self._lock:
            self._state["poses"] = {}
            self._state["buttons"] = {"A": False, "B": False, "X": False, "Y": False}
            self._state["movement_enabled"] = False
        self._reset_orientation = True
        self._reset_origin = True
        self._vr_origin_pos = None
        self._vr_origin_rot = None
