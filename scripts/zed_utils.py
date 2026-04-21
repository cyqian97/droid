"""
ZED camera recording utilities.

CameraRecorder encapsulates two ZED cameras and MP4 video writing.
grab() is meant to be called once per control loop iteration so that
camera frames are frame-locked to the robot control rate.

Typical usage:
    rec = CameraRecorder(serial0=12345, serial1=67890, fps=15)
    rec.open()

    # inside control loop:
    rec.grab()          # grabs + writes if recording

    rec.start()         # begin recording (A button)
    rec.stop()          # stop and save (B button)

    rec.close()         # on exit
"""

import datetime
import os
from typing import Optional
import threading

import cv2
import numpy as np

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False


# ── Low-level helpers (used by CameraRecorder and by openvla_oft_franka_client) ──

def list_cameras() -> list:
    """Return a list of dicts with serial/model info for all detected ZED cameras."""
    if not ZED_AVAILABLE:
        return []
    device_list = sl.Camera.get_device_list()
    return [{"serial": d.serial_number, "model": str(d.camera_model)} for d in device_list]


def open_camera(serial: Optional[int], fps: int, resolution: str, label: str = ""):
    """Open one ZED camera. Returns (sl.Camera, sl.RuntimeParameters)."""
    if not ZED_AVAILABLE:
        raise RuntimeError("pyzed not installed.")
    res_map = {
        "HD2K":   sl.RESOLUTION.HD2K,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720":  sl.RESOLUTION.HD720,
        "VGA":    sl.RESOLUTION.VGA,
    }
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = res_map.get(resolution, sl.RESOLUTION.HD720)
    params.camera_fps = fps
    params.depth_mode = sl.DEPTH_MODE.NONE
    if serial is not None:
        params.set_from_serial_number(serial)
    status = zed.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        tag = f" [{label}]" if label else ""
        sn  = f" serial={serial}" if serial else ""
        raise RuntimeError(f"Failed to open ZED{tag}{sn}: {status}")
    sn_actual = zed.get_camera_information().serial_number
    tag = f" [{label}]" if label else ""
    print(f"[OK] ZED{tag} opened  serial={sn_actual}  {resolution} @ {fps} fps")
    return zed, sl.RuntimeParameters()


def grab_bgr(zed, runtime) -> Optional[np.ndarray]:
    """Grab one LEFT-eye BGR frame. Returns (H, W, 3) ndarray or None."""
    if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
        return None
    mat = sl.Mat()
    zed.retrieve_image(mat, sl.VIEW.LEFT)
    return mat.get_data()[:, :, :3].copy()


# ── CameraRecorder ────────────────────────────────────────────────────────────

class CameraRecorder:
    """Two-camera recorder whose grab() is driven by the caller's control loop.

    Frame rate of the saved video equals the rate at which grab() is called,
    which keeps the recording frame-locked to the robot control loop.
    The fps argument sets the ZED hardware capture rate (should be >= control hz)
    and the VideoWriter fps (used as the playback speed of the saved file).

    Parameters
    ----------
    serial0, serial1 : int or None
        ZED camera serial numbers. None = first/second detected.
    fps : int
        Camera hardware capture rate and VideoWriter playback fps.
    resolution : str
        One of HD2K, HD1080, HD720, VGA.
    out_dir : str
        Root directory; each recording goes into a timestamped subdirectory.
    """

    def __init__(self, serial0: Optional[int], serial1: Optional[int],
                 fps: int = 60, resolution: str = "HD720",
                 out_dir: str = "scripts/recordings"):
        if not ZED_AVAILABLE:
            raise RuntimeError("pyzed not installed — cannot use CameraRecorder.")
        self._serial0   = serial0
        self._serial1   = serial1
        self._fps       = fps
        self._res       = resolution
        self._out_dir   = out_dir

        self._zed0 = self._zed1 = None
        self._rt0  = self._rt1  = None
        self._shape0 = self._shape1 = None   # (H, W)

        self._recording  = False
        self._writers    = []
        self._paths      = []
        self._n_frames   = 0
        self._lock       = threading.Lock()  # protects _recording + _writers

    # ── Setup / teardown ──────────────────────────────────────────────────────

    def open(self):
        """Open both cameras and warm up (grabs first frame to determine shape)."""
        detected = list_cameras()
        if detected:
            print(f"[ZED] {len(detected)} camera(s) detected:")
            for d in detected:
                print(f"       serial={d['serial']}  model={d['model']}")
        else:
            print("[ZED] No cameras detected by sl.Camera.get_device_list()")

        self._zed0, self._rt0 = open_camera(self._serial0, self._fps, self._res, "cam0")
        self._zed1, self._rt1 = open_camera(self._serial1, self._fps, self._res, "cam1")

        print("[ZED] Warming up cameras (waiting for first frame)...")
        f0 = f1 = None
        while f0 is None:
            f0 = grab_bgr(self._zed0, self._rt0)
        print("      cam0 first frame received")
        while f1 is None:
            f1 = grab_bgr(self._zed1, self._rt1)
        print("      cam1 first frame received")
        self._shape0 = f0.shape[:2]   # (H, W)
        self._shape1 = f1.shape[:2]
        print(f"  cam0 frame: {self._shape0[1]}×{self._shape0[0]}  "
              f"cam1 frame: {self._shape1[1]}×{self._shape1[0]}")

    def close(self):
        """Stop any in-progress recording and close both cameras."""
        with self._lock:
            if self._recording:
                self._flush()
        if self._zed0 is not None:
            self._zed0.close()
        if self._zed1 is not None:
            self._zed1.close()

    # ── Per-loop call ─────────────────────────────────────────────────────────

    def grab(self):
        """Grab one frame from each camera and write to video if recording.

        Call this once per control loop iteration to keep camera output
        frame-locked to the robot control rate.
        """
        f0 = grab_bgr(self._zed0, self._rt0)
        f1 = grab_bgr(self._zed1, self._rt1)
        with self._lock:
            if self._recording and f0 is not None and f1 is not None:
                self._writers[0].write(f0)
                self._writers[1].write(f1)
                self._n_frames += 1

    # ── Recording control ─────────────────────────────────────────────────────

    def start(self):
        """Begin recording. Creates a new timestamped subdirectory."""
        with self._lock:
            if self._recording:
                return
            ts     = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            subdir = os.path.join(self._out_dir, ts)
            os.makedirs(subdir, exist_ok=True)

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writers  = []
            self._paths    = []
            self._n_frames = 0
            for i, (h, w) in enumerate([self._shape0, self._shape1]):
                path = os.path.join(subdir, f"cam{i}.mp4")
                self._writers.append(cv2.VideoWriter(path, fourcc, self._fps, (w, h)))
                self._paths.append(path)
            self._recording = True

        for p in self._paths:
            print(f"  [REC] → {p}")

    def stop(self):
        """Stop recording, flush and close video files.

        Returns (n_frames, paths).
        """
        with self._lock:
            if not self._recording:
                return 0, []
            n, paths = self._flush()
        duration = n / self._fps if self._fps > 0 else 0
        for p in paths:
            print(f"  [SAVED] {p}  ({n} frames, {duration:.1f} s)")
        return n, paths

    @property
    def is_recording(self) -> bool:
        return self._recording

    # ── Internal ──────────────────────────────────────────────────────────────

    def _flush(self):
        """Release writers. Must be called with self._lock held."""
        for w in self._writers:
            w.release()
        n, paths = self._n_frames, list(self._paths)
        self._writers   = []
        self._paths     = []
        self._n_frames  = 0
        self._recording = False
        return n, paths