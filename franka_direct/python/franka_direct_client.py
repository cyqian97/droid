"""
FrankaDirectClient — Python gRPC client for franka_direct servers.

Connects to either of the two C++ servers that drive the Franka FR3
directly via libfranka (no Polymetis, no IK, no TorchScript):

  - franka_server          (joint torque control)   → use set_joint_target()
  - franka_server_cartesian (Cartesian velocity PD) → use set_ee_target()

Both servers share the same gRPC proto (franka_control.proto) and this
client.  The gripper, state query, reset, and stop RPCs work with either.
"""

import os
import sys
import time

# Make sure the generated protobuf stubs are importable
sys.path.insert(0, os.path.dirname(__file__))

import grpc
import franka_control_pb2 as pb2
import franka_control_pb2_grpc as pb2_grpc


class FrankaDirectClient:
    """Client for the C++ franka_server gRPC service."""

    def __init__(self, host: str = "192.168.1.6", port: int = 50052, timeout: float = 5.0):
        self.channel = grpc.insecure_channel(f"{host}:{port}")
        self.stub    = pb2_grpc.FrankaControlStub(self.channel)
        self.timeout = timeout

    # ── State ──────────────────────────────────────────────────────────────

    def get_robot_state(self):
        """
        Returns dict with keys:
            pose              – list[16] col-major O_T_EE (actual measured pose)
            q                 – list[7]  actual measured joint positions (rad)
            target_q          – list[7]  current interpolated joint command (rad)
            cmd_success_rate  – float in [0, 1]
            ready             – bool
            error             – str (empty if OK)
            gripper_width     – float, actual gripper opening in metres [0, 0.08]
            gripper_grasping  – bool, True while gripping an object

        Use ``target_q`` (not ``q``) as the starting point for new
        SetJointTarget calls to avoid velocity discontinuities.
        """
        resp = self.stub.GetRobotState(pb2.Empty(), timeout=self.timeout)
        return {
            "pose":             list(resp.pose),
            "q":                list(resp.q),
            "dq":               list(resp.dq),
            "target_q":         list(resp.target_q),
            "target_pose":      list(resp.target_pose),
            "cmd_success_rate": resp.cmd_success_rate,
            "ready":            resp.ready,
            "error":            resp.error,
            "gripper_width":    resp.gripper_width,
            "gripper_grasping": resp.gripper_grasping,
        }

    def wait_until_ready(self, timeout: float = 10.0, poll_hz: float = 5.0):
        """Block until franka_server reports ready (control loop running)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                s = self.get_robot_state()
                if s["ready"]:
                    return s
                if s["error"]:
                    raise RuntimeError(f"franka_server error: {s['error']}")
            except grpc.RpcError:
                pass
            time.sleep(1.0 / poll_hz)
        raise TimeoutError("franka_server did not become ready in time")

    # ── Control ────────────────────────────────────────────────────────────

    def set_joint_target(self, q_7: list):
        """
        Set desired joint positions.

        Args:
            q_7: 7-element list of joint positions in radians.
        Returns:
            (success: bool, message: str)
        """
        req  = pb2.JointTarget(q=list(q_7))
        resp = self.stub.SetJointTarget(req, timeout=self.timeout)
        return resp.success, resp.message

    def set_ee_target(self, pose_16: list):
        """
        Set desired end-effector pose (Cartesian server).

        Args:
            pose_16: 16-element list — 4x4 homogeneous transform, column-major.
                     Same layout as libfranka O_T_EE.
        Returns:
            (success: bool, message: str)
        """
        req  = pb2.EETarget(pose=list(pose_16))
        resp = self.stub.SetEETarget(req, timeout=self.timeout)
        return resp.success, resp.message

    def set_gripper_target(self, width_m: float, speed_mps: float = 0.1):
        """
        Set desired Franka Hand gripper opening width.

        Args:
            width_m:   target opening in metres, range [0, 0.08].
                       0.0 = fully closed, 0.08 = fully open.
            speed_mps: max finger speed in m/s, range [0.01, 0.2].
        Returns:
            (success: bool, message: str)
        """
        req  = pb2.GripperTarget(width=float(width_m), speed=float(speed_mps))
        resp = self.stub.SetGripperTarget(req, timeout=self.timeout)
        return resp.success, resp.message

    def reset_to_joints(self, q_7: list, speed: float = 0.3, max_duration: float = 5.0,
                         timeout: float = 30.0):
        """
        Move robot to a joint configuration using smooth joint-space motion.

        Blocks until the server completes the move (or errors out).

        Args:
            q_7:          7-element list of joint positions in radians.
            speed:        fraction of max joint velocity [0..1] (default 0.3).
            max_duration: maximum seconds for the move (default 5.0).
            timeout:      gRPC call timeout in seconds (default 30).
        Returns:
            (success: bool, message: str)
        """
        req = pb2.JointResetTarget(q=list(q_7), speed=float(speed),
                                   max_duration=float(max_duration))
        resp = self.stub.ResetToJoints(req, timeout=timeout)
        return resp.success, resp.message

    def stop(self):
        """Tell the server to finish the control loop gracefully."""
        resp = self.stub.Stop(pb2.Empty(), timeout=self.timeout)
        return resp.success

    def close(self):
        self.channel.close()

    # ── Context manager ────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        try:
            self.stop()
        except Exception:
            pass
        self.close()
