"""
FrankaDirectClient — Python gRPC client for franka_server.

Connects to the C++ franka_server that drives the robot via libfranka's
joint position controller (no Polymetis, no IK, no TorchScript).
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

        Use ``target_q`` (not ``q``) as the starting point for new
        SetJointTarget calls to avoid velocity discontinuities.
        """
        resp = self.stub.GetRobotState(pb2.Empty(), timeout=self.timeout)
        return {
            "pose":             list(resp.pose),
            "q":                list(resp.q),
            "target_q":         list(resp.target_q),
            "cmd_success_rate": resp.cmd_success_rate,
            "ready":            resp.ready,
            "error":            resp.error,
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
