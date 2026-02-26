#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

# -----------------------------------------------------------------------------
# Hardcoded config (edit here)
# -----------------------------------------------------------------------------
# Publish PoseStamped delta_pose as independent sine waves on each DOF.
#
# Units:
# - Translation: meters
# - Rotation: radians (roll/pitch/yaw, ZYX convention applied)
#
# Notes:
# - This is a quick test tool, so CLI args are intentionally omitted.
# - The TT publisher consumes this as a delta pose (not an absolute pose).

TOPIC = "/admittance_controller/delta_pose_cmd"
FRAME_ID = "ur_arm_tool0"
RATE_HZ = 100.0

# Translation DOFs
DX_AMP, DX_FREQ, DX_PHASE, DX_BIAS = 0.025, 0.25, 0.0, 0.0
DY_AMP, DY_FREQ, DY_PHASE, DY_BIAS = 0.00, 0.5, 0.0, 0.0
DZ_AMP, DZ_FREQ, DZ_PHASE, DZ_BIAS = 0.00, 0.5, 0.0, 0.0

# Rotation DOFs (axis-angle in radians): axis (ax,ay,az) and angle theta.
# If the axis is (0,0,0) or theta ~ 0, identity rotation is published.
AX, AY, AZ = 1.0, 0.0, 0.0
THETA_AMP, THETA_FREQ, THETA_PHASE, THETA_BIAS = 0.00, 0.5, 0.0, 0.0


def _sine(t: float, amp: float, freq: float, phase: float, bias: float) -> float:
    return bias + amp * math.sin(2.0 * math.pi * freq * t + phase)


def _quat_from_axis_angle(ax: float, ay: float, az: float, theta: float):
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(theta) < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = ax / n, ay / n, az / n
    half = 0.5 * theta
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


class PubDeltaPose(Node):
    def __init__(self):
        super().__init__("pub_delta_pose")
        self._pub = self.create_publisher(PoseStamped, TOPIC, 10)
        self._t0 = self.get_clock().now()

        period = 1.0 / max(1e-3, float(RATE_HZ))
        self.create_timer(period, self._on_timer)
        self.get_logger().info(f"Publishing delta_pose sine: topic={TOPIC} frame_id={FRAME_ID} rate={RATE_HZ:.1f}Hz")

    def _on_timer(self):
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9

        dx = _sine(t, DX_AMP, DX_FREQ, DX_PHASE, DX_BIAS)
        dy = _sine(t, DY_AMP, DY_FREQ, DY_PHASE, DY_BIAS)
        dz = _sine(t, DZ_AMP, DZ_FREQ, DZ_PHASE, DZ_BIAS)

        theta = _sine(t, THETA_AMP, THETA_FREQ, THETA_PHASE, THETA_BIAS)
        qw, qx, qy, qz = _quat_from_axis_angle(AX, AY, AZ, theta)

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = FRAME_ID
        msg.pose.position.x = dx
        msg.pose.position.y = dy
        msg.pose.position.z = dz
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = PubDeltaPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
