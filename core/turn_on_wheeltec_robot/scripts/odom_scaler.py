#!/usr/bin/env python3
"""Rescale wheel odometry to match measured ground truth.

The Wheeltec STM32 firmware integrates wheel-encoder ticks into a
nav_msgs/Odometry stream on /odom. On this mini_akm the integrated
distance overcounts by a consistent factor (real motion is ~0.60 of
what /odom reports — measured 2026-09-29 with a ~91 cm ground-truth
translation vs. a 153 cm odom delta). Since we can't rebuild the STM32
firmware from here, this node reads /odom, multiplies pose.position
and twist.linear by a configurable factor, and republishes on
/odom_scaled. Downstream consumers (EKF, Nav2) can be pointed at
/odom_scaled instead of /odom.

Not wired into any launch by default — running it is a deliberate
choice per platform, since the factor is specific to this robot's
wheel diameter / encoder resolution.

Usage:

    ros2 run turn_on_wheeltec_robot odom_scaler.py \\
        --ros-args -p scale_factor:=0.60

    # Then remap /odom to /odom_scaled in your EKF config, or
    # remap /odom_scaled -> /odom in the launch:
    #   ros2 run <consumer> <node> --ros-args -r /odom:=/odom_scaled

Parameters:
    scale_factor     (double, default 0.60)   applied to pose.position.{x,y,z}
                                               and twist.linear.{x,y,z}
    input_topic      (string, default /odom)
    output_topic     (string, default /odom_scaled)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomScaler(Node):
    def __init__(self) -> None:
        super().__init__("odom_scaler")

        self.declare_parameter("scale_factor", 0.60)
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odom_scaled")

        self.k = float(self.get_parameter("scale_factor").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self._cb, 10)

        self.get_logger().info(
            f"odom_scaler: {in_topic!r} -> {out_topic!r}, "
            f"scale_factor={self.k:.4f}"
        )

    def _cb(self, msg: Odometry) -> None:
        # Scale pose.position and twist.linear. Orientation and angular
        # velocity pass through unchanged — the wheel scale factor only
        # affects translation.
        msg.pose.pose.position.x *= self.k
        msg.pose.pose.position.y *= self.k
        msg.pose.pose.position.z *= self.k
        msg.twist.twist.linear.x *= self.k
        msg.twist.twist.linear.y *= self.k
        msg.twist.twist.linear.z *= self.k
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = OdomScaler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
