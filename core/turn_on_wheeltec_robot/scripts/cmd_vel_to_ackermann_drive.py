#!/usr/bin/env python3
# Author: christoph.roesmann@tu-dortmund.de

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0.0
    radius = v / omega
    return math.atan(wheelbase / radius)


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')

        # Platform geometry. Defaults match the mini_akm chassis.
        # Wheelbase reference values for stock Wheeltec Ackermann platforms:
        #   mini_akm       0.143
        #   senior_akm     0.320
        #   top_akm_bs     0.503
        #   top_akm_dl     0.549
        self.declare_parameter('wheelbase', 0.143)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('cmd_angle_instead_rotvel', False)
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('input_topic', 'cmd_vel')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.frame_id = self.get_parameter('frame_id').value
        self.cmd_angle_instead_rotvel = self.get_parameter('cmd_angle_instead_rotvel').value
        out_topic = self.get_parameter('output_topic').value
        in_topic = self.get_parameter('input_topic').value

        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(AckermannDriveStamped, out_topic, qos)
        self.create_subscription(Twist, in_topic, self.cmd_callback, qos)
        self.get_logger().info(
            f"cmd_vel_to_ackermann_drive started (wheelbase={self.wheelbase} m, frame_id='{self.frame_id}')"
        )

    def cmd_callback(self, data):
        v = data.linear.x
        if self.cmd_angle_instead_rotvel:
            steering = data.angular.z
        else:
            steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(v)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
