#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped


def main(args=None):
    rclpy.init(args=args)

    boundary_points_publisher_node = Node('boundary_points_publisher_node')
    boundary_points_publisher_node.publisher_ = boundary_points_publisher_node.create_publisher(PointStamped, 'clicked_point', 10)

    msg = PointStamped()

    boundary_points = [[10.0,10.0], [-10.0,10.0], [-10.0, -10.0], [10.0, -10.0], [0.0,0.0]]
    for point in boundary_points:
        msg.point.x = point[0]
        msg.point.y = point[1]

        boundary_points_publisher_node.publisher_.publish(msg)
        boundary_points_publisher_node.get_logger().info('Publishing: {%f, %f}' % (msg.point.x, msg.point.y))

    boundary_points_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





