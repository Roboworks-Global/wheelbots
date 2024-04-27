#!/usr/bin/env python3
# Author: christoph.roesmann@tu-dortmund.de

#import rospy, math
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float32

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
          #if radius<0.38:
          #  radius=0.38
    return math.atan(wheelbase / radius)


def cmd_callback(data):
          
    v = data.linear.x
    cmd_angle_instead_rotvel = False
    wheelbase = 0.143 #for mini_akm
    #wheelbase = 0.320 #for senior_akm
    #wheelbase = 0.503 #for top_akm_bs
    #wheelbase = 0.549 #for top_akm_dl
    if cmd_angle_instead_rotvel:
        steering = data.angular.z
    else:
        steering = 1.0*convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
    msg = AckermannDriveStamped()
    msg.header.stamp = NODE.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = steering
    msg.drive.speed = v
    PUBLISHER.publish(msg)

def main():
    rclpy.init()
    global NODE
    global PUBLISHER
    global frame_id
    global wheelbase
    global cmd_angle_instead_rotvel
    wheelbase = 0.143
    frame_id = "odom_combined"
    cmd_angle_instead_rotvel = False
    print("Node 'cmd_vel_to_ackermann_drive' started")
    qos = QoSProfile(depth=10)
    NODE = rclpy.create_node('cmd_vel_to_ackermann_drive')
    PUBLISHER = NODE.create_publisher(AckermannDriveStamped,'/ackermann_cmd', qos)
    NODE.create_subscription(Twist,'cmd_vel',cmd_callback,qos)
    rclpy.spin(NODE)
    NODE.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

