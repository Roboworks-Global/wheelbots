from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

     find_robot_action_server = Node(
            package='wheeltec_robot_rrt',
            executable='find_robot_action_server',
            name='find_robot_action_server',
            output='both')

     pick_robot_action_server = Node(
            package='wheeltec_robot_rrt',
            executable='pick_robot_action_server',
            name='pick_robot_action_server',
            output='both')

     return LaunchDescription([
          find_robot_action_server,
          pick_robot_action_server,
     ])
