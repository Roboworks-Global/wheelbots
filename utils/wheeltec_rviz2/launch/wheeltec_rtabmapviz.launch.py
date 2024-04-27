import os
import launch
import launch.actions
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    parameters=[{
          'queue_size':20,
          'frame_id':'camera_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True}]

    remappings=[
          ('odom', '/odom_combined'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Nodes to launch

        Node(
            package='rtabmap_ros', executable='rtabmapviz',
            parameters=parameters,
            remappings=remappings),

    ])
