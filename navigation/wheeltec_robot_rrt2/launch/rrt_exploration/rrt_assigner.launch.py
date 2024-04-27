import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    configured_params = [os.path.join(get_package_share_directory("wheeltec_robot_rrt"), 'config', 'exploration_params.yaml'),
                            {'use_sim_time': use_sim_time}]
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
         
        Node(
            package='wheeltec_robot_rrt',
            executable='assigner',
            name='assigner',
            output='both',
            parameters=configured_params),
        '''    
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_assigner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['assigner']}]),
      '''

    ])
