import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    configured_params = [os.path.join(get_package_share_directory("wheeltec_robot_rrt"), 'config', 'exploration_params.yaml'),
                            {'use_sim_time': use_sim_time}]
    wheeltec_rrt_dir = get_package_share_directory('wheeltec_robot_rrt')

    return LaunchDescription([

        Node(
            package='wheeltec_robot_rrt',
            executable='local_rrt',
            name='local_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='wheeltec_robot_rrt',
            executable='global_rrt',
            name='global_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='wheeltec_robot_rrt',
            executable='filter',
            name='filter',
            output='both',
            parameters=configured_params),
            
        Node(
            package='wheeltec_robot_rrt',
            executable='assigner',
            name='assigner',
            output='both',
            parameters=configured_params),

    ])
