import os
from pathlib import Path
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    axis_linear = LaunchConfiguration('axis_linear', default='1')
    axis_angular = LaunchConfiguration('axis_angular', default='0')  
    vlinear = LaunchConfiguration('vlinear', default='0.3')
    vangular = LaunchConfiguration('vangular', default='0.5')  
    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
  
    return LaunchDescription([
        wheeltec_robot,
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node',
            name='joy_node', 
            output="screen",),

        launch_ros.actions.Node(
            package='wheeltec_joy', 
            executable='wheeltec_joy',
            name='wheeltec_joy',  
            parameters=[{'axis_linear': axis_linear, 
                         'axis_angular': axis_angular, 
                         'vlinear': vlinear,
                         'vangular': vangular}],
            output="screen",)

  ])
