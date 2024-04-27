import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time', default=False)
     use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value=use_sim_time)
     
     wheeltec_rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
     wheeltec_slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
     
     nav2_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
               os.path.join(wheeltec_rrt_dir, 'launch', 'navigation', 'nav2_bringup_launch.py')),
               launch_arguments={'slam': "True",
                              'map': os.path.join(wheeltec_rrt_dir, 'map_data', 'image_map.yaml'),
                              'use_sim_time': use_sim_time,
                              'params_file': os.path.join(wheeltec_rrt_dir, 'config', 'nav2_params.yaml'),
                              'default_bt_xml_filename' : os.path.join(wheeltec_rrt_dir, 'behaviour_trees', 'navigate_w_replanning_time.xml'),
                              'autostart': 'True'}.items()
     )

     rrt_exploration = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration', 'rrt_exploration.launch.py')
          ),
     )
     
     wheeltec_slam = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_slam_dir, 'launch', 'online_sync.launch.py')
          ),
     )
     
     action_servers = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration','action_servers.launch.py')
          )
     )

     robot_picker = Node(
               package='wheeltec_robot_rrt',
               executable='robot_picker',
               name='robot_picker',
               #prefix=['xterm -e gdb -ex run --args'],
               parameters=[{'bt_xml_filename':os.path.join(get_package_share_directory("wheeltec_robot_rrt"), 'behaviour_trees', 'robot_picker_behaviour_tree.xml')}])

     robot_pose_publisher = Node(
            package='wheeltec_robot_rrt',
            executable='robot_pose_publisher',
            name='robot_pose_publisher',
            )
            
     return LaunchDescription([
          #wheeltec_slam,     
          #nav2_include,
          #rrt_exploration,
          action_servers,
          robot_picker,
          robot_pose_publisher,
          use_sim_time_arg
     ])
