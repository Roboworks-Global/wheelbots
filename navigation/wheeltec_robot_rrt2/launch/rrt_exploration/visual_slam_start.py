import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    use_sim_time_arg = DeclareLaunchArgument(
               'use_sim_time',
               default_value=use_sim_time,
               description='If true, use simulated clock')

    republish_left = Node(
       package='image_transport',
       executable='republish',
       name='republish_left',
       output='screen',
       arguments=[
                "compressed",
                "in:=/stereo_camera/left/image_raw_throttle",
                "raw",
                "out:=/stereo_camera/left/image_raw_throttle_relay",
            ],
       )

    republish_right = Node(
       package='image_transport',
       executable='republish',
       name='republish_right',
       output='screen',
       arguments=[
                "compressed",
                "in:=/stereo_camera/right/image_raw_throttle",
                "raw",
                "out:=/stereo_camera/right/image_raw_throttle_relay",
            ],
       )

    left_image_proc = Node(
       package='image_proc',
       executable='image_proc',
       name='left_image_proc',
       output='screen',
       remappings=[
                ("image", "stereo_camera/left/image_raw"),
                ("camera_info", "stereo_camera/left/camera_info"),
                ("image_mono", "stereo_camera/left/image_mono"),
                ("image_rect", "stereo_camera/left/image_rect"),
                ("image_color", "stereo_camera/left/image_color"),
                ("image_rect_color", "stereo_camera/left/image_rect_color"),
            ]
       )

    right_image_proc = Node(
       package='image_proc',
       executable='image_proc',
       name='right_image_proc',
       output='screen',
       remappings=[
                ("image", "stereo_camera/right/image_raw"),
                ("camera_info", "stereo_camera/right/camera_info"),
                ("image_mono", "stereo_camera/right/image_mono"),
                ("image_rect", "stereo_camera/right/image_rect"),
                ("image_color", "stereo_camera/right/image_color"),
                ("image_rect_color", "stereo_camera/right/image_rect_color"),
            ]
       )

    disparity_node = Node(
       package='stereo_image_proc',
       executable='disparity_node',
       name='disparity_node',
       output='screen',
       remappings=[
                ("/left/camera_info", "stereo_camera/left/camera_info"),
                ("/left/image_rect", "stereo_camera/left/image_rect"),
                ("/right/camera_info", "stereo_camera/right/camera_info"),
                ("/right/image_rect", "stereo_camera/right/image_rect"),
            ]
       )

    point_cloud_node = Node(
       package='stereo_image_proc',
       executable='point_cloud_node',
       name='point_cloud_node',
       output='screen',
       remappings=[
                ("/left/camera_info", "stereo_camera/left/camera_info"),
                ("/left/image_rect_color", "stereo_camera/left/image_rect"),
                ("/right/camera_info", "stereo_camera/right/camera_info"),
            ]
       )
    
    rtabmap = Node(
       package='rtabmap_ros',
       executable='rtabmap',
       name='rtabmap',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('wheeltec_robot_rrt'), 'config', 'rtabmap.yaml'), {"publish_tf": False, 'use_sim_time':use_sim_time}],
       arguments=['--delete_db_on_start'],
       remappings=[
            ("left/image_rect", "/stereo_camera/left/image_rect"),
            ("right/image_rect", "/stereo_camera/right/image_rect"),
            ("left/camera_info", "/stereo_camera/left/camera_info"),
            ("right/camera_info", "/stereo_camera/right/camera_info"),
            ("map", "/rtabmap_ros/map"),
        ])
   
    robot_state_publisher_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('wheeltec_robot_rrt'), 'launch', 'robot_state_publisher.py')
          )
     )
    
    velocity_smoother = Node(
          package="wheeltec_robot_rrt",
          executable="velocity_smoother",
          output="both",
     )

    return LaunchDescription([
        # republish_left,
        # republish_right,
        left_image_proc,
        right_image_proc,
        disparity_node,
        point_cloud_node,
        rtabmap,
        robot_state_publisher_include,
        velocity_smoother,
        use_sim_time_arg
        ])

