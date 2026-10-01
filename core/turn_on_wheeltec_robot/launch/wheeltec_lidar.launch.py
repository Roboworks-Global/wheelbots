import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # This rosbot2 ships with a Slamtec RPLIDAR A1M8 (model 0x18, HW7). An
    # earlier hookup targeted the Leishen M10 by mistake — same CP2102 USB
    # bridge and same Wheeltec udev symlink (/dev/wheeltec_laser), but a
    # different vendor, protocol (Slamtec RP at 115200 baud), and driver.
    sllidar_launch_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'), 'launch'
    )

    rplidar_a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_launch_dir, 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/wheeltec_laser',
            'frame_id': 'laser_link',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(rplidar_a1)
    return ld
