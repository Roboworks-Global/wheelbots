import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    # The rosbot2's classic Astra Pro splits into an Orbbec proprietary depth
    # endpoint (2bc5:060f) and a Sonix UVC RGB endpoint (2bc5:050f). OrbbecSDK v2
    # dropped support for the classic Astra Pro, so depth is unavailable via the
    # official driver on this platform. We publish the UVC RGB stream via
    # v4l2_camera; if depth is needed later, switch to OrbbecSDK v1 or a Gemini/
    # Astra 2 sensor.
    v4l2 = launch_ros.actions.Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            # YUYV is uncompressed and natively supported by v4l2_camera's
            # rgb8 conversion path. MJPG would need mjpeg-tools and Kilted's
            # v4l2_camera doesn't decode it inline (throws on unknown encoding).
            'pixel_format': 'YUYV',
            'image_size': [640, 480],
            'output_encoding': 'rgb8',
            'camera_frame_id': 'camera_link',
        }],
    )

    ld = LaunchDescription()
    ld.add_action(v4l2)
    return ld
