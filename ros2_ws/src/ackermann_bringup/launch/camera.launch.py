from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'format': 'RGB888',
                'frame_id': 'camera_link',
                'camera_info_url': 'package://ackermann_description/config/camera_calibration.yaml',
            }],
        ),
    ])
