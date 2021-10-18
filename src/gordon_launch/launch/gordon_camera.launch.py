from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[{"camera_frame_id": "camera_link"}],
    )

    return LaunchDescription([camera])

