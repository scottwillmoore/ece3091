from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package="v4l2_camera", executable="v4l2_camera_node", name="camera"
    )

    return LaunchDescription([camera])