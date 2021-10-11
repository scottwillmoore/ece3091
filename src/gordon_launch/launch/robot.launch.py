from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera",
        name="camera",
    )

    return LaunchDescription([camera])
