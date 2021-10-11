from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")

    camera = Node(
        package="v4l2_camera", executable="v4l2_camera_node", name="camera"
    )

    left_rotary_encoder = Node(
        package="gordon_hardware",
        executable="rotary_encoder",
        name="left_rotary_encoder",
        parameters=[{"a_pin": 20, "b_pin": 21}],
        remappings=[("count", "left_count")],
    )

    right_rotary_encoder = Node(
        package="gordon_hardware",
        executable="rotary_encoder",
        name="right_rotary_encoder",
        parameters=[{"a_pin": 19, "b_pin": 26}],
        remappings=[("count", "right_count")],
    )

    return LaunchDescription(
        [camera, left_rotary_encoder, right_rotary_encoder]
    )
