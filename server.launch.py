from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_decoder = Node(
        package="image_transport",
        executable="republish",
        name="image_decoder",
        arguments=["compressed"],
        remappings=[
            ("in", "image_raw"),
            ("in/compressed", "image_raw/compressed"),
            ("in/compressedDepth", "image_raw/compressedDepth"),
            ("in/theora", "image_raw/theora"),
            ("out", "image"),
            ("out/compressed", "image/compressed"),
            ("out/compressedDepth", "image/compressedDepth"),
            ("out/theora", "image/theora"),
        ],
    )

    rqt = Node(package="rqt_gui", executable="rqt_gui", name="rqt")

    rviz = Node(package="rviz2", executable="rviz2", name="rviz")

    return LaunchDescription([image_decoder, rqt, rviz])
