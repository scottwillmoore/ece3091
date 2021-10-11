from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")

    image_decoder = Node(
        package="image_transport",
        executable="republish",
        namespace="camera",
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

    image_processor = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        namespace="camera",
        name="image_processor",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                namespace="camera",
                name="image_rectifier",
                remappings=[
                    ("image_rect", "image_rectified"),
                    ("image_rect/compressed", "image_rectified/compressed"),
                    ("image_rect/compressedDepth", "image_rectified/compressedDepth"),
                    ("image_rect/theora", "image_rectified/theora"),
                ],
            )
        ],
    )

    rqt = Node(package="rqt_gui", executable="rqt_gui", name="rqt")

    rviz = Node(package="rviz2", executable="rviz2", name="rviz")

    return LaunchDescription([image_decoder, image_processor, rqt, rviz])

