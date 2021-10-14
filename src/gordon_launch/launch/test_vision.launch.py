from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # configuration_path = "/home/scott/yolov3/yolov3.cfg"
    # model_path = "/home/scott/yolov3/yolov3.weights"

    configuration_path = "/home/scott/yolov4_tiny/yolov4-tiny-obj.cfg"
    model_path = "/home/scott/yolov4_tiny/yolov4_tiny_new__training_final.weights"

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

    test_vision = Node(
        package="gordon_vision",
        executable="test_vision",
        name="test_vision",
        parameters=[{"configuration": configuration_path, "model": model_path}],
        remappings=[("image", "camera/image_rectified")],
    )

    return LaunchDescription([image_decoder, image_processor, test_vision])

