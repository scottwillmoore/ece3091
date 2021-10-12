from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")
    gordon_description_path = get_package_share_directory("gordon_description")

    robot_description_path = join(gordon_description_path, "urdf/gordon_description.urdf")

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[{"camera_frame_id": "camera_link"}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", robot_description_path])}],
    )

    return LaunchDescription([camera, joint_state_publisher, robot_state_publisher])
