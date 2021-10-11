from os.path import join


from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share_path = FindPackageShare("gordon_description").find(
        "gordon_description"
    )

    robot_description_path = join(
        package_share_path, "urdf/gordon_description.urdf"
    )

    rviz_configuration_path = join(
        package_share_path, "rviz/gordon_description.rviz"
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", robot_description_path])}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_configuration_path],
    )

    return LaunchDescription(
        [joint_state_publisher, robot_state_publisher, rviz]
    )

