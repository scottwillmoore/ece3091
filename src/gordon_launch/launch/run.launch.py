from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")

    localization_config_path = join(gordon_launch_path, "config", "ekf.yaml")

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[localization_config_path],
    )

    """
    gordon_description_path = find_package.find("gordon_description")

    robot_description_path = join(
        gordon_description_path, "urdf", "gordon_description.urdf"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", robot_description_path])}
        ],
    )
    """

    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    join(
                        get_package_share_directory("gordon_launch"),
                        "config",
                        "ekf.yaml",
                    )
                ],
            )
        ]
    )

