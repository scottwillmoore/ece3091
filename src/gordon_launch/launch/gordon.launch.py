from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


def generate_launch_description():
    gordon_launch_path = get_package_share_directory("gordon_launch")

    camera_path = join(gordon_launch_path, "launch/gordon_camera.launch.py")
    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_path))

    control_path = join(gordon_launch_path, "launch/gordon_control.launch.py")
    control = IncludeLaunchDescription(PythonLaunchDescriptionSource(control_path))

    description_path = join(gordon_launch_path, "launch/gordon_description.launch.py")
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(description_path))

    return LaunchDescription([camera, control, description])

