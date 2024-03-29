from launch import LaunchDescription
from launch_ros.actions import Node

FRONT_SENSOR_ECHO_PIN = 24
FRONT_SENSOR_TRIGGER_PIN = 23

LEFT_SENSOR_ECHO_PIN = 17
LEFT_SENSOR_TRIGGER_PIN = 27

"""RIGHT_SENSOR_ECHO_PIN = 0
RIGHT_SENSOR_TRIGGER_PIN = 0"""

def generate_launch_description():
    
    front_sensor_controller = Node(
        package="gordon_control",
        executable="ultrasonic_controller",
        name="front_sensor_controller",
        remappings=[(range)],
        parameters=[ {
                "echo_pin": FRONT_SENSOR_ECHO_PIN,
                "trigger_pin": FRONT_SENSOR_TRIGGER_PIN},],
    )

    left_sensor_controller = Node(
        package="gordon_control",
        executable="ultrasonic_controller",
        name="left_sensor_controller",
        remappings=[(range)],
        parameters=[{
                "echo_pin": LEFT_SENSOR_ECHO_PIN,
                "trigger_pin": LEFT_SENSOR_TRIGGER_PIN},],
    )

    """right_sensor_controller = Node(
        package="gordon_ultrasonic",
        executable="ultrasonic_controller",
        name="right_sensor_controller",
        remappings=[(range)],
        parameters=[{
                "ECHO_PIN": RIGHT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": RIGHT_SENSOR_TRIGGER_PIN},],
    )"""


    return LaunchDescription([front_sensor_controller,left_sensor_controller])
