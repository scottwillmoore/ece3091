from launch import LaunchDescription
from launch_ros.actions import Node

FRONT_SENSOR_ECHO_PIN = 8
FRONT_SENSOR_TRIGGER_PIN = 7

LEFT_SENSOR_ECHO_PIN = 23
LEFT_SENSOR_TRIGGER_PIN = 24

RIGHT_SENSOR_ECHO_PIN = 17
RIGHT_SENSOR_TRIGGER_PIN = 27

def generate_launch_description():
    
    front_sensor_controller = Node(
        package="gordon_ultrasonic_control",
        executable="ultrasonic_controller",
        name="front_sensor_controller",
        remappings=[()],
        parameters=[ {
                "ECHO_PIN": FRONT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": FRONT_SENSOR_TRIGGER_PIN},],
    )

    left_sensor_controller = Node(
        package="gordon_ultrasonic_control",
        executable="ultrasonic_controller",
        name="left_sensor_controller",
        remappings=[()],
        parameters=[{
                "ECHO_PIN": LEFT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": LEFT_SENSOR_TRIGGER_PIN},],
    )

    right_sensor_controller = Node(
        package="gordon_ultrasonic_control",
        executable="ultrasonic_controller",
        name="right_sensor_controller",
        remappings=[()],
        parameters=[{
                "ECHO_PIN": RIGHT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": RIGHT_SENSOR_TRIGGER_PIN},],
    )


    return LaunchDescription()