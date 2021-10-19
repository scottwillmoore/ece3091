from launch import LaunchDescription
from launch_ros.actions import Node

<<<<<<< HEAD
FRONT_SENSOR_ECHO_PIN = 8
FRONT_SENSOR_TRIGGER_PIN = 7

LEFT_SENSOR_ECHO_PIN = 23
LEFT_SENSOR_TRIGGER_PIN = 24

RIGHT_SENSOR_ECHO_PIN = 17
RIGHT_SENSOR_TRIGGER_PIN = 27
=======
FRONT_SENSOR_ECHO_PIN = 24
FRONT_SENSOR_TRIGGER_PIN = 23

LEFT_SENSOR_ECHO_PIN = 17
LEFT_SENSOR_TRIGGER_PIN = 27

"""RIGHT_SENSOR_ECHO_PIN = 0
RIGHT_SENSOR_TRIGGER_PIN = 0"""
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b

def generate_launch_description():
    
    front_sensor_controller = Node(
<<<<<<< HEAD
        package="gordon_ultrasonic_control",
        executable="ultrasonic_controller",
        name="front_sensor_controller",
        remappings=[()],
=======
        package="gordon_ultrasonic",
        executable="ultrasonic",
        name="front_sensor_controller",
        remappings=[(range)],
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b
        parameters=[ {
                "ECHO_PIN": FRONT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": FRONT_SENSOR_TRIGGER_PIN},],
    )

    left_sensor_controller = Node(
<<<<<<< HEAD
        package="gordon_ultrasonic_control",
        executable="ultrasonic_controller",
        name="left_sensor_controller",
        remappings=[()],
=======
        package="gordon_ultrasonic",
        executable="ultrasonic",
        name="left_sensor_controller",
        remappings=[(range)],
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b
        parameters=[{
                "ECHO_PIN": LEFT_SENSOR_ECHO_PIN,
                "TRIGGER_PIN": LEFT_SENSOR_TRIGGER_PIN},],
    )

<<<<<<< HEAD
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
=======
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
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b
