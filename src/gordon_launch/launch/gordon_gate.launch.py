from launch import LaunchDescription
from launch_ros.actions import Node


LEFT_SERVO_PIN = 25
RIGHT_SERVO_PIN = 18

def generate_launch_description():
    gate_controller = Node(
            package="gordon_control",
            executable="gate_controller",
            name="gate_controller",
            remappings=[()],
            parameters=[{
                    "left_servo_pin": LEFT_SERVO_PIN,
                    "right_servo_pin": RIGHT_SERVO_PIN,
                },
            ],
        )
    return LaunchDescription([gate_controller])
