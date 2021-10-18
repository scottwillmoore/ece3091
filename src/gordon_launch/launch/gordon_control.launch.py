from launch import LaunchDescription
from launch_ros.actions import Node


LEFT_MOTOR_DIRECTION_PIN = 16
LEFT_MOTOR_SPEED_PIN = 12

LEFT_ROTARY_ENCODER_A_PIN = 20
LEFT_ROTARY_ENCODER_B_PIN = 21

RIGHT_MOTOR_DIRECTION_PIN = 6
RIGHT_MOTOR_SPEED_PIN = 13

RIGHT_ROTARY_ENCODER_A_PIN = 19
RIGHT_ROTARY_ENCODER_B_PIN = 26

UPDATE_RATE = 20
TIMEOUT_DURATION = 1.0

WHEEL_RADIUS = 0.027
WHEEL_SEPARATION = 0.092

WHEEL_K_P = 15.0
WHEEL_K_I = 60.0
WHEEL_K_D = 0.0

WHEEL_K_T = 1.0

WHEEL_MAX_SPEED = 0.07

ENCODER_RESOLUTION = 32
GEAR_RATIO = 344.2

WHEEL_TO_MOTOR_RATIO = ENCODER_RESOLUTION * GEAR_RATIO
MOTOR_TO_WHEEL_RATIO = WHEEL_TO_MOTOR_RATIO ** -1


def generate_launch_description():
    wheel_parameters = {
        "update_rate": UPDATE_RATE,
        "timeout_duration": TIMEOUT_DURATION,
        "k_p": WHEEL_K_P,
        "k_i": WHEEL_K_I,
        "k_d": WHEEL_K_D,
        "k_t": WHEEL_K_T,
        "motor_to_wheel_ratio": MOTOR_TO_WHEEL_RATIO,
        "wheel_radius": WHEEL_RADIUS,
        "max_speed": WHEEL_MAX_SPEED,
    }

    left_wheel_controller = Node(
        package="gordon_control",
        executable="wheel_controller",
        name="left_wheel_controller",
        remappings=[("velocity", "left_velocity"), ("desired_velocity", "left_desired_velocity")],
        parameters=[
            wheel_parameters,
            {
                "a_pin": LEFT_ROTARY_ENCODER_A_PIN,
                "b_pin": LEFT_ROTARY_ENCODER_B_PIN,
                "direction_pin": LEFT_MOTOR_DIRECTION_PIN,
                "speed_pin": LEFT_MOTOR_SPEED_PIN,
            },
        ],
    )

    right_wheel_controller = Node(
        package="gordon_control",
        executable="wheel_controller",
        name="right_wheel_controller",
        remappings=[("velocity", "right_velocity"), ("desired_velocity", "right_desired_velocity")],
        parameters=[
            wheel_parameters,
            {
                "a_pin": RIGHT_ROTARY_ENCODER_A_PIN,
                "b_pin": RIGHT_ROTARY_ENCODER_B_PIN,
                "direction_pin": RIGHT_MOTOR_DIRECTION_PIN,
                "speed_pin": RIGHT_MOTOR_SPEED_PIN,
            },
        ],
    )

    drive_controller = Node(
        package="gordon_hardware",
        executable="drive_controller",
        name="drive_controller",
        parameters=[
            {
                "update_rate": UPDATE_RATE,
                "timeout_duration": TIMEOUT_DURATION,
                "wheel_separation": WHEEL_SEPARATION,
            }
        ],
    )

    return LaunchDescription([left_wheel_controller, right_wheel_controller])

