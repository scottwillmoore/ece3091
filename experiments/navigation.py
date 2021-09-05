from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep
from math import pi, cos, sin
 
# LEFT_MOTOR_PHASE_PIN = 6
# LEFT_MOTOR_ENABLE_PIN = 13
 
# RIGHT_MOTOR_PHASE_PIN = 26
# RIGHT_MOTOR_ENABLE_PIN = 19
 
# LEFT_ENCODER_A_PIN = 23
# LEFT_ENCODER_B_PIN = 24

# RIGHT_ENCODER_A_PIN = 25
# RIGHT_ENCODER_B_PIN = 12

# Swapped as left and right are swapped when the robot is flipped upside down

RIGHT_MOTOR_PHASE_PIN = 6
RIGHT_MOTOR_ENABLE_PIN = 13

LEFT_MOTOR_PHASE_PIN = 26
LEFT_MOTOR_ENABLE_PIN = 19
 
RIGHT_ENCODER_A_PIN = 23
RIGHT_ENCODER_B_PIN = 24

LEFT_ENCODER_A_PIN = 25
LEFT_ENCODER_B_PIN = 12


# All measurements are in millimetres!
WHEEL_RADIUS = 30
WHEEL_SEPARATION = 92

ENCODER_RESOLUTION = 32
GEAR_RATIO = 344.2
WHEEL_TO_MOTOR_RATIO = ENCODER_RESOLUTION * GEAR_RATIO
MOTOR_TO_WHEEL_RATIO = 1 / WHEEL_TO_MOTOR_RATIO

TIME_STEP = 1 / 30

TOTAL_TIME = 5
TOTAL_ITERATIONS = int(TOTAL_TIME / TIME_STEP)

SPEED_LEFT = 0.4
SPEED_RIGHT = 0.3

RADIANS_TO_DEGREES = 360 / (2 * pi)
 
left_motor = PhaseEnableMotor(phase=LEFT_MOTOR_PHASE_PIN, enable=LEFT_MOTOR_ENABLE_PIN)
right_motor = PhaseEnableMotor(phase=RIGHT_MOTOR_PHASE_PIN, enable=RIGHT_MOTOR_ENABLE_PIN)
 
left_encoder = RotaryEncoder(a=LEFT_ENCODER_A_PIN, b=LEFT_ENCODER_B_PIN, max_steps = 0)
right_encoder = RotaryEncoder(a=RIGHT_ENCODER_A_PIN, b=RIGHT_ENCODER_B_PIN,max_steps = 0)

total_turns_left = 0
average_turns_per_second_left = 0
average_velocity_left = 0
total_distance_left = 0

total_turns_right = 0
average_turns_per_second_right = 0
average_velocity_right = 0
total_distance_right = 0

average_velocity = 0
total_distance = 0
average_angular_velocity = 0

x = 0
y = 0
theta = 0

left_motor.forward(SPEED_LEFT)
right_motor.forward(SPEED_RIGHT)

for i in range(TOTAL_ITERATIONS):
    steps_left = -left_encoder.steps
    left_encoder.steps = 0

    steps_right = right_encoder.steps
    right_encoder.steps = 0
    
    turns_left = steps_left * MOTOR_TO_WHEEL_RATIO
    turns_right = steps_right * MOTOR_TO_WHEEL_RATIO

    total_turns_left += turns_left
    total_turns_right += turns_right
    
    turns_per_second_left = turns_left / TIME_STEP
    turns_per_second_right = turns_right / TIME_STEP

    average_turns_per_second_left += turns_per_second_left
    average_turns_per_second_right += turns_per_second_right
    
    angular_velocity_left = 2 * pi * turns_per_second_left
    angular_velocity_right = 2 * pi * turns_per_second_right
    
    velocity_left = WHEEL_RADIUS * angular_velocity_left
    velocity_right = WHEEL_RADIUS * angular_velocity_right

    average_velocity_left += velocity_left
    average_velocity_right += velocity_right
    
    distance_left = TIME_STEP * velocity_left
    distance_right = TIME_STEP * velocity_right

    total_distance_left += distance_left
    total_distance_right += distance_right
    
    velocity = (velocity_left + velocity_right) / 2
    average_velocity += velocity

    distance = TIME_STEP * velocity
    total_distance += distance
    
    angular_velocity = (velocity_right - velocity_left) / WHEEL_SEPARATION
    average_angular_velocity += angular_velocity
    
    theta += TIME_STEP * angular_velocity 
    
    x += TIME_STEP * velocity * cos(theta)
    y += TIME_STEP * velocity * sin(theta)
    
    sleep(TIME_STEP)

average_turns_per_second_left /= TOTAL_ITERATIONS
average_velocity_left /= TOTAL_ITERATIONS

average_turns_per_second_right /= TOTAL_ITERATIONS
average_velocity_right /= TOTAL_ITERATIONS

average_velocity /= TOTAL_ITERATIONS
average_angular_velocity /= TOTAL_ITERATIONS

print(f"total_turns_left: {total_turns_left}")
print(f"average_turns_per_second_left: {average_turns_per_second_left}")
print(f"average_velocity_left: {average_velocity_left}")
print(f"total_distance_left: {total_distance_left}")

print(f"total_turns_right: {total_turns_right}")
print(f"average_turns_per_second_right: {average_turns_per_second_right}")
print(f"average_velocity_right: {average_velocity_right}")
print(f"total_distance_right: {total_distance_right}")

print(f"average_velocity: {average_velocity}")
print(f"average_angular_velocity: {average_angular_velocity}")
print(f"total_distance: {total_distance}")

print(f"x: {x}")
print(f"y: {y}")
print(f"theta: {theta}")
print(f"theta (degrees): {theta * RADIANS_TO_DEGREES}")

left_motor.stop()
right_motor.stop()