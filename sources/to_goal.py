from math import cos, floor, sin
from signal import SIGINT, default_int_handler, signal
from time import sleep, time_ns

from gpio import Encoder, Motor

from constants import *
from measurements import *
from pins import *


class Wheel:
    def __init__(self, encoder, motor):
        self.encoder = encoder
        self.motor = motor

        self.desired_velocity = 0
        # self.kp = 0.0055
        # self.ki = 0.0030
        self.kp = 0.005
        self.ki = 0.007
        self.error = 0

        self.revolutions_per_second = 0
        self.angular_velocity = 0
        self.velocity = 0
        self.distance = 0

    def update(self, dtime):
        steps = self.encoder.count
        self.encoder.count = 0

        revolutions = MOTOR_TO_WHEEL_RATIO * steps

        self.revolutions_per_second = revolutions / dtime
        self.angular_velocity = 2 * PI * self.revolutions_per_second
        self.velocity = WHEEL_RADIUS * self.angular_velocity

        ddistance = self.velocity * dtime
        self.distance += ddistance

        derror = self.desired_velocity - self.velocity
        self.error += derror

        u = self.kp * derror + self.ki * self.error
        u_clamped = min(1, max(-1, u))

        self.motor.speed = abs(u_clamped)
        self.motor.direction = u_clamped > 0


class Robot:
    def __init__(self, left_wheel, right_wheel):
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel

        self.velocity = 0
        self.angular_velocity = 0
        self.distance = 0

        self.x = 0
        self.y = 0
        self.theta = 0

    def update(self, dtime):
        self.left_wheel.update(dtime)
        self.right_wheel.update(dtime)

        velocity_left = self.left_wheel.velocity
        velocity_right = self.right_wheel.velocity

        self.velocity = (velocity_left + velocity_right) / 2
        self.angular_velocity = (velocity_right - velocity_left) / WHEEL_SEPARATION

        ddistance = self.velocity * dtime
        dx = self.velocity * cos(self.theta) * dtime
        dy = self.velocity * sin(self.theta) * dtime
        dtheta = self.angular_velocity * dtime

        self.distance += ddistance
        self.x += dx
        self.y += dy
        self.theta += dtheta


left_encoder = Encoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
left_motor = Motor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_SPEED_PIN)
left_wheel = Wheel(left_encoder, left_motor)

right_encoder = Encoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
right_motor = Motor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_SPEED_PIN)
right_wheel = Wheel(right_encoder, right_motor)

robot = Robot(left_wheel, right_wheel)

TIME_STEP = 1 / 30

STATE_FORWARD = 0
STATE_TURN = 1
STATE_LEFT = 2
STATE_END = 3

state = STATE_FORWARD

previous_time_ns = time_ns()

total_average_error = 0
iteration_count = 0

signal(SIGINT, default_int_handler)

try:
    while True:
        sleep(TIME_STEP)

        current_time_ns = time_ns()
        dtime_ns = current_time_ns - previous_time_ns
        previous_time_ns = current_time_ns

        dtime = dtime_ns / (1000 ** 3)

        robot.update(dtime)

        total_average_error += (
            abs(left_wheel.desired_velocity - left_wheel.velocity)
            + abs(right_wheel.desired_velocity - right_wheel.velocity)
        ) / 2
        iteration_count += 1

        if state == STATE_FORWARD:
            left_wheel.desired_velocity = 40
            right_wheel.desired_velocity = 40

            if abs(robot.x) > 300:
                print(f"x: {robot.x}")
                print(f"y: {robot.y}")
                print(f"theta: {robot.theta * RADIANS_TO_DEGREES}")
                state = STATE_TURN

        elif state == STATE_TURN:
            left_wheel.desired_velocity = -10
            right_wheel.desired_velocity = 10

            if abs(robot.theta) > (PI / 2) - (PI / 90):
                print(f"x: {robot.x}")
                print(f"y: {robot.y}")
                print(f"theta: {robot.theta * RADIANS_TO_DEGREES}")
                state = STATE_LEFT

        elif state == STATE_LEFT:
            left_wheel.desired_velocity = 40
            right_wheel.desired_velocity = 40

            if abs(robot.y) > 300:
                print(f"x: {robot.x}")
                print(f"y: {robot.y}")
                print(f"theta: {robot.theta * RADIANS_TO_DEGREES}")
                state = STATE_END

        elif state == STATE_END:
            break

except KeyboardInterrupt:
    exit()

print(total_average_error / iteration_count)
