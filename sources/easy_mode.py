from functools import partial
from math import cos, floor, pi, sin
from signal import SIGINT, default_int_handler, signal
from time import sleep, time_ns

from constants import *
from measurements import *
from pins import *
from utilities import *

from gpio import Encoder, Motor

# Sensors
# Actuators
# Controllers
# Models
# Strategies

# TODO: Should this take `dtime` into account...
class PISystem:
    def __init__(self, proportional_gain, integral_gain, *, saturation=None):
        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain

        self.error = 0

        self.saturation = saturation

    def update(self, dtime, desired_value, value):
        derror = desired_value - value
        self.error += derror

        control = self.proportional_gain * derror + self.integral_gain * self.error

        if self.saturation:
            control = self.saturation(control)

        return control


class WheelController:
    def __init__(self, encoder, motor):
        self.encoder = encoder
        self.motor = motor
        self.motor_to_wheel_ratio = MOTOR_TO_WHEEL_RATIO
        self.wheel_radius = WHEEL_RADIUS

        self.revolutions_per_second = 0
        self.angular_velocity = 0
        self.velocity = 0
        self.distance = 0

        kp = 0.0055
        ki = 0.0030
        saturation = partial(clamp, -1, 1)

        self.system = PISystem(kp, ki, saturation=saturation)

    def measure(self, dtime):
        steps = self.encoder.count
        self.encoder.count = 0

        revolutions = self.motor_to_wheel_ratio * steps

        self.revolutions_per_second = revolutions / dtime
        self.angular_velocity = 2 * pi * self.revolutions_per_second
        self.velocity = self.wheel_radius * self.angular_velocity

        ddistance = self.velocity * dtime
        self.distance += ddistance

        return self.velocity

    def apply(self, dtime, desired_velocity):
        signed_speed = self.system.update(dtime, desired_velocity, self.velocity)

        self.motor.direction = signed_speed > 0
        self.motor.speed = abs(signed_speed)


class DifferentialDriveModel:
    def __init__(self):
        self.wheel_separation = WHEEL_SEPARATION

        self.velocity = 0
        self.angular_velocity = 0
        self.distance = 0

        self.x = 0
        self.y = 0
        self.theta = 0

    def update(self, dtime, left_velocity, right_velocity):
        self.velocity = (left_velocity + right_velocity) / 2
        self.angular_velocity = (right_velocity - left_velocity) / self.wheel_separation

        ddistance = self.velocity * dtime
        dx = self.velocity * cos(self.theta) * dtime
        dy = self.velocity * sin(self.theta) * dtime
        dtheta = self.angular_velocity * dtime

        self.distance += ddistance
        self.x += dx
        self.y += dy
        self.theta += dtheta

        return self.x, self.y, self.theta


left_encoder = Encoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
left_motor = Motor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_SPEED_PIN)

right_encoder = Encoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
right_motor = Motor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_SPEED_PIN)

left_wheel = WheelController(left_encoder, left_motor)
right_wheel = WheelController(right_encoder, right_motor)

model = DifferentialDriveModel()

FREQUENCY = 10
PERIOD = 1 / FREQUENCY

TOTAL_TIME = 2
TOTAL_ITERATION_COUNT = floor(TOTAL_TIME / PERIOD)

STATE_FORWARD = 0
STATE_TURN = 1
STATE_LEFT = 2
STATE_END = 3

print("-> STATE_FORWARD")
state = STATE_FORWARD

previous_time_ns = time_ns()

signal(SIGINT, default_int_handler)

try:
    while True:
        sleep(PERIOD)

        current_time_ns = time_ns()
        dtime_ns = current_time_ns - previous_time_ns
        previous_time_ns = current_time_ns

        dtime = dtime_ns / (1000 ** 3)

        left_velocity = left_wheel.measure(dtime)
        right_velocity = right_wheel.measure(dtime)

        x, y, theta = model.update(dtime, left_velocity, right_velocity)
        print(f"({x: 8.2f}, {y: 8.2f}, {theta*RADIANS_TO_DEGREES: 7.2f}º)")

        if state == STATE_FORWARD:
            left_wheel.apply(dtime, 50)
            right_wheel.apply(dtime, 50)

            if abs(x) > 300:
                print("-> STATE_TURN")
                state = STATE_TURN
                sleep(1)

        elif state == STATE_TURN:
            left_wheel.apply(dtime, -30)
            right_wheel.apply(dtime, 30)

            if abs(theta) > (PI / 2) - (PI / 90):
                print("-> STATE_LEFT")
                state = STATE_LEFT
                sleep(1)

        elif state == STATE_LEFT:
            left_wheel.apply(dtime, 50)
            right_wheel.apply(dtime, 50)

            if abs(y) > 300:
                print("-> STATE_END")
                state = STATE_END
                sleep(1)

        elif state == STATE_END:
            break

except KeyboardInterrupt:
    exit()
