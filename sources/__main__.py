from copy import copy
from functools import partial
from math import atan2, cos, floor, inf, pi, sin, sqrt
from signal import SIGINT, default_int_handler, signal
from time import sleep, time_ns

from constants import *
from measurements import *
from pins import *
from utilities import *

from gpio import Encoder, Motor
from gpiozero import DistanceSensor, LED

# Sensors
# Actuators
# Controllers
# Models
# Strategies

# Collision detection and resolution
# Circle/point at the front
# Circle/point at the back of the robot
# Check collision for both points
# Smaller obstacles
# With collected obstacles
# Some way to filter out incorrect obstacles in array

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

        # kp = 0.0055
        # ki = 0.0030
        kp = 0.005
        ki = 0.007
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


class TentacleStrategy:
    def __init__(self, model, target_x, target_y, target_theta, time_step):
        self.model = model

        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        self.time_step = time_step

        self.obstacle_x = None
        self.obstacle_y = None

        self.alpha = 1
        self.beta = 0.2

        self.steps = 25
        self.tentacles = [
            (30, 30),  # Medium forward
            (10, 10),  # Slow forward
            (10, 40),  # Soft left
            (40, 40),  # Soft right
            (-20, 40),  # Medium left
            (50, -30),  # Medium left
        ]

    def attempt(self, left_velocity, right_velocity):
        future_model = copy(self.model)

        for _ in range(self.steps):
            future_model.update(self.time_step, left_velocity, right_velocity)

        future_x = future_model.x
        future_y = future_model.y
        future_theta = future_model.theta

        if self.obstacle_x:
            center_x = future_x + cos(future_theta) * 80
            center_y = future_y + cos(future_theta) * 80
            distance = sqrt((self.obstacle_x - center_x) ** 2 + (self.obstacle_y - center_y) ** 2)
            if distance < 80:
                return 1000000 * (1 / distance)

        e_distance = (self.target_x - future_x) ** 2 + (self.target_y - future_y) ** 2
        e_theta = self.target_theta - future_theta
        e_theta = atan2(sin(e_theta), cos(e_theta))

        cost = self.alpha * e_distance + self.beta * (e_theta ** 2)

        return cost

    def plan(self):
        costs = list(map(lambda x: self.attempt(*x), self.tentacles))

        min_i = 1
        min_cost = inf
        for i, cost in enumerate(costs):
            if cost < min_cost:
                min_i = i
                min_cost = cost

        return self.tentacles[min_i]


FREQUENCY = 10
PERIOD = 1 / FREQUENCY

TOTAL_TIME = 30
TOTAL_ITERATION_COUNT = floor(TOTAL_TIME / PERIOD)

sensor = DistanceSensor(SENSOR_ECHO_PIN, SENSOR_TRIGGER_PIN)

led = LED(LED_PIN)

left_encoder = Encoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
left_motor = Motor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_SPEED_PIN)

right_encoder = Encoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
right_motor = Motor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_SPEED_PIN)

left_wheel = WheelController(left_encoder, left_motor)
right_wheel = WheelController(right_encoder, right_motor)

model = DifferentialDriveModel()
model.theta = PI / 4

strategy = TentacleStrategy(model, 300, 300, 0, PERIOD)


obstacle_cooldown = 0

previous_time_ns = time_ns()

signal(SIGINT, default_int_handler)

try:
    for i in range(TOTAL_ITERATION_COUNT):
        sleep(PERIOD)

        current_time_ns = time_ns()
        dtime_ns = current_time_ns - previous_time_ns
        previous_time_ns = current_time_ns

        dtime = dtime_ns / (1000 ** 3)
        obstacle_cooldown -= dtime

        left_velocity = left_wheel.measure(dtime)
        right_velocity = right_wheel.measure(dtime)

        x, y, theta = model.update(dtime, left_velocity, right_velocity)
        print(f"MODEL: ({x: 8.2f}, {y: 8.2f}, {theta*RADIANS_TO_DEGREES: 7.2f}º)")

        obstacle_distance = (sensor.distance * 1000) + 80

        if obstacle_distance < 250:
            led.on()
        else:
            led.off()

        if obstacle_distance < 200 and not obstacle_cooldown > 0:
            obstacle_cooldown = 4
            strategy.obstacle_x = x + cos(theta) * obstacle_distance
            strategy.obstacle_y = y + sin(theta) * obstacle_distance
            print(f"OBSTACLE: ({strategy.obstacle_x: 8.2f}, {strategy.obstacle_y: 8.2f})")

        target_distance = (strategy.target_x - x) ** 2 + (strategy.target_y - y) ** 2
        if target_distance < 60:
            exit()

        new_left_velocity, new_right_velocity = strategy.plan()

        left_wheel.apply(dtime, new_left_velocity)
        right_wheel.apply(dtime, new_right_velocity)

except KeyboardInterrupt:
    exit()
