from copy import copy
from functools import partial
from math import atan2, cos, floor, inf, pi, sin, sqrt
from signal import SIGINT, default_int_handler, signal
from time import sleep, time_ns

from constants import *
from measurements import *
from pins import *
from utilities import *

import numpy as np
from gpio import Encoder, Motor
from gpiozero import DistanceSensor, LED
# from distance_sensor import obstacleScanner

led = LED(LED_PIN)


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


class Obstacle:
    def __init__(self, x, y, *, life=None):
        self.x = x
        self.y = y
        self.life = life
        pass

    def returnFunc(x,y):
        return [x,y]

    def __repr__(self):
        return f"({self.x}, {self.y})"

class ObjectList:
    def __init__(self, *, threshold=10):
        self.threshold = threshold ** 2

        self.obstacles = []

    def update(self, dtime):
        for obstacle in self.obstacles:
            if obstacle.life:
                obstacle.life -= dtime
                if obstacle.life < 0:
                    # remove it!
                    # how to safely remove list item in a loop...
                    #pop(0)
                    pass

    def get(self, x, y):
        for obstacle in self.obstacles:
            distance = (x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2
            if distance < self.threshold:
                return obstacle

        return None

    def set(self, x, y):
        if self.get(x, y):
            return None
        else:
            obstacle = Obstacle.returnFunc(x, y)
            self.obstacles.append(obstacle)
            return obstacle
sensor = DistanceSensor(SENSOR_ECHO_PIN, SENSOR_TRIGGER_PIN)
sensorL = DistanceSensor(SENSOR_LEFT_ECHO_PIN,SENSOR_LEFT_TRIGGER_PIN)
sensorR = DistanceSensor(SENSOR_RIGHT_ECHO_PIN,SENSOR_RIGHT_TRIGGER_PIN)
#sensorB = DistanceSensor(SENSOR_BACK_ECHO_PIN, SENSOR_BACK_TRIGGER_PIN)
# robot_distance = 160

class TentacleStrategy:
    def __init__(self, model, target_x, target_y, target_theta, time_step):
        self.model = model

        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        self.time_step = time_step

        # self.obstacles = obstacles

        self.grid = ObjectList()

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
            (0, 0), # stationary
        ]

    def attempt(self, left_velocity, right_velocity):
        future_model = copy(self.model)

        for _ in range(self.steps):
            future_model.update(self.time_step, left_velocity, right_velocity)

        future_x = future_model.x
        future_y = future_model.y
        future_theta = future_model.theta

        """if self.obstacle_x:
            center_x = future_x + cos(future_theta) * 80
            center_y = future_y + cos(future_theta) * 80
            distance = sqrt((self.obstacle_x - center_x) ** 2 + (self.obstacle_y - center_y) ** 2)
            if distance < 80:
                return 1000000 * (1 / distance)"""

        # if there is an obstacle make the cost really high to avoid using this path
        # potentially need to use a different x and y but its too confusing
        if (self.check_collision(future_x,future_y)):
            return 10000000

        e_distance = (self.target_x - future_x) ** 2 + (self.target_y - future_y) ** 2
        e_theta = self.target_theta - future_theta
        e_theta = atan2(sin(e_theta), cos(e_theta))

        cost = self.alpha * e_distance + self.beta * (e_theta ** 2)

        return cost

    def grid_update(self):
        x = self.model.x
        y = self.model.y
        theta = self.model.theta

        robot_distanceL = 60
        robot_distanceR = 60
        robot_distanceF = 75
        if sensor.distance < 0.3:
            #led.on()
            
            sensor_distance = sensor.distance * 1000
            distance = robot_distanceF + sensor_distance
            object_x = x + cos(theta) * distance
            object_y = y + sin(theta) * distance

            self.grid.set(object_x, object_y)
            sleep(0.01)

        if sensorL.distance < 0.3:
            #led.on()
            
            sensor_distanceL = sensorL.distance * 1000
            distance = robot_distanceL + sensor_distanceL
            object_x = x - sin(theta) * distance
            object_y = y + cos(theta) * distance
            
            self.grid.set(object_x,object_y)
            sleep(0.01)
        
        if sensorR.distance < 0.3:
            #led.on()
            
            sensor_distanceR = sensorR.distance * 1000
            distance = robot_distanceR + sensor_distanceR
            object_x = x + sin(theta) * distance
            object_y = y - cos(theta) * distance
            
            self.grid.set(object_x,object_y)
            sleep(0.01)
        
            """if sensorB.distance < 0.3:
            #led.on()
            
            sensor_distanceB = sensorB.distance *1000
            distance = robot_distanceB + sensor_distanceB
            object_x = x - cos(theta) * distance
            object_y = y - sin(theta) * distance
            
            grid.set(object_x,object_y)
            sleep(0.01)"""

        #else:
            #led.off()

        list = np.array(self.grid.obstacles)
        self.obstacles = list
        # sleep(1)

        return list
    

    # this function reads the obstacle inputs which are provided as a np array and then determines the minimum distance to an object
    # if the object is less than 40000, return that there is an obstacle in the way
    def check_collision(self,x,y):
        # if statement checks to see if there are any obstacles
        # return false if no obstacles or obstacles far away
        if self.obstacles.size > 0:
            min_dist = np.min(np.sqrt((x-self.obstacles[:,0])**2 + (y-self.obstacles[:,1])**2))
            # allow for 150mm from reference point to nose, and then 50 mm clearance (200^2)
            # reference allowance can be tweaked as we go
            if (min_dist < 40000):
                return True
        return False

    def plan(self):
        costs = list(map(lambda x: self.attempt(*x), self.tentacles))

        min_i = 1
        min_cost = inf
        for i, cost in enumerate(costs):
            if cost < min_cost:
                min_i = i
                min_cost = cost

        return self.tentacles[min_i]


FREQUENCY = 1
PERIOD = 1 / FREQUENCY

TOTAL_TIME = 30
TOTAL_ITERATION_COUNT = floor(TOTAL_TIME / PERIOD)

#sensor = DistanceSensor(SENSOR_ECHO_PIN, SENSOR_TRIGGER_PIN)


left_encoder = Encoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
left_motor = Motor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_SPEED_PIN)

right_encoder = Encoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
right_motor = Motor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_SPEED_PIN)

left_wheel = WheelController(left_encoder, left_motor)
right_wheel = WheelController(right_encoder, right_motor)

model = DifferentialDriveModel()
# model.theta = PI / 4

# obstacles = obstacleScanner()

strategy = TentacleStrategy(model, 300, 300, 90, PERIOD)

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

        obstacles = strategy.grid_update()
        
        print(obstacles)
        
        x, y, theta = model.update(dtime, left_velocity, right_velocity)
        #print(f"MODEL: ({x: 8.2f}, {y: 8.2f}, {theta*RADIANS_TO_DEGREES: 7.2f}º)")

        """obstacle_distance = (sensor.distance * 1000) + 80

        if obstacle_distance < 250:
            led.on()
        else:
            led.off()

        if obstacle_distance < 200 and not obstacle_cooldown > 0:
            obstacle_cooldown = 4
            strategy.obstacle_x = x + cos(theta) * obstacle_distance
            strategy.obstacle_y = y + sin(theta) * obstacle_distance
            print(f"OBSTACLE: ({strategy.obstacle_x: 8.2f}, {strategy.obstacle_y: 8.2f})")"""

        target_distance = (strategy.target_x - x) ** 2 + (strategy.target_y - y) ** 2
        if target_distance < 60:
            exit()

        new_left_velocity, new_right_velocity = strategy.plan()

        # left_wheel.apply(dtime, new_left_velocity)
        # right_wheel.apply(dtime, new_right_velocity)

except KeyboardInterrupt:
    exit()
