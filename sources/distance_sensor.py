from time import sleep
from math import cos, sin

import gpio
from gpiozero import DistanceSensor, LED

from pins import *

led = LED(LED_PIN)

sensor = DistanceSensor(SENSOR_ECHO_PIN, SENSOR_TRIGGER_PIN)


class Obstacle:
    def __init__(self, x, y, *, life=None):
        self.x = x
        self.y = y
        self.life = life
        pass

    def __repr__(self):
        return f"({self.x}, {self.y})"


# Sensor with metadata (relative position to robot)
# - Detect if obstacle
# - Get obstacle position relative to robot
# ObstacleStorage
# - Get obstacle
# - Set obstacle


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
                    pass

    def get(self, x, y):
        for obstacle in self.obstacles:
            distance = (x - obstacle.x) ** 2 + (y - obstacle.y) ** 2
            if distance < self.threshold:
                return obstacle

        return None

    def set(self, x, y):
        if self.get(x, y):
            return None
        else:
            obstacle = Obstacle(x, y)
            self.obstacles.append(obstacle)
            return obstacle


# grid = ObjectGrid()
grid = ObjectList()

x = 0
y = 0
theta = 0
robot_distance = 160

while True:

    if sensor.distance < 0.3:
        led.on()

        sensor_distance = sensor.distance * 1000
        distance = robot_distance + sensor_distance
        object_x = x + cos(theta) * distance
        object_y = y + sin(theta) * distance

        grid.set(object_x, object_y)
    else:
        led.off()

    print(f"{sensor.distance}")
    print(grid.obstacles)
    sleep(1)
