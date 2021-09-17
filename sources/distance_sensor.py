from time import sleep
from math import cos, sin, atan, pi, sqrt

import gpio
import numpy as np
from gpiozero import DistanceSensor, LED

from pins import *

#led = LED(LED_PIN)

sensor = DistanceSensor(SENSOR_ECHO_PIN, SENSOR_TRIGGER_PIN)
sensorL = DistanceSensor(SENSOR_LEFT_ECHO_PIN,SENSOR_LEFT_TRIGGER_PIN)
sensorR = DistanceSensor(SENSOR_RIGHT_ECHO_PIN,SENSOR_RIGHT_TRIGGER_PIN)
#sensorB = DistanceSensor(SENSOR_BACK_ECHO_PIN, SENSOR_BACK_TRIGGER_PIN)

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



x = 0
y = 0
thetad = 55

# these variables will change as devices are moved, need fine tuning
left_US_y = 75
right_US_y = 75
left_US_x = 60
right_US_x = 60
theta = thetad*pi/180
robot_distance = 160

def obstacleScanner():

    # grid = ObjectList()
    
    if sensor.distance < 0.3:
        #led.on()

        sensor_distance = sensor.distance * 1000
        distance = robot_distance + sensor_distance
        object_x = x + cos(theta) * distance
        object_y = y + sin(theta) * distance

        grid.set(object_x, object_y)
        sleep(0.01)

    if sensorL.distance < 0.3:
        #led.on()
        
        sensor_distanceL = sensorL.distance * 1000
        distance = robot_distance + sensor_distanceL
        object_x = x - sin(theta) * distance
        object_y = y + cos(theta) * distance
        
        grid.set(object_x,object_y)
        sleep(0.01)
    
    if sensorR.distance < 0.3:
        #led.on()
        
        sensor_distanceR = sensorR.distance * 1000
        distance = robot_distance + sensor_distanceR
        object_x = x + sin(theta) * distance
        object_y = y - cos(theta) * distance
        
        grid.set(object_x,object_y)
        sleep(0.01)
    
        """if sensorB.distance < 0.3:
        #led.on()
        
        sensor_distanceB = sensorB.distance *1000
        distance = robot_distance + sensor_distanceB
        object_x = x - cos(theta) * distance
        object_y = y - sin(theta) * distance
        
        grid.set(object_x,object_y)
        sleep(0.01)"""

    #else:
        #led.off()

    list = np.array(grid.obstacles)

    sleep(1)

    return list
    
