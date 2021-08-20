# https://gpiozero.readthedocs.io/en/stable/api_input.html#distancesensor-hc-sr04

from gpiozero import DistanceSensor
from time import sleep

FRONT_SENSOR_ECHO_PIN = 38
FRONT_SENSOR_TRIGGER_PIN = 13

frontSensor = DistanceSensor(echo=FRONT_SENSOR_ECHO_PIN, trigger=FRONT_SENSOR_TRIGGER_PIN)

while True:
    print(f"distance: {frontSensor.distance}")
    sleep(1)
