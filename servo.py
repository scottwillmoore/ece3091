# Navigation of robot
# Taylor Christie
# 09/08/2021

import time
import pigpio

LEFT_SERVO_PIN = 10
RIGHT_SERVO_PIN = 9
frequency = 50

pLeft = pigpio.pi()
pLeft.set_mode(LEFT_SERVO_PIN, pigpio.OUTPUT)
pLeft.set_PWM_frequency(LEFT_SERVO_PIN, frequency)

pRight = pigpio.pi()
pRight.set_mode(RIGHT_SERVO_PIN, pigpio.OUTPUT)
pRight.set_PWM_frequency(RIGHT_SERVO_PIN, frequency)

# rotate 270 degrees and return to starting point
pLeft.set_servo_pulsewidth(LEFT_SERVO_PIN, 1800)
pRight.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1730)
time.sleep(0.65)
pLeft.set_servo_pulsewidth(LEFT_SERVO_PIN, 1500)
pRight.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1500)
time.sleep(2)
pLeft.set_servo_pulsewidth(LEFT_SERVO_PIN, 1200)
pRight.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1130)
time.sleep(0.6)
pLeft.set_servo_pulsewidth(LEFT_SERVO_PIN, 1500)
pRight.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1500)
time.sleep(2)
