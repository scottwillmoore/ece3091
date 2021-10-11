# Navigation of robot
# Taylor Christie
# 09/08/2021

import time
import pigpio

LEFT_SERVO_PIN = 8
RIGHT_SERVO_PIN = 11
SERVO_PWM_FREQUENCY = 50

pi = pigpio.pi()

pi.set_mode(LEFT_SERVO_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(LEFT_SERVO_PIN, SERVO_PWM_FREQUENCY)

pi.set_mode(RIGHT_SERVO_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(RIGHT_SERVO_PIN, SERVO_PWM_FREQUENCY)

# rotate 270 degrees and return to starting point
pi.set_servo_pulsewidth(LEFT_SERVO_PIN, 2000)
pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1000)
time.sleep(1.5)
pi.set_servo_pulsewidth(LEFT_SERVO_PIN, 1500)
pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1500)
time.sleep(1.5)

pi.set_servo_pulsewidth(LEFT_SERVO_PIN, 1300)
pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1700)
time.sleep(2.1)
pi.set_servo_pulsewidth(LEFT_SERVO_PIN, 1500)
pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, 1500)
time.sleep(1)
