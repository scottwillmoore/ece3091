from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

from constants import *
from pins import *

SPEED = 1
DURATION = 5

leftMotor = PhaseEnableMotor(phase=LEFT_MOTOR_PHASE_PIN, enable=LEFT_MOTOR_ENABLE_PIN)
rightMotor = PhaseEnableMotor(phase=RIGHT_MOTOR_PHASE_PIN, enable=RIGHT_MOTOR_ENABLE_PIN)

leftEncoder = RotaryEncoder(a=LEFT_ENCODER_A_PIN, b=LEFT_ENCODER_B_PIN, max_steps=0)
rightEncoder = RotaryEncoder(a=RIGHT_ENCODER_A_PIN, b=RIGHT_ENCODER_B_PIN, max_steps=0)

rightMotor.forward(SPEED)
leftMotor.forward(SPEED)

sleep(DURATION)
 
rightMotor.stop()
leftMotor.stop()

leftSteps = leftEncoder.steps
rightSteps = rightEncoder.steps

print(f"leftSteps: {leftSteps}")
print(f"rightSteps: {rightSteps}")

leftTurns = leftSteps * MOTOR_TO_WHEEL_RATIO
rightTurns = rightSteps * MOTOR_TO_WHEEL_RATIO

print(f"leftTurns: {leftTurns}")
print(f"rightTurns: {rightTurns}")
