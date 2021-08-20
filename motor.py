# https://gpiozero.readthedocs.io/en/stable/api_output.html#phaseenablemotor
# https://gpiozero.readthedocs.io/en/stable/api_input.html#rotaryencoder

from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

LEFT_MOTOR_PHASE_PIN = 31
LEFT_MOTOR_ENABLE_PIN = 33

leftMotor = PhaseEnableMotor(LEFT_MOTOR_PHASE_PIN, LEFT_MOTOR_PHASE_PIN)

RIGHT_MOTOR_PHASE_PIN = 37
RIGHT_MOTOR_ENABLE_PIN = 35

rightMotor = PhaseEnableMotor(RIGHT_MOTOR_PHASE_PIN, RIGHT_MOTOR_PHASE_PIN)

LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 18

leftEncoder = RotaryEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)

RIGHT_ENCODER_A_PIN = 22
RIGHT_ENCODER_B_PIN = 28

rightEncoder = RotaryEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)

while True:
    leftMotor.forward(1)
    rightMotor.forward(1)
    print("forward")
    sleep(1)

    leftMotor.stop()
    rightMotor.stop()
    print("stop")
    print(f"leftEncoder.steps: {leftEncoder.steps}")
    print(f"rightEncoder.steps: {rightEncoder.steps}")
    sleep(1)
