# https://gpiozero.readthedocs.io/en/stable/api_output.html#phaseenablemotor
# https://gpiozero.readthedocs.io/en/stable/api_input.html#rotaryencoder

from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

LEFT_MOTOR_PHASE_PIN = 31
LEFT_MOTOR_ENABLE_PIN = 33

leftMotor = PhaseEnableMotor(phase=LEFT_MOTOR_PHASE_PIN, enable=LEFT_MOTOR_ENABLE_PIN)

RIGHT_MOTOR_PHASE_PIN = 37
RIGHT_MOTOR_ENABLE_PIN = 35

rightMotor = PhaseEnableMotor(phase=RIGHT_MOTOR_PHASE_PIN, enable=RIGHT_MOTOR_ENABLE_PIN)

LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 18

leftEncoder = RotaryEncoder(a=LEFT_ENCODER_A_PIN, b=LEFT_ENCODER_B_PIN)

RIGHT_ENCODER_A_PIN = 22
RIGHT_ENCODER_B_PIN = 28

rightEncoder = RotaryEncoder(a=RIGHT_ENCODER_A_PIN, b=RIGHT_ENCODER_B_PIN)

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
