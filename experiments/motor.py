from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep
 
LEFT_MOTOR_PHASE_PIN = 6
LEFT_MOTOR_ENABLE_PIN = 13
 
leftMotor = PhaseEnableMotor(phase=LEFT_MOTOR_PHASE_PIN, enable=LEFT_MOTOR_ENABLE_PIN)
 
RIGHT_MOTOR_PHASE_PIN = 26
RIGHT_MOTOR_ENABLE_PIN = 19
 
rightMotor = PhaseEnableMotor(phase=RIGHT_MOTOR_PHASE_PIN, enable=RIGHT_MOTOR_ENABLE_PIN)
 
LEFT_ENCODER_A_PIN = 23
LEFT_ENCODER_B_PIN = 24
 
leftEncoder = RotaryEncoder(a=LEFT_ENCODER_A_PIN, b=LEFT_ENCODER_B_PIN, max_steps = 0)
 
RIGHT_ENCODER_A_PIN = 25
RIGHT_ENCODER_B_PIN = 12
 
rightEncoder = RotaryEncoder(a=RIGHT_ENCODER_A_PIN, b=RIGHT_ENCODER_B_PIN,max_steps = 0)
 
MOTOR_TO_WHEEL_RATIO = 1 / (32 * 344.2)


rightMotor.forward(0.4)
leftMotor.forward(0.4)

print("forward")
sleep(5)
 
rightMotor.stop()
leftMotor.stop()

print("stop")
print(f"left steps: {-leftEncoder.steps}")
print(f"right steps: {rightEncoder.steps}")
print(f"left revolutions: {-leftEncoder.steps * MOTOR_TO_WHEEL_RATIO}")
print(f"right revolutions: {rightEncoder.steps * MOTOR_TO_WHEEL_RATIO}")
