from device import Motor, RotaryEncoder, set_up, tear_down
from time import sleep

LEFT_MOTOR_DIRECTION_PIN = 6
LEFT_MOTOR_ENABLE_PIN = 13

LEFT_ENCODER_A_PIN = 9
LEFT_ENCODER_B_PIN = 11

ENCODER_RESOLUTION = 32
GEAR_RATIO = 344.2
WHEEL_TO_MOTOR_RATIO = ENCODER_RESOLUTION * GEAR_RATIO
MOTOR_TO_WHEEL_RATIO = WHEEL_TO_MOTOR_RATIO ** -1

SPEED = 1.0
DURATION = 5

if __name__ == "__main__":
    set_up()
    
    left_motor = Motor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_ENABLE_PIN)
    left_encoder = RotaryEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
    
    left_motor.setSpeed(SPEED)

    sleep(DURATION)

    left_motor.setSpeed(0)
    
    left_step_count = left_encoder.getCount()
    left_turn_count = left_step_count * MOTOR_TO_WHEEL_RATIO
    
    print(f"step_count: {left_step_count}")
    print(f"turn_count: {left_turn_count}")
