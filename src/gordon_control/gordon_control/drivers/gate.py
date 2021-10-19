import time
import pigpio

SERVO_PWM_FREQUENCY = 50

class Gate:

    _pi: pigpio.pi 

    _left_servo_pin: int
    _right_servo_pin: int

    _pwm_frequency: int


    def __init__(self, left_servo_pin: int, right_servo_pin: int, pwm_frequency: int=SERVO_PWM_FREQUENCY) -> None:
        self._left_servo_pin = left_servo_pin
        self._right_servo_pin = right_servo_pin

        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Failed to initialize GPIO")

        self._pwm_frequency = pwm_frequency

        self._pi.set_mode(self._left_servo_pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(self._left_servo_pin, self._pwm_frequency)

        self._pi.set_mode(self._right_servo_pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(self._right_servo_pin, self._pwm_frequency)

    def rotate(self) -> None:
        self._pi.set_servo_pulsewidth(self._left_servo_pin, 2000)
        self._pi.set_servo_pulsewidth(self._right_servo_pin, 1000)
        time.sleep(1.5)
        self._pi.set_servo_pulsewidth(self._left_servo_pin, 1500)
        self._pi.set_servo_pulsewidth(self._right_servo_pin, 1500)
        time.sleep(1.5)

        self._pi.set_servo_pulsewidth(self._left_servo_pin, 1300)
        self._pi.set_servo_pulsewidth(self._right_servo_pin, 1700)
        time.sleep(2.1)
        self._pi.set_servo_pulsewidth(self._left_servo_pin, 1500)
        self._pi.set_servo_pulsewidth(self._right_servo_pin, 1500)
        time.sleep(1)

    def close(self) -> None:
        self._pi.set_mode(self._left_servo_pin, pigpio.OUTPUT)
        self._pi.set_mode(self._right_servo_pin, pigpio.OUTPUT)

        self._pi.write(self._left_servo_pin, 0)
        self._pi.write(self._right_servo_pin, 0)


gate = Gate(25,18)
gate.rotate()