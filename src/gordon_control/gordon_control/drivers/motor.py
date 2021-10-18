import pigpio


DEFAULT_MIN_SPEED = -1.0
DEFAULT_MAX_SPEED = +1.0

DEFAULT_PWM_FREQUENCY = 10_000


class Motor:
    @property
    def min_speed(self) -> float:
        return self._min_speed

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, speed: float) -> None:
        self._speed = speed

        direction_value = int(self._speed < 0)
        speed_duty_cycle = int(abs(self._speed) * 1_000_000)

        self._pi.write(self._direction_pin, direction_value)
        self._pi.hardware_PWM(self._speed_pin, self._pwm_frequency, speed_duty_cycle)

    _pi: pigpio.pi

    _direction_pin: int
    _speed_pin: int

    _min_speed: float
    _max_speed: float

    _pwm_frequency: int

    _speed: float = 0.0

    def __init__(
        self,
        direction_pin: int,
        speed_pin: int,
        *,
        min_speed: float = DEFAULT_MIN_SPEED,
        max_speed: float = DEFAULT_MAX_SPEED,
        pwm_frequency: int = DEFAULT_PWM_FREQUENCY,
    ) -> None:
        self._direction_pin = direction_pin
        self._speed_pin = speed_pin

        self._min_speed = min_speed
        self._max_speed = max_speed
        self._pwm_frequency = pwm_frequency

        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Failed to initialize GPIO")

        self._pi.set_mode(self._direction_pin, pigpio.OUTPUT)

        error = self._pi.hardware_PWM(self._speed_pin, self._pwm_frequency, 0)
        if error > 0:
            raise RuntimeError("Failed to initialize hardware PWM")

    def close(self) -> None:
        self._pi.set_mode(self._direction_pin, pigpio.OUTPUT)
        self._pi.set_mode(self._speed_pin, pigpio.OUTPUT)

        self._pi.write(self._direction_pin, 0)
        self._pi.write(self._speed_pin, 0)
