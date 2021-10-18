import pigpio


class RotaryEncoder:
    @property
    def count(self) -> int:
        return self._count

    _pi: pigpio.pi

    _a_pin: int
    _b_pin: int

    _previous_pin: int = 0

    _a_value: int = 0
    _b_value: int = 0

    _count: int = 0

    def __init__(self, a_pin: int, b_pin: int) -> None:
        self._a_pin = a_pin
        self._b_pin = b_pin

        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Failed to initialize GPIO")

        self._pi.set_mode(self._a_pin, pigpio.INPUT)
        self._pi.set_mode(self._b_pin, pigpio.INPUT)

        self._pi.set_pull_up_down(self._a_pin, pigpio.PUD_DOWN)
        self._pi.set_pull_up_down(self._b_pin, pigpio.PUD_DOWN)

        self._pi.callback(self._a_pin, pigpio.EITHER_EDGE, self._callback)
        self._pi.callback(self._b_pin, pigpio.EITHER_EDGE, self._callback)

    def close(self) -> None:
        self._pi.set_mode(self._a_pin, pigpio.OUTPUT)
        self._pi.set_mode(self._b_pin, pigpio.OUTPUT)

        self._pi.write(self._a_pin, 0)
        self._pi.write(self._b_pin, 0)

    def _callback(self, pin: int, value: int, _: int) -> None:
        if pin == self._a_pin:
            self._a_value = value

        elif pin == self._b_pin:
            self._b_value = value

        if not pin == self._previous_pin:
            self._previous_pin = pin

            if pin == self._a_pin and value == 1:
                if self._b_value == 1:
                    self._count -= 1

            elif pin == self._b_pin and value == 1:
                if self._a_value == 1:
                    self._count += 1
