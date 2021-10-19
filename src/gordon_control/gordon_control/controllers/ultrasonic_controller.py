from gpiozero import DistanceSensor

class UltrasonicSensor:
    @property
    def range(self) -> float:
        return self._range

    def __init__(self, echo_pin: int, trigger_pin: int):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin

    def read_distance(self):
        sensor = DistanceSensor(self._echo_pin,self._trigger_pin)
        distance = sensor.distance
        self._range = distance
