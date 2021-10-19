from gpiozero import DistanceSensor

class UltrasonicSensor:
    @property
    def range(self) -> float:
        return self._sensor.distance


    def __init__(self, echo_pin: int, trigger_pin: int):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin
        self._sensor = DistanceSensor(self._echo_pin,self._trigger_pin)

sensor2=UltrasonicSensor(24,23)
print(sensor2.range)

