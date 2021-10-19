<<<<<<< HEAD
class UltrasonicController:
=======
from gpiozero import DistanceSensor

class UltrasonicSensor:
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b
    @property
    def range(self) -> float:
        return self._range

<<<<<<< HEAD
    _range: float = 0.0
        
    def __init__(self) -> None:

    def update(self) -> None:
        distance = x

        self._range = distance
=======
    def __init__(self, echo_pin: int, trigger_pin: int):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin

    def read_distance(self):
        sensor = DistanceSensor(self._echo_pin,self._trigger_pin)
        distance = sensor.distance
        self._range = distance
>>>>>>> a043740b232e6382b359af95b1dbf047e977588b
