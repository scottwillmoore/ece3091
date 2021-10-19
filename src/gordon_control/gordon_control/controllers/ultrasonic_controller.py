from gpiozero import DistanceSensor

class UltrasonicSensor:
    def __init__(self, echo_pin: int, trigger_pin: int):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin
        self._sensor = DistanceSensor(self._echo_pin,self._trigger_pin)
    
    def read_distance(self):
        range_measure = self._sensor.distance
        self.range = range_measure

