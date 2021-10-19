from gpiozero import DistanceSensor

class UltrasonicSensor:
    def __init__(self, echo_pin: int, trigger_pin: int, range: int=0,):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin
        self._sensor = DistanceSensor(self._echo_pin,self._trigger_pin)
        self._range = range
    
    def read_distance(self):
        range_measure = self._sensor.distance
        print(range_measure)
        self._range = range_measure


