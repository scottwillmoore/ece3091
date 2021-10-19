from gpiozero import DistanceSensor

class UltrasonicSensor:
    def __init__(self, echo_pin: int, trigger_pin: int, range: int=0):
        self._echo_pin = echo_pin
        self._trigger_pin = trigger_pin
        self._sensor = DistanceSensor(self._echo_pin,self._trigger_pin)
    
    def read_distance(self):
        range = self._sensor.distance
        print(range)
        self.range = range


