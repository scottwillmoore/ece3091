from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Range

from gordon_control.controllers.ultrasonic_controller import UltrasonicSensor

class UltrasonicControllerNode(Node):

  _ultrasonic: UltrasonicSensor

  _last_update_time: Time
  
  def __init__(self):
    super().__init__("ultrasonic_controller")

    self.declare_parameter("echo_pin", Parameter.Type.INTEGER)
    self.declare_parameter("trigger_pin", Parameter.Type.INTEGER)

    self._range_publisher = self.create_publisher(Range, "range", 10)

    self._timer = self.create_timer(2.0, self._timer_callback)

    echo_pin = self.get_parameter("echo_pin").value
    trigger_pin = self.get_parameter("trigger_pin").value

    self._ultrasonic = UltrasonicSensor(echo_pin,trigger_pin)

    current_time = self.get_clock().now()
    self._last_update_time = current_time

  def _timer_callback(self):
    current_time = self.get_clock().now()

    self._last_update_time = current_time

    measurement = Range()
    measurement.header
    #measurement.header
    measurement.radiation_type = 0
    measurement.header.stamp
    measurement.field_of_view = 0.5236
    measurement.min_range = 0.002
    measurement.max_range = 1.0
    measurement.range = self._ultrasonic.range

    self._range_publisher.publish(measurement)
    
def main(args=None):
  init(args=args)
  node = UltrasonicControllerNode()

  try:
    spin(node)
  except KeyboardInterrupt:
    pass

  node.destroy_node()
  shutdown()

if __name__ == "__main__":
    main()
