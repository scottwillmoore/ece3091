from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.timer import Timer
from std_msgs.msg import Float64

from gordon_control.controllers.ultrasonic_controller import UltrasonicController
from gordon_control.drivers.ultrasonic import UltrasonicSensor

class UltrasonicControllerNode(Node):

  _ultrasonic: UltrasonicSensor

  _last_update_time: Time
  
  def __init__(self):
    super().__init__("ultrasonic_controller")

    self.declare_parameter("echo_pin", Parameter.Type.INTEGER)
    self.declare_parameter("trigger_pin", Parameter.Type.INTEGER)

    self._range_publisher = self.create_publisher(Float64, "range", 10)

    self._timer = self.create_timer(2.0, self._timer_callback)

    echo_pin = self.get_parameter("echo_pin").value
    trigger_pin = self.get_parameter("trigger_pin").value

    self._ultrasonic = UltrasonicSensor(echo_pin,trigger_pin)

    current_time = self.get_clock().now()
    self._last_update_time = current_time

  def _timer_callback(self):
    current_time = self.get_clock().now()

    self._last_update_time = current_time

    range = Float64()
    range.data = self._ultrasonic_controller.range

    self._range_publisher.publish(range)
    
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