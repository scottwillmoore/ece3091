from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.time import Time

from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from gordon_control.controllers.strategy import Strategy

class StrategyNode(Node):

  _strategy: Strategy

  _wheel_controller_publisher: Publisher
  _gate_controller_publisher: Publisher

  _camera_subscription: Subscription
  _front_ultrasonic_subscription: Subscription
  _left_ultrasonic_subscription: Subscription
  _left_velocity_subscription: Subscription
  _right_velocity_subscription: Subscription

  def __init__(self):
    super().__init__("action_controller")

    self._create_publishers()
    self._create_subscriptions()


  # publish information to the wheels
  def _create_publishers(self) -> None:
    self._wheel_controller_publisher = self.create_publisher(Twist, "wheel_controller", 10)
    self._gate_controller_publisher = self.create_publisher(Bool, "gate_controller", 10)

  # subscribe to information from the US, camera, wheels
  def _create_subscriptions(self) -> None:
    self._left_velocity_subscription = self.create_subscription(Float64, "left_velocity", self._left_velocity_callback, 10)
    self._right_velocity_subscription = self.create_subscription(Float64, "right_velocity", self._right_velocity_callback, 10)
    self._camera_subscription = self.create_subscription(PointStamped, "point", self._camera_controller_callback, 10)
    self._front_ultrasonic_subscription = self.create_subscription(Range, "range", self._ultrasonic_callback, 10)
    self._left_ultrasonic_subscription = self.create_subscription(Range, "range", self._ultrasonic_callback, 10)

  # not sure if we need to know the velocity
  def _left_velocity_callback(self, message: Float64) -> None:
    pass

  def _right_velocity_callback(self, message: Float64) -> None:
    pass
  
  def _camera_controller_callback(self, message) -> None:
    self._strategy_goal_position_x = message.point.x 
    self._strategy_goal_position_y = message.point.y

  def _ultrasonic_callback(self, message) -> None:
    self._strategy_obstacle = message.data
  
    

    

def main(args=None):
  init(args=args)
  node = StrategyNode()

  try:
    spin(node)
  except KeyboardInterrupt:
    pass

  node.destroy_node()
  shutdown()

if __name__ == "__main__":
    main()
