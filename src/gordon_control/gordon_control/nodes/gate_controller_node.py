from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from gordon_control.drivers.gate import Gate

class GateControllerNode(Node):
    
    _gate: Gate

    def __init__(self):
        super().__init__("gate_controller")

        self.declare_parameter("left_servo_pin", Parameter.Type.INTEGER)
        self.declare_parameter("right_servo_pin", Parameter.Type.INTEGER)

        self._gate_subscriber = self.create_subscription(Bool, "gate", self._listener_callback, 10)

        left_servo_pin = self.get_parameter("left_servo_pin").value
        right_servo_pin = self.get_parameter("right_servo_pin").value

        self._gate = Gate(left_servo_pin,right_servo_pin)

    def _listener_callback(self, message: Bool) -> None:
        if message.data is True: #untested
            _gate.rotate() #untested

def main(args=None):
  init(args=args)
  node = GateControllerNode()

  try:
    spin(node)
  except KeyboardInterrupt:
    pass

  node.destroy_node()
  shutdown()

if __name__ == "__main__":
    main()
