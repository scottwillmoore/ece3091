from geometry_msgs.msg import Twist
from rclpy import init, shutdown, spin
from rclpy.node import Node
from std_msgs.msg import Int32


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")

        self.rate = self.declare_parameter("~rate", 10)
        self.wheel_separation = self.declare_parameter("~wheel_separation", 10.0)
        self.timeout = self.declare_parameter("~timeout", 0.1)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.desired_left_velocity_publisher = self.create_publisher(
            Int32, "desired_left_velocity", 10
        )

        self.desired_right_velocity_publisher = self.create_publisher(
            Int32, "desired_right_velocity", 10
        )

        self.desired_velocity_subscription = self.create_subscription(
            Twist, "desired_velocity", self.desired_velocity_callback, 10
        )

        self.create_timer(self.rate ** -1, self.timer_callback)

    def desired_velocity_callback(self, message):
        self.linear_velocity = message.linear.x
        self.angular_velocity = message.angular.z

    def timer_callback(self):

        self.get_logger().info("timer")


def main(args=None):
    init(args=args)
    drive_controller = DriveController()
    spin(drive_controller)
    drive_controller.destroy_node()
    shutdown()


if __name__ == "__main__":
    main()

