from __future__ import annotations
from typing import List


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


# https://github.com/merose/diff_drive/blob/master/nodes/diff_drive_controller
# https://github.com/merose/diff_drive/blob/master/nodes/diff_drive_odometry


class DriveControllerNode(Node):
    _update_rate: int
    _timeout_duration: float

    _base_frame_id: str
    _odometry_frame_id: str

    _wheel_separation: float

    _transform_broadcaster: TransformBroadcaster
    _odometry_publisher: Publisher

    _left_desired_velocity_publisher: Publisher
    _right_desired_velocity_publisher: Publisher

    _desired_velocity_subscription: Subscription

    _left_velocity_subscription: Subscription
    _right_velocity_subscription: Subscription

    _timer: Timer

    def __init__(self):
        super().__init__("drive_controller")

        self._declare_parameters()
        self._get_parameters()
        self.add_on_set_parameters_callback(self._update_parameters)

        self._create_publishers()
        self._create_subscriptions()

        self._create_timers()
        self._update_timers()

    def _declare_parameters(self) -> None:
        self.declare_parameter("update_rate", Parameter.Type.INTEGER)
        self.declare_parameter("timeout_duration", Parameter.Type.DOUBLE)

        self.declare_parameter("base_frame_id", Parameter.Type.STRING)
        self.declare_parameter("odometry_frame_id", Parameter.Type.STRING)

        self.declare_parameter("wheel_separation", Parameter.Type.DOUBLE)

    def _get_parameters(self) -> None:
        self._update_rate = self.get_parameter("update_rate").value
        assert self._update_rate >= 0

        self._timeout_duration = self.get_parameter("timeout_duration").value
        assert self._timeout_duration >= 0.0

        self._base_frame_id = self.get_parameter("base_frame_id").value
        assert self._base_frame_id

        self._odometry_frame_id = self.get_parameter("odometry_frame_id").value
        assert self._odometry_frame_id

        self._wheel_separation = self.get_parameter("wheel_separation").value
        assert self._wheel_separation >= 0.0

    def _update_parameters(self, parameters: List[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name is None:
                continue

            elif parameter.name == "update_rate":
                self._update_rate = parameter.value
                assert self._update_rate >= 0
                self._update_timers()

            elif parameter.name == "timeout_duration":
                self._timeout_duration = parameter.value
                assert self._timeout_duration >= 0.0

            elif parameter.name == "base_frame_id":
                self._base_frame_id = parameter.value
                assert self._base_frame_id

            elif parameter.name == "odometry_frame_id":
                self._odometry_frame_id = parameter.value
                assert self._odometry_frame_id

            elif parameter.name == "wheel_separation":
                self._wheel_separation = parameter.value
                assert self._wheel_separation >= 0.0

        return SetParametersResult(successful=True)

    def _create_publishers(self) -> None:
        self._transform_broadcaster = TransformBroadcaster(self, 10)

        self._odometry_publisher = self.create_publisher(Odometry, "odometry", 10)

        self._left_desired_velocity_publisher = self.create_publisher(
            Float64, "left_desired_velocity", 10
        )

        self._right_desired_velocity_publisher = self.create_publisher(
            Float64, "right_desired_velocity", 10
        )

    def _create_subscriptions(self) -> None:
        self._desired_velocity_subscription = self.create_subscription(
            Twist, "desired_velocity", self._desired_velocity_callback, 10
        )

        self._left_velocity_subscription = self.create_subscription(
            Float64, "left_velocity", self._left_velocity_callback, 10
        )

        self._right_velocity_subscription = self.create_subscription(
            Float64, "right_velocity", self._right_velocity_callback, 10
        )

    def _create_timers(self) -> None:
        update_period = self._update_rate ** -1
        self._timer = self.create_timer(update_period, self._timer_callback)

    def _update_timers(self) -> None:
        if self._update_rate == 0:
            self._timer.cancel()

        else:
            update_period = self._update_rate ** -1
            self._timer.timer_period_ns = update_period * 1e9

            if self._timer.is_canceled():
                self._timer.reset()

    def _desired_velocity_callback(self, message: Twist) -> None:
        pass

    def _left_velocity_callback(self, message: Float64) -> None:
        pass

    def _right_velocity_callback(self, message: Float64) -> None:
        pass

    def _timer_callback(self):
        self.get_logger().info("...")


def main(args=None):
    init(args=args)
    node = DriveControllerNode()

    try:
        spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    shutdown()


if __name__ == "__main__":
    main()

