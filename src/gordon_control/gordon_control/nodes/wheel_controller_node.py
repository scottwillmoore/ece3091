from __future__ import annotations
from typing import List


from rcl_interfaces.msg import SetParametersResult
from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.timer import Timer
from std_msgs.msg import Float64


from gordon_control.controllers.pid_controller import PIDController
from gordon_control.controllers.wheel_controller import WheelController
from gordon_control.drivers.motor import Motor
from gordon_control.drivers.rotary_encoder import RotaryEncoder


class WheelControllerNode(Node):
    _velocity_publisher: Publisher

    _desired_velocity_subscription: Subscription

    _timer: Timer

    _motor: Motor
    _rotary_encoder: RotaryEncoder

    _pid_controller: PIDController

    _wheel_controller: WheelController

    _update_rate: int
    _timeout_duration: float

    _last_desired_velocity_time: Time
    _last_update_time: Time

    def __init__(self):
        super().__init__("wheel_controller")

        self.declare_parameter("update_rate", Parameter.Type.INTEGER)
        self.declare_parameter("timeout_duration", Parameter.Type.DOUBLE)

        self.declare_parameter("a_pin", Parameter.Type.INTEGER)
        self.declare_parameter("b_pin", Parameter.Type.INTEGER)
        self.declare_parameter("direction_pin", Parameter.Type.INTEGER)
        self.declare_parameter("speed_pin", Parameter.Type.INTEGER)

        self.declare_parameter("k_p", Parameter.Type.DOUBLE)
        self.declare_parameter("k_i", Parameter.Type.DOUBLE)
        self.declare_parameter("k_d", Parameter.Type.DOUBLE)

        self.declare_parameter("k_t", Parameter.Type.DOUBLE)

        self.declare_parameter("motor_to_wheel_ratio", Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)

        self._velocity_publisher = self.create_publisher(Float64, "velocity", 10)
        self.create_subscription(Float64, "desired_velocity", self._desired_velocity_callback, 10)

        self._update_rate = self.get_parameter("update_rate").value
        self._timeout_duration = self.get_parameter("timeout_duration").value

        update_period = self._update_rate ** -1
        self._timer = self.create_timer(update_period, self._timer_callback)

        a_pin = self.get_parameter("a_pin").value
        b_pin = self.get_parameter("b_pin").value
        direction_pin = self.get_parameter("direction_pin").value
        speed_pin = self.get_parameter("speed_pin").value

        self._motor = Motor(direction_pin, speed_pin)
        self._rotary_encoder = RotaryEncoder(a_pin, b_pin)

        k_p = self.get_parameter("k_p").value
        k_i = self.get_parameter("k_i").value
        k_d = self.get_parameter("k_d").value

        k_t = self.get_parameter("k_t").value

        self._pid_controller = PIDController(
            k_p, k_i, k_d, k_t=k_t, u_min=self._motor.min_speed, u_max=self._motor.max_speed
        )

        motor_to_wheel_ratio = self.get_parameter("motor_to_wheel_ratio").value
        wheel_radius = self.get_parameter("wheel_radius").value

        self._wheel_controller = WheelController(
            self._motor,
            self._rotary_encoder,
            self._pid_controller,
            motor_to_wheel_ratio=motor_to_wheel_ratio,
            wheel_radius=wheel_radius,
        )

        current_time = self.get_clock().now()
        self._last_desired_velocity_time = current_time
        self._last_update_time = current_time

        self.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def destroy_node(self) -> bool:
        self._motor.close()
        self._rotary_encoder.close()

        return super().destroy_node()

    def _on_set_parameters_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "":
                continue

            elif parameter.name == "update_rate":
                self._update_rate = parameter.value

                if self._update_rate == 0:
                    self._timer.cancel()

                else:
                    update_period = self._update_rate ** -1
                    self._timer.timer_period_ns = update_period * 1e9

                    if self._timer.is_canceled():
                        self._timer.reset()

            elif parameter.name == "timeout_duration":
                self._timeout_duration = parameter.value

            elif parameter.name == "a_pin":
                return SetParametersResult(successful=False)

            elif parameter.name == "b_pin":
                return SetParametersResult(successful=False)

            elif parameter.name == "direction_pin":
                return SetParametersResult(successful=False)

            elif parameter.name == "speed_pin":
                return SetParametersResult(successful=False)

            elif parameter.name == "k_p":
                self._pid_controller.k_p = parameter.value
                pass

            elif parameter.name == "k_i":
                self._pid_controller.k_i = parameter.value
                pass

            elif parameter.name == "k_d":
                self._pid_controller.k_d = parameter.value

            elif parameter.name == "k_t":
                self._pid_controller.k_t = parameter.value

            elif parameter.name == "motor_to_wheel_ratio":
                self._wheel_controller.motor_to_wheel_ratio = parameter.value

            elif parameter.name == "wheel_radius":
                self._wheel_controller.wheel_radius = parameter.value

        return SetParametersResult(successful=True)

    def _desired_velocity_callback(self, message: Float64) -> None:
        current_time = self.get_clock().now()
        self._last_desired_velocity_time = current_time
        self._wheel_controller.desired_velocity = message.data

    def _timer_callback(self):
        current_time = self.get_clock().now()

        elapsed_time = (current_time - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = current_time

        elapsed_timeout_time = (current_time - self._last_desired_velocity_time).nanoseconds / 1e9
        if self._timeout_duration and elapsed_timeout_time >= self._timeout_duration:
            self._wheel_controller.desired_velocity = 0.0

        self._wheel_controller.update(elapsed_time)

        velocity = Float64()
        velocity.data = self._wheel_controller.velocity

        self._velocity_publisher.publish(velocity)


def main(args=None):
    init(args=args)
    node = WheelControllerNode()

    try:
        spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    shutdown()


if __name__ == "__main__":
    main()
