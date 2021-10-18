from math import pi

from gordon_control.controllers.pid_controller import PIDController
from gordon_control.drivers.motor import Motor
from gordon_control.drivers.rotary_encoder import RotaryEncoder


class WheelController:
    motor_to_wheel_ratio: float
    wheel_radius: float

    desired_velocity: float = 0.0

    @property
    def velocity(self) -> float:
        return self._velocity

    _motor: Motor
    _rotary_encoder: RotaryEncoder

    _pid_controller: PIDController

    _previous_count: int = 0

    _velocity: float = 0.0

    def __init__(
        self,
        motor: Motor,
        rotary_encoder: RotaryEncoder,
        pid_controller: PIDController,
        *,
        motor_to_wheel_ratio: float,
        wheel_radius: float,
    ) -> None:
        self._motor = motor
        self._rotary_encoder = rotary_encoder

        self._pid_controller = pid_controller

        self._motor_to_wheel_ratio = motor_to_wheel_ratio
        self._wheel_radius = wheel_radius

        self._previous_count = self._rotary_encoder.count

    def update(self, elapsed_time: float) -> None:
        count = self._rotary_encoder.count
        elapsed_count = count - self._previous_count
        self._previous_count = count

        elapsed_rotation = self._motor_to_wheel_ratio * elapsed_count

        self._angular_velocity = 2 * pi * elapsed_rotation / elapsed_time
        self._velocity = self._wheel_radius * self._angular_velocity

        speed = self._pid_controller.update(self.desired_velocity, self._velocity, elapsed_time)

        self._motor.speed = speed
