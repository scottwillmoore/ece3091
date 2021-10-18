from .wheel_controller import WheelController


class DriveController:
    _left: WheelController
    _right: WheelController
    
    _wheel_separation: float
