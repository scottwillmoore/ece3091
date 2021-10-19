class Strategy:

    _goal_position_x: float
    _goal_position_y: float
    _obstacle: float

    _velocity: float

    # not sure if we will end up needing obstacle defined in class...
    def __init__(self,goal_position_x: float, goal_position_y: float, obstacle: float)->None:
        self._goal_position_x = goal_position_x
        self._goal_position_y = goal_position_y
        self._obstacle = obstacle


    # read the camera
    # if there is a ball bearing present, drive towards goal position
    # if obstacle within 5cm: stop driving, rotate towards left, drive forwards once obstacle gone
    # if no ball bearing drive forwards until obstacle is present, or ball bearing is found

