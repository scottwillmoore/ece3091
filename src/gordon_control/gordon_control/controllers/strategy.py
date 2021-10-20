from state_machine import StateMachine

# the msg component is leftover from the code from google
# i don't think it will have an effect if we just leave it there
# if you remove it, you need to change state_machine.py

# not sure if this kind of structure will work, or if needs to be mulitple classes/functions/files

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


    def start_transitions(msg):
    # subscribe to the coords from the camera
    # if ball bearing is present
        if :
            newState = "Drive_to_goal"
            print("drive 1")
        else:
            newState = "Scan_left"
            print("scan 1")
        return (newState, msg)

    def drive_to_goal_transitions(msg):
    # publish wheel direction which is aligned to the coordinates
    # publish a set velocity
    # add obstacle avoidance later
    # if get close to the ball bearing
        if :
            newState = "Get_close"
            print("close")
        else:
            newState = "Drive_to_goal"
            print("drive x")
        return (newState, msg)

    def get_close_transitions(msg):
    # publish velocity as stop
    # this will require testing to get it to stop in the right spot from camera
    # publish gripper operation bool as true
        newState = "End"
        print("to end")
        return (newState, msg)

    def end_state(msg):
    # could print time taken here but not necessary for marks
        print("Task Completed")
        return ("end_state", "")

    def scan_left_transitions(msg):
    # publish the wheels to rotate left
    # subscribe to coords from camera
    # if ball bearing
        msg = "scan left"
        if :
            newState = "Drive_to_goal"
            print("drive 2")
        else:
            newState = "Scan_right"
            print("scan 2")
        return (newState, msg)

    def scan_right_transitions(msg):
    # publish the wheels to rotate right
    # subscribe to coords from camera
    # if ball bearing
        if :
            newState = "Drive_to_goal"
            print("drive 3")
        else:
            newState = "Drive_forwards"
            print("scan 3")
        return (newState, msg)

    def drive_forwards_transitions(msg):
    # publish the wheels to drive forwards a set amount
    # subscribe to the coords from camera
    # if ball bearing
        if :
            newState = "Drive_to_goal"
            print("drive 4")
        else:
            newState = "Scan_left"
            print("scan 4")
        return (newState, msg)

    machine = StateMachine()
    machine.add_state("Start", start_transitions)
    machine.add_state("Drive_to_goal", drive_to_goal_transitions)
    machine.add_state("Get_close", get_close_transitions)
    machine.add_state("End", None, end_state=1)

    machine.add_state("Scan_left", scan_left_transitions)
    machine.add_state("Scan_right", scan_right_transitions)
    machine.add_state("Drive_forwards", drive_forwards_transitions)

    machine.set_start("Start")

    machine.run("Gordon")
