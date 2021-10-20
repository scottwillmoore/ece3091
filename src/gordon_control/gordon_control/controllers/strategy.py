from state_machine import StateMachine

# the msg component is leftover from the code from google
# i don't think it will have an effect if we just leave it there
# if you remove it, you need to change state_machine.py

# not sure if this kind of structure will work, or if needs to be mulitple classes/functions/files

# the print statements were for testing

class Strategy:

    _goal_position_x: float
    _goal_position_y: float
    _operate_gate: bool
    _obstacle: float

    _left_desired_velocity: float
    _right_desired_velocity: float

    # somehow need all the information to come from the strategy node publishing/subscribing
    def __init__(self,goal_position_x: float, goal_position_y: float, operate_gate: bool)->None:
        self._goal_position_x = goal_position_x
        self._goal_position_y = goal_position_y
        self._operate_gate = operate_gate
        # self._obstacle = obstacle


    def start_transitions(msg):
    # subscribe to the coords from the camera and check for ball bearing
        ball_bearing = False
        if self._goal_position_x != 10000 and self._goal_position_y = !10000: # use some dummy output coordinates from camera for no ball bearing
            ball_bearing = True
        
        if ball_bearing is True:
            newState = "Drive_to_goal"
            print("drive 1")
        else:
            newState = "Scan_left"
            print("scan 1")
        return (newState, msg)

    def drive_to_goal_transitions(msg):
        close = False
        # first test the position of goal for ball bearing proximity
        if 0<self._goal_position_x<1 and 2<self._goal_position_y<2: #untested, need to update values
            close = True
        # only drive if not close to ball bearing
        if close is False:
            #if self._goal_position_y > pixel 250
                # direction to left
            # if self._goal_position_y < pixel 250
                # direction to right
            # calculate the direction from camera, SAAD
            velocity = 0.01 # set a velocity
            direction = 1 # set the direction
            sleep(5) # drive for x amount of time before rescanning
        
        # if get close to the ball bearing
        if close is True:
            newState = "Get_close"
            print("close")
        else:
            newState = "Drive_to_goal"
            print("drive x")
        return (newState, msg)

    def get_close_transitions(msg):
        velocity = 0.05 #continue driving straight for x time
        sleep(2) # we need to travel about 4cm from when the ball bearing exits cameras frame. I assume also processing delays
        velocity = 0 # stop (hopefully lined up with ball bearing)
        operate_gate = True # publish the message to operate gate
    
        newState = "End"
        print("to end")
        return (newState, msg)

    def end_state(msg):
    # could print time taken here but not necessary for marks
        print("Task Completed")
        return ("end_state", "")

    def scan_left_transitions(msg):
        # rotate to the left
        self._left_desired_velocity = 0.001
        self._right_resired_velocity = 0.01
       
        ball_bearing = False
        if self._goal_position_x != 10000 and self._goal_position_y = !10000: # use some dummy output coordinates from camera for no ball bearing
            ball_bearing = True
        
        if ball_bearing = True:
            newState = "Drive_to_goal"
            print("drive 2")
        else:
            newState = "Scan_right"
            print("scan 2")
        return (newState, msg)

    def scan_right_transitions(msg):
        # rotate to the right
        self._left_desired_velocity = 0.01
        self._right_resired_velocity = 0.001
       
        ball_bearing = False
        if self._goal_position_x != 10000 and self._goal_position_y = !10000: # use some dummy output coordinates from camera for no ball bearing
            ball_bearing = True
        if ball_bearing = True:
            newState = "Drive_to_goal"
            print("drive 3")
        else:
            newState = "Drive_forwards"
            print("scan 3")
        return (newState, msg)

    def drive_forwards_transitions(msg):
        # drive forwards
        self._left_desired_velocity = 0.01
        self._right_resired_velocity = 0.01
       
        ball_bearing = False
        if self._goal_position_x != 10000 and self._goal_position_y = !10000: # use some dummy output coordinates from camera for no ball bearing
            ball_bearing = True
        if ball_bearing is True:
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
