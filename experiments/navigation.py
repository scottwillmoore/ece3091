import numpy as np
from matplotlib import pyplot as plt
from IPython import display
import gpiozero
import time

# Adaption of Michael's Navigation Code

# defined robot parameters
WHEELRADIUS = 56 #mm
WHEELCIRCUMFRENCE = 2* WHEELRADIUS * np.pi #mm
WHEELSEPERATION = 100 #mm, potentially need to asjust this value very slightly
TIMESTEP = 0.1
ENCODERRESOLUTION = 32 # not sure if this is the correct value?
KP = 0.1 # need to calculate this
KI = 0.01 # need to calculate this
INERTIA = 10 # need to calculate this
DRAG = 2 # need to calculate this

LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 18

encoderLeft = gpiozero.RotaryEncoder(a=LEFT_ENCODER_A_PIN, b=LEFT_ENCODER_B_PIN)

RIGHT_ENCODER_A_PIN = 22
RIGHT_ENCODER_B_PIN = 28

encoderRight = gpiozero.RotaryEncoder(a=RIGHT_ENCODER_A_PIN, b=RIGHT_ENCODER_B_PIN)


#####################################################################################


# parameters that may or may not be needed
"""motorRPM = 38 #rpm
motorRatio = 344.2 # :1
motorTorque = 2276 #gf/cm
"""

class DiffDriveRobot:

    def __init__(self, inertia=INERTIA, dt=TIMESTEP, drag=DRAG, wheel_radius=WHEELRADIUS, wheel_sep=WHEELSEPERATION,resolution=ENCODERRESOLUTION):

        self.x = 0.0  # y-position
        self.y = 0.0  # y-position
        self.th = 0.0  # orientation

        self.wl = 0.0  # rotational velocity left wheel
        self.wr = 0.0  # rotational velocity right wheel

        self.I = inertia
        self.d = drag
        self.dt = dt

        self.r = wheel_radius
        self.l = wheel_sep
        self.res = resolution

    # Should be replaced by motor encoder measurement which measures how fast wheel is turning
    # Here, we simulate the real system and measurement
    def encoder_measurement(self,dt,res):

        # count the number of ticks for the encoders
        preCountLeft = encoderLeft.steps
        preCountRight = encoderRight.steps
        time.sleep(dt)  # allow time for counting

        # determine how many ticks have occurred for the time period
        countLeft = encoderLeft.steps - preCountLeft
        countRight = encoderRight.steps - preCountRight

        # if step count is negative, then the robot is moving backwards
        # hence a negative angular velocity is in reverse

        angularDistanceLeft = countLeft / res
        angularDistanceRight = countRight / res

        wl = angularDistanceLeft / dt # encoder angular velocity left
        wr = angularDistanceRight / dt # encoder angular velocity right

        return wl, wr

    # Velocity motion model
    def base_velocity(self, wl, wr):

        v = (wl * self.r + wr * self.r) / 2.0

        w = (wl - wr) / self.l

        return v, w

    # Kinematic motion model
    def pose_update(self, duty_cycle_l, duty_cycle_r):

        self.wl = self.encoder_measurement(self.wl, duty_cycle_l)
        self.wr = self.encoder_measurement(self.wr, duty_cycle_r)

        v, w = self.base_velocity(self.wl, self.wr)

        self.x = self.x + self.dt * v * np.cos(self.th)
        self.y = self.y + self.dt * v * np.sin(self.th)
        self.th = self.th + w * self.dt

        return self.x, self.y, self.th


class RobotController:

    # need to write some code to determine appropriate values for Kp and Ki
    def __init__(self, Kp=KP, Ki=KI, wheel_radius=WHEELRADIUS, wheel_sep=WHEELSEPERATION):
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0

    def p_control(self, w_desired, w_measured, e_sum):
        duty_cycle = min(max(-1, self.Kp * (w_desired - w_measured) + self.Ki * e_sum), 1)

        e_sum = e_sum + (w_desired - w_measured)

        return duty_cycle, e_sum

    def drive(self, v_desired, w_desired, wl, wr):
        wl_desired = v_desired / self.r + self.l * w_desired / 2
        wr_desired = v_desired / self.r - self.l * w_desired / 2

        duty_cycle_l, self.e_sum_l = self.p_control(wl_desired, wl, self.e_sum_l)
        duty_cycle_r, self.e_sum_r = self.p_control(wr_desired, wr, self.e_sum_r)

        return duty_cycle_l, duty_cycle_r

# navigation code
class TentaclePlanner:

    def __init__(self, obstacles, dt=TIMESTEP, steps=5, alpha=1, beta=0.1):

        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.0, 0.5), (0.0, -0.5), (0.1, 1.0), (0.1, -1.0), (0.1, 0.5), (0.1, -0.5), (0.1, 0.0),
                          (0.0, 0.0)]

        self.alpha = alpha
        self.beta = beta

        self.obstacles = obstacles

    # Play a trajectory and evaluate where you'd end up
    def roll_out(self, v, w, goal_x, goal_y, goal_th, x, y, th):

        for j in range(self.steps):
            x = x + self.dt * v * np.cos(th)
            y = y + self.dt * v * np.sin(th)
            th = (th + w * self.dt)

            if (self.check_collision(x,y)):
                return np.inf

        e_th = goal_th - th
        e_th = np.arctan2(np.sin(e_th), np.cos(e_th))

        return self.alpha * ((goal_x - x) ** 2 + (goal_y - y) ** 2) + self.beta * (e_th ** 2)

    def check_collision(self,x,y):

        min_dist = np.min(np.sqrt((x-self.obstacles[:,0])**2 + (y - self.obstacles[:,1])*2))

        if (min_dist < 0):
            return True
        return False

    # Choose trajectory that will get you closest to the goal
    def plan(self, goal_x, goal_y, goal_th, x, y, th):

        costs = []
        for v, w in self.tentacles:
            costs.append(self.roll_out(v, w, goal_x, goal_y, goal_th, x, y, th))

        best_idx = np.argmin(costs)

        return self.tentacles[best_idx]

# create an instantiation of the classes
obstacles = [10,10] # define an obstacle or multiple
robot = DiffDriveRobot(inertia=INERTIA, dt=TIMESTEP, drag=DRAG, wheel_radius=WHEELRADIUS, wheel_sep=WHEELSEPERATION,resolution=ENCODERRESOLUTION)
controller = RobotController(Kp=KP, Ki=KI, wheel_radius=WHEELRADIUS, wheel_sep=WHEELSEPERATION)
planner = TentaclePlanner(obstacles,dt=TIMESTEP,steps=5,alpha=1,beta=1e-5)

#################################################################################################

# test - move 30cm forwards and 30cm to right, obstacle at 15cm, 15cm (like the competition)
# all distances must be in mm or modify starting variables
goal_x = 300
goal_y = 300
goal_th = 0 # not sure what orientation is what number
obstacles = [[145,145],[150,150],[147,147],[145,150],[150,145]] # designed to be kind of like a solid block

for i in range(200):
    v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th)
    duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr)

    # send these duty cycles to the robot motor


# could do any tests that you want, probably good to do some simpler ones...
