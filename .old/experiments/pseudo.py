# code to control the motor PWM
# change speed
# change direction
# independent for each wheel


# code for the camera/computer vision
# reading the camera data
# computer vision
# output for debugging?
# output result of the computer vision processing


# code for the rotary encoder
# read the speed of the motors
# compare the speed of real/predicted
# correct speed


# code for the servo PWM
# change speed
# choose angle
# prepare to hit at the angle with enough power
# close fully after hit balls


# code for the ultrasonic sensor
# code to read distance
# code to tell us if we are too close
# code to tell us we are fine


# main()

# potentially a warm up/alignment test
# calibration??


# loop that runs continuously
# each subsystem runs
# take in all the input every so much time (fraction of second)

# Step 1: read the camera
# - Can I see a ball?
# - What do I do if I can see a ball?
# - What do I do if I can't see a ball?

# Step 2: I can see a ball
# - Stop scanning whole arena
# - How close is the ball to me?
# - Use mathematics to figure out the x/y location is
# - Make a plan to get to the ball
# - What do I do if I see multiple balls?
# - Figure out what the closest one is

# Step 3: I cannot see a ball
# - Where am I in the arena from ultrasonic sensors?
# - Rotate away from wall, or default 45 left