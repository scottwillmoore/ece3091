# Steps:

# 1) Plug in the camera
# 2) type sudo raspi-config
# 3) Interfacing options
# 4) Click on camera
# 5) Re-boot the rapberry pi
# 6) Go to IDLE


#

# Just for testing
from picamera import Picamera
import time

camera = Picamera()
# Change resolution
camera.resolution(640, 480)
# Flip a camera
camera.vflip = True

camera.start_preview()
time.sleep(2)

# save the photo on the raspberry pi
camera.capture("test.jpg")






# Record a video
camera.start_recording("my_video.h264")
time.sleep(5)
camera.stop_recording


# For playing the video
# Open rasbperry pi terminal
# type "omxplayer"
# type "cd Desktop"
# type omxplayer my_video.h264 and hit enter 
# The video should pop up on the screen





# Extra sources
# Start a preview for 10 seconds with the default settings:
import time
import picamera

camera = picamera.PiCamera()
try:
    camera.start_preview()
    time.sleep(10)
    camera.stop_preview()
finally:
    camera.close()
    
# Close used to clean up resources

# Use this for "live" preview is running, here we are changing brightness while the camera is capturing
with picamera.PiCamera() as camera:
    camera.start_preview()
    try:
        for i in range(100):
            camera.brightness = i
            time.sleep(0.2)
    finally:
        camera.stop_preview()

# Setting resolution

with picamera.PiCamera() as camera:
    camera.resolution = (1280, 720)
    camera.start_preview()`
    camera.exposure_compensation = 2
    camera.exposure_mode = 'spotlight'
    camera.meter_mode = 'matrix'
    camera.image_effect = 'gpen'
    # Give the camera some time to adjust to conditions
    time.sleep(2)
    camera.capture('example.jpg')
    camera.stop_preview()
    
    
# Capturing an image in raw RGB format
with picamera.PiCamera() as camera:
    camera.resolution = (1024, 768)
    camera.start_preview()
    time.sleep(2)
    camera.capture('image.data', 'rgb')
    
    
    
 # Recording video to a file
 
stream = io.BytesIO()
with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.start_recording(stream, quantization=23)
    camera.wait_recording(15)
    camera.stop_recording()
    
    
    
 # Recording full-resolution video

    
with picamera.PiCamera() as camera:
    camera.resolution = (2592, 1944)
    camera.start_recording('full_res.h264', resize=(1024, 768))
    camera.wait_recording(60)
    camera.stop_recording()
