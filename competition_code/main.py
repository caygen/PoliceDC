#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import threading

########################

## Constants
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
pwm_range = 25
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
red_lower = np.array([124, 0, 210])
red_upper = np.array([180, 255, 255])
blue_lower = np.array([110, 50, 54])
blue_upper = np.array([130, 255, 255])
targetErr = 2

########################
## Globals Variables
ALLSTOP = False
target = ColorObj("target")
redObs = ColorObj("red")
blueObs = ColorObj("blue")
greenObs = ColorObj("green")
yellowObs = ColorObj("yellow")
orangeObs = ColorObj("orange")
shootNow = False
hasTarget_red = False
targetPos_red = []
hasTarget_blue = False
targetPos_blue = []
image = None
hsv = None
image_red = None
image_blue = None

########################

## Motor pinsimshow

robot = Robot()

# A = PWM mode, B = direction, pwmPin = A input
# frontLeft: A = gray, B = blue, PWM = green
robot.frontLeft = Motor(A=25, B=24, pwmPin=23, duty=10, range=pwm_range)

# frontRight: A = black, B = yellow, PWM = white
robot.frontRight = Motor(A=16, B=21, pwmPin=20, duty=10, range=pwm_range)

# rearLeft: A = white, B = gray, PWM = purple
robot.rearLeft = Motor(A=26, B=19, pwmPin=13, duty=10, range=pwm_range)

# rearRight: A = purple, B = blue, PWM = green
robot.rearRight = Motor(A=17, B=27, pwmPin=22, duty=10, range=pwm_range)

########################

# Hardware PWM - not using
#pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
#GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
#pwm = GPIO.PWM(pwmPin, freq)  # Initialize PWM
#pwm.start(dc) # Initial state

########################

# Software PWM
wiringpi.wiringPiSetupGpio()
wiringpi.softPwmCreate(robot.rearLeft.softPWM, robot.rearLeft.val, robot.rearLeft.range)
wiringpi.softPwmCreate(robot.rearRight.softPWM, robot.rearRight.val, robot.rearRight.range)
wiringpi.softPwmCreate(robot.frontLeft.softPWM, robot.frontLeft.val, robot.frontLeft.range)
wiringpi.softPwmCreate(robot.frontRight.softPWM, robot.frontRight.val, robot.frontRight.range)

########################

## Timer
TIMEOUT = 120
startTime = time.time()

########################

## Camera Tuning

def tunerCb_red(x):
    global red_lower
    red_lower[0] = cv2.getTrackbarPos('r_Hlow', 'mask0')
    red_lower[1] = cv2.getTrackbarPos('r_Slow', 'mask0')
    red_lower[2] = cv2.getTrackbarPos('r_Vlow', 'mask0')
    red_upper[0] = cv2.getTrackbarPos('r_Hhi', 'mask0')
    red_upper[1] = cv2.getTrackbarPos('r_Shi', 'mask0')
    red_upper[2] = cv2.getTrackbarPos('r_Vhi', 'mask0')
    return

cv2.namedWindow('mask0', cv2.WINDOW_NORMAL)
cv2.createTrackbar('r_Hlow', 'mask0', red_lower[0], 180, tunerCb_red)
cv2.createTrackbar('r_Slow', 'mask0', red_lower[1], 255, tunerCb_red)
cv2.createTrackbar('r_Vlow', 'mask0', red_lower[2], 255, tunerCb_red)
cv2.createTrackbar('r_Hhi', 'mask0', red_upper[0], 180, tunerCb_red)
cv2.createTrackbar('r_Shi', 'mask0', red_upper[1], 255, tunerCb_red)
cv2.createTrackbar('r_Vhi', 'mask0', red_upper[2], 255, tunerCb_red)

def tunerCb_blue(x):
    global blue_lower
    blue_lower[0] = cv2.getTrackbarPos('b_Hlow', 'mask255')
    blue_lower[1] = cv2.getTrackbarPos('b_Slow', 'mask255')
    blue_lower[2] = cv2.getTrackbarPos('b_Vlow', 'mask255')
    blue_upper[0] = cv2.getTrackbarPos('b_Hhi', 'mask255')
    blue_upper[1] = cv2.getTrackbarPos('b_Shi', 'mask255')
    blue_upper[2] = cv2.getTrackbarPos('b_Vhi', 'mask255')
    return

cv2.namedWindow('mask255', cv2.WINDOW_NORMAL)
cv2.createTrackbar('b_Hlow', 'mask255', blue_lower[0], 180, tunerCb_blue)
cv2.createTrackbar('b_Slow', 'mask255', blue_lower[1], 255, tunerCb_blue)
cv2.createTrackbar('b_Vlow', 'mask255', blue_lower[2], 255, tunerCb_blue)
cv2.createTrackbar('b_Hhi', 'mask255', blue_upper[0], 180, tunerCb_blue)
cv2.createTrackbar('b_Shi', 'mask255', blue_upper[1], 255, tunerCb_blue)
cv2.createTrackbar('b_Vhi', 'mask255', blue_upper[2], 255, tunerCb_blue)

########################

## Camera Processes

class CameraThread (threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)
        ## Camera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        
        # Camera warmup
        time.sleep(0.1)

        # Init red & blue threads
        #redThread = RedFilterThread(red_lower, red_upper, "red")
        #redThread.daemon = True
        #redThread.start()
        #blueThread = BlueFilterThread(blue_lower, blue_upper, "blue")
        #blueThread.daemon = True
        #blueThread.start()
        
    def run(self):
        global ALLSTOP
        global target
        global hasTarget_red
        global targetPos_red
        global hasTarget_blue
        global targetPos_blue
        global hasRedObs
        global hasBlueObs
        global blueObsPos
        global redObsPos

        print "Camera thread is running"

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Grab frame
            #image = cv2.flip(frame.array,0)
    	    big = cv2.flip(frame.array,0)
    	    image = cv2.resize(big, (200, int(big.shape[0]*200/big.shape[1])), interpolation = cv2.INTER_AREA)

    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            hasTarget_red, coords_red, targetPos_red = ColorFilter(image, hsv, red_lower, red_upper, (0,0,255), targetErr)
            hasTarget_blue, coords_blue, targetPos_blue = ColorFilter(image, hsv, blue_lower, blue_upper, (255,0,0), targetErr)

            # Cleanup
            self.rawCapture.truncate(0)

            # Clear obstacles
            if len(targetPos_red) is 0:
                redObs.there = False
                redObs.where = 0
            if len(targetPos_blue) is 0:
                blueObs.there = False
                blueObs.where = 0
            
            # Compare
            targetPosList = list(set(targetPos_red).intersection(targetPos_blue))
            if len(targetPosList) > 0:
                target.where = targetPosList[0]
                target.there = True
                print target
            elif len(targetPos_red) > 0:
                redObs.there = True
                redObs.where = targetPos_red[0]
            elif len(targetPos_blue) > 0:
                blueObs.there = True
                blueObs.where = targetPos_blue[0]
            else:
                target.there = False
                target.where = 0
                #print "No match","Red", targetPos_red,"Blue", targetPos_blue

            if ALLSTOP:
                 break

            cv2.imshow("image", image)
            cv2.imshow("hsv", hsv)
            cv2.waitKey(1)

####################

class RedFilterThread (threading.Thread):
    def __init__(self, lower, upper, id):
    	threading.Thread.__init__(self)
        self.lower = lower
        self.upper = upper
        self.id = id

    def run(self):
        global hasTarget_red
        global targetPos_red
        global image_red
        print "Red thread is running"
        while not ALLSTOP:
            hasTarget_red, coords_red, targetPos_red, image_red = ColorFilter(image, hsv, self.lower, self.upper, (0,0,255))

####################

class BlueFilterThread (threading.Thread):
    def __init__(self, lower, upper, id):
    	threading.Thread.__init__(self)
        self.lower = lower
        self.upper = upper
        self.id = id

    def run(self):
        global hasTarget_blue
        global targetPos_blue
        global image_blue
        print "Blue thread is running"
        while not ALLSTOP:
            hasTarget_blue, coords_blue, targetPos_blue, image_blue = ColorFilter(image, hsv, self.lower, self.upper, (255,0,0))
          
################################################################################

### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
thread = CameraThread()
thread.daemon = True
thread.start()
try:
    while 1:
        
        robot.setAllSpeeds(90)
        robot.forward()
        if target.there and target.where < 10 and target.where > -10:
            robot.stop()
        elif target.there and target.where > 10:
            robot.turnRight()
        elif target.there and target.where < -10:
            robot.turnLeft()

        runTime = 0 #time.time()-startTime
	if (runTime > TIMEOUT):
	    print "timeout:",runTime 
            robot.stop()
            GPIO.cleanup() # cleanup all GPIO
            break

# If CTRL+C is pressed, exit cleanly:
except KeyboardInterrupt: 
    robot.stop()
    GPIO.cleanup() # cleanup all GPIO
    cv2.destroyAllWindows()

