#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import threading
from random import randint

########################

## Constants
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
pwm_range = 25
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
red_lower = np.array([0, 106, 210])
red_upper = np.array([81, 178, 255])
blue_lower = np.array([97, 50, 160])
blue_upper = np.array([120, 255, 255])
targetErr = 2
res_var = 300
center_range = res_var/10
minArea = center_range * center_range

########################
## Globals Variables
ALLSTOP = False
target = ColorObj("target")
#redObs = ColorObj("red")
#blueObs = ColorObj("blue")
#greenObs = ColorObj("green")
#yellowObs = ColorObj("yellow")
#orangeObs = ColorObj("orange")
shootNow = False
image = None
hsv = None
leftEdge = False
rightEdge = False

########################

## Motor pin

robot = Robot()

# A = PWM mode, B = direction, pwmPin = A input
# rearLeft: A = gray, B = blue, PWM = green
robot.rearLeft = Motor(A=25, B=24, pwmPin=23, duty=10, range=pwm_range)

# rearRight: A = black, B = yellow, PWM = white
robot.rearRight = Motor(A=16, B=21, pwmPin=20, duty=10, range=pwm_range)

# frontLeft: A = white, B = gray, PWM = purple
robot.frontLeft = Motor(A=26, B=19, pwmPin=13, duty=10, range=pwm_range)

# frontRight: A = purple, B = blue, PWM = green
robot.frontRight = Motor(A=17, B=27, pwmPin=22, duty=10, range=pwm_range)

########################

# Hardware PWM - not using
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, 50)  # Initialize PWM
pwm.start(dc) # Initial state

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

## Shooter Process

class ShooterThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.servo = Servo()
        self.servo.update(30)

    def run(self):
        print "shooting thread works"
        while not ALLSTOP:
            if 1:
                self.servo.shoot()
                
########################

## Comparator Process

class ComparatorThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        print "Comparator thread is running"
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.IN)
        GPIO.setup(6, GPIO.IN)

    def run(self):
        while 1:
            leftEdge = not GPIO.input(5)
            rightEdge = not GPIO.input(6)
            time.sleep(0.5)


########################

## Camera Processes

class CameraThread (threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)
        # Camera setup
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.camera.vflip = True
        self.camera.hflip = True
        self.camera.video_stabilization = True
        
        # Camera warmup
        time.sleep(0.1)
        
        # Camera configuration
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = 1.5
        self.camera.iso = 100
        self.camera.shutter_speed = 6660

        
    def run(self):
        global ALLSTOP
        global target

        print "Camera thread is running"
        

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Grab frame
    	    big = frame.array
    	    image = cv2.resize(big, (res_var, int(big.shape[0]*res_var/big.shape[1])), interpolation = cv2.INTER_AREA)
            image = image[30:127, :] #img[y:y+h, x:x+w]
            
            # Cleanup
            self.rawCapture.truncate(0)
            
    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Color Filter
            target.there, coords, targetPosList, image = ColorFilter2(image, hsv, [red_lower, blue_lower], [red_upper, blue_upper], (0,255,0), minArea)
            
            # Handle target
            targetPosList = list(set(targetPosList))
            
            if len(targetPosList) > 0:
                 target.where = targetPosList[0]

            if ALLSTOP:
                 break

########################

class Servo:

    def __init__(self, angle=43):
        self.angle = angle

    def shoot(self):
        if shootNow:
            self.update(180)
            print "shot(s) fired"
            time.sleep(0.12)
            self.update(43)
            #print "reload"
            time.sleep(1)

    def update(self, angle):
        global pwm
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)
          
################################################################################

### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
### CALIBRATION ###
cameraThread = CameraThread()
cameraThread.daemon = True
cameraThread.start()

shooterThread = ShooterThread()
shooterThread.daemon = True
shooterThread.start()

compThread = ComparatorThread()
compThread.daemon = True
compThread.start()

try:
    while 1:
        robot.setAllSpeeds(30)
        if leftEdge and rightEdge:
            shootNow = False
            robot.reverse(0.5)
            #robot.turnRight()
            print "Ack! I'm going to fall"
        elif leftEdge:
            shootNow = False
            robot.turnRight()
            print "Avoiding left edge"
        elif rightEdge:
            shootNow = False
            robot.turnLeft()
            print "Avoiding right edge"
        elif target.there and target.where < center_range and target.where > -center_range:
            shootNow = True
            robot.forward()
            print "I see you"
        elif target.there and target.where < center_range:
            robot.goRight()
            shootNow = False
            print "Pursuing right",target.where
        elif target.there and target.where > -center_range:
            robot.goLeft()
            shootNow = False
            print "Pursuing left",target.where
        else:
            shootNow = False
            nextMove = 2 #randint(0, 5)
            if nextMove < 1:
                robot.turnLeft()
            elif nextMove < 2:
                robot.turnRight()
            else:
                #robot.forward()
                robot.stop()
            print "Exploring", nextMove
		

# If CTRL+C is pressed, exit cleanly:
except KeyboardInterrupt:
    ALLSTOP = True 
    robot.stop()
    GPIO.cleanup() # cleanup all GPIO
    cv2.destroyAllWindows()

