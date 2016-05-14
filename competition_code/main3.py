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
red_lower = np.array([0, 106, 210]) # Red color parameters
red_upper = np.array([81, 178, 255])
blue_lower = np.array([97, 50, 160]) # Blue color paramters
blue_upper = np.array([120, 255, 255])
res_var = 300 # Image width
center_range = res_var/10 # Center range to shoot
minArea = center_range * center_range # Minimum area to detect
turn_ratio = 0.7 # Speed reduction for turning
last_seen = "unknown" # Last seen direction of target
last_time = 10 # Last time since target seen
time_out = 5 # Ignore targets seen longer than time_out seconds ago
max_time = 6 # just shoot already
## Command sequence
C_F = 0
C_TL = 1
C_TR = 2
C_REV = 3
C_L = 4
C_R = 5
C_S = 6
C_WORDS = ["C_F", "C_TL", "C_TR", "C_REV", "C_L", "C_R", "C_S", "C_TL"]
C_LIST = [C_F, C_S] #, "C_F", "C_TL", "C_F", "C_TL", "C_F", "C_TL", "C_F", "C_TL"
C_IDX = 0

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
isBump = False


########################

## Comparator pins

left_comp = 9
right_comp = 11

########################

## Bumper pin

bump = 4

########################

## Motor pin

robot = Robot()

# A = PWM mode, B = direction, pwmPin = A input
# rearLeft: A = gray, B = blue, PWM = green
robot.rearLeft = Motor(A=25, B=24, pwmPin=23, duty=60, range=pwm_range)

# rearRight: A = black, B = yellow, PWM = white
robot.rearRight = Motor(A=16, B=21, pwmPin=20, duty=60, range=pwm_range)

# frontLeft: A = white, B = gray, PWM = purple
robot.frontLeft = Motor(A=26, B=19, pwmPin=13, duty=90, range=pwm_range)

# frontRight: A = purple, B = blue, PWM = green
robot.frontRight = Motor(A=22, B=27, pwmPin=15, duty=60, range=pwm_range)

########################

# Hardware PWM - for shooter servo
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

## Shooter Process

class ShooterThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.servo = Servo()
        self.servo.update(30)

    def run(self):
        print "Shooting thread start"
        while not ALLSTOP:
            if 1:
                self.servo.shoot()
                
########################

## Comparator Event - using callbacks instead of thread

class ComparatorThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(left_comp, GPIO.IN)
        GPIO.setup(right_comp, GPIO.IN)

    def run(self):
        print "Comparator thread start"
        GPIO.wait_for_edge(left_com, GPIO.RISING)
        while 1:
            leftEdge = not GPIO.input(left_comp)
            rightEdge = not GPIO.input(right_comp)
            time.sleep(0.2)

def LeftComparatorCb(channel):
    if GPIO.input(left_comp) is 0:
        leftEdge = True
    else:
        leftEdge = False
    print "LEFT EDGE", leftEdge
    

def RightComparatorCb(channel):
    if GPIO.input(right_comp) is 0:
        rightEdge = True
    else:
        rightEdge = False
    print "RIGHT EDGE", rightEdge

########################

## Bumper Event

def BumpCb(channel):
    if GPIO.input(bump) is 0:
        isBump = True
    else:
        isBump = False
    print "BUMP", isBump
    
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
        global last_seen
        global last_time

        print "Camera thread start"
        

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
            target.there, coords, target.where, image = ColorFilter2(image, hsv, [red_lower, blue_lower], [red_upper, blue_upper], (0,255,0), minArea)
            
            if target.there:
                if target.where < center_range and target.where > -center_range:
                    last_seen = "center"
                elif target.where <= center_range:
                    last_seen = "right"
                elif target.where >= -center_range:
                    last_seen = "left"
                last_time = time.clock()
            
            #print "Target", target.there, last_time, last_seen

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
            time.sleep(0.13)
            self.update(43)
            #print "reload"
            time.sleep(1)

    def update(self, angle):
        global pwm
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)
          
################################################################################

### MAIN FUNCTION
robot.forward()
# Camera
cameraThread = CameraThread()
cameraThread.daemon = True
cameraThread.start()

# Shooter
shooterThread = ShooterThread()
shooterThread.daemon = True
shooterThread.start()

# Comparator
GPIO.setup(left_comp, GPIO.IN)
GPIO.setup(right_comp, GPIO.IN)
GPIO.add_event_detect(left_comp, GPIO.BOTH, callback=LeftComparatorCb)
GPIO.add_event_detect(right_comp, GPIO.BOTH, callback=RightComparatorCb)
#compThread = ComparatorThread()
#compThread.daemon = True
#compThread.start()

# Bumper
GPIO.setup(bump, GPIO.IN)
GPIO.add_event_detect(bump, GPIO.FALLING, callback=BumpCb)

#time.sleep(5-time.clock())
#now = time.clock()
#print "Waited", now

try:
    while 1:
		
		# nothing to lose
        if time.clock() > max_time:
            shootNow = True
            robot.turnLeft()
        # stop if about to go off edge
        elif leftEdge and rightEdge or isBump:
            shootNow = False
            robot.reverse(0.5)
            #robot.turnRight
            print "!!!  I'm going to fall !!!"
        # turn right if about to fall off left
        elif leftEdge: 
            shootNow = False
            robot.turnRight()
            print "--> Avoiding left edge"
        # turn left if about to fall off right
        elif rightEdge: 
            shootNow = False
            robot.turnLeft()
            print "<-- Avoiding right edge"
        # shoot & go forward if target seen
        elif target.there and last_seen == "center": 
            shootNow = True
            robot.forward()
            print "** I see you"
        # chase target right
        elif target.there and last_seen =="right": 
            robot.goRight(int(2*target.where/res_var*turn_ratio))
            shootNow = True
            print "* -> Pursuing right",target.where
        # chase target left
        elif target.there and last_seen =="left":
            robot.goLeft(int(2*target.where/res_var*turn_ratio))
            shootNow = True
            print "* <- Pursuing left",target.where
        # last saw target going left
        elif last_seen is "left":
            robot.goLeft(turn_ratio)
            shootNow = False
            print "(<-) Remember left"
        # last saw target going right
        elif last_seen is "right": 
            robot.goRight(turn_ratio)
            shootNow = False
            print "(->) Remember right"
        # no idea, guess??
        elif C_IDX < len(C_LIST): 
            shootNow = True
            nextMove = C_LIST[C_IDX]
            #robot.forward()
            if nextMove is C_L:
                robot.goLeft(turn_ratio)
            elif nextMove is C_R:
                robot.goRight(turn_ratio)
            elif nextMove is C_TL:
                robot.turnLeft()
            elif nextMove is C_TR:
                robot.turnRight()
            elif nextMove is C_F:
                robot.forward()
            elif nextMove is C_R:
                robot.reverse()
            elif nextMove is C_S:
                robot.stop()
            #elif nextMove is C_F:
            #    robot.forward()
            else:
                robot.stop()
            C_IDX += 1
            #print "$ Command sequence", C_IDX, C_WORDS[C_IDX]
        else:
            C_IDX = 0
            #shootNow = True
            nextMove = 8 #randint(0, 10)
            if nextMove < 1:
                robot.turnLeft()
            elif nextMove < 2:
                robot.turnRight()
            elif nextMove < 4:
                robot.goLeft(turn_ratio)
            elif nextMove < 6:
                robot.goRight(turn_ratio)
            elif nextMove < 7:
				robot.stop()
            else:
                robot.forward()
            print "? Exploring", nextMove
        
        # Remove old targets
        if time.clock()-last_time > time_out:
            last_seen = "unknown"
            print "Time out:",time.clock()-last_time
            last_time = time.clock()
            
# If CTRL+C is pressed, exit cleanly:
except KeyboardInterrupt:
    ALLSTOP = True 
    robot.stop()
    GPIO.cleanup() # cleanup all GPIO
    cv2.destroyAllWindows()

