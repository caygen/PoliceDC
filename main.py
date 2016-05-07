#!/usr/bin/python

# External module imports
import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *
from picamera.array import PiRGBArray
from picamer import PiCamera
import cv2
import numpy as np
import thread

### FUNCTIONS
class Motor:
    def __init__(self, A, B, pwmPin, duty=95, range=25):
        self.A = A
        self.B = B
        self.mode = "brake"
	self.duty = duty
	self.val = int(duty/100.0*range)
	self.range = range
	self.softPWM = pwmPin
        GPIO.setup(A, GPIO.OUT)
        GPIO.setup(B, GPIO.OUT)
        self.brake()
    
    def __str__(self):
        return "Pin A: " + str(self.A) + ", Pin B: " + str(self.B) + ", Mode: " + self.mode

    def cw(self):
	self.set_speed(self.duty)
        GPIO.output(self.A, GPIO.HIGH)
	GPIO.output(self.B, GPIO.LOW)
        self.mode = "cw"

    def ccw(self):
	self.set_speed(self.duty)
	GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "ccw"

    def brake(self):
	self.set_speed(0)
	GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"
    
    def set_speed(self, speed):
	self.val = int(speed/100.0*self.range)
	wiringpi.softPwmWrite(self.softPWM, self.val)

    def set_duty(self, duty):
        self.duty = duty

class Robot:
    def __init__(self):
        self.frontLeft = None
        self.frontRight = None
        self.rearLeft = None
        self.rearRight = None

    def backward(self, t=2):
        print "MOVING BACKWARD"
        self.frontLeft.cw()
        self.frontRight.ccw()
        self.rearLeft.cw()
        self.rearRight.ccw()
        time.sleep(t)

    def forward(self, t=2):
        print "MOVING FORWARD"
        self.frontLeft.ccw()
        self.frontRight.cw()
        self.rearLeft.ccw()
        self.rearRight.cw()
        time.sleep(t)

    def turnLeft(self, t=2):
        print "TURNING LEFT"
        self.frontLeft.cw()
        self.frontRight.cw()
        self.rearLeft.cw()
        self.rearRight.cw()
        time.sleep(t)

    def turnRight(self, t=2):
        print "TURNING RIGHT"
        self.frontLeft.ccw()
        self.frontRight.ccw()
        self.rearLeft.ccw()
        self.rearRight.ccw()
        time.sleep(t)
    
    def stop(self, t=1):
        print "STOPPING"
        self.frontLeft.brake()
        self.frontRight.brake()
        self.rearLeft.brake()
        self.rearRight.brake()
        time.sleep(t)

    def setSpeeds(self, speed):
        self.frontLeft.set_duty(speed)
        self.frontRight.set_duty(speed)
        self.rearLeft.set_duty(speed)
        self.rearRight.set_duty(speed)

### SETUP

## Constants
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
range = 25

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

## Motor pins

robot = Robot()

# A = PWM mode, B = direction, pwmPin = A input
# frontLeft: A = gray, B = blue, PWM = green
robot.frontLeft = Motor(A=25, B=24, pwmPin=23, duty=10, range=range)

# frontRight: A = black, B = yellow, PWM = white
robot.frontRight = Motor(A=16, B=21, pwmPin=20, duty=10, range=range)

# rearLeft: A = white, B = gray, PWM = purple
robot.rearLeft = Motor(A=26, B=19, pwmPin=13, duty=10, range=range)

# rearRight: A = purple, B = blue, PWM = green
robot.rearRight = Motor(A=17, B=27, pwmPin=22, duty=10, range=range)

# Hardware PWM
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, freq)  # Initialize PWM
pwm.start(dc) # Initial state

# Software PWM
wiringpi.wiringPiSetupGpio()
wiringpi.softPwmCreate(robot.rearLeft.softPWM, robot.rearLeft.val, robot.rearLeft.range)
wiringpi.softPwmCreate(robot.rearRight.softPWM, robot.rearRight.val, robot.rearRight.range)
wiringpi.softPwmCreate(robot.frontLeft.softPWM, robot.frontLeft.val, robot.frontLeft.range)
wiringpi.softPwmCreate(robot.frontRight.softPWM, robot.frontRight.val, robot.frontRight.range)

## Timer
TIMEOUT = 120
startTime = time.time()



class CameraThread (threading.Thread):
    def __init__(self, threadID, name):
    	threading.Thread.__init__(self)
    	## Camera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        camera.framerate = 32
        self.rawCapture = PiRGBArray(camera, size=(640, 480))

        # Camera warmup
        time.sleep(0.1)

    def run(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr"for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # Grab frame
    	    image = frame.array
    	    image = cv2.resize(image, (100, int(image.shape[0]*100/image.shape[1])), interpolation = cv2.INTER_AREA)
    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    	    # Filter by HSV color & blur mask
    	    mask = cv2.inRange(hsv, lower, upper)
    	    _, contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    	    print "Found", len(contours), "contours"
    
            # Find centers of blobs and draw blue circle
    	    centers = []
    
	    index = -1
    	    maxArea = 0
    	    for i in range(len(contours)):
	        area = cv2.contourArea(contours[i])
                if area > maxArea:
                    index = i
                    maxArea = area
    	    if index > -1:
            #for i in range(0, min(len(contours),5)): # only draw top 5
                cv2.drawContours(image, [contours[index]], 0, (0, 255, 0))
                moments = cv2.moments(contours[index])
                coord = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
                centers.append(coord)
                if (coord[0] is not 0 and coord[1] is not 0):
	             cv2.circle(image, centers[-1], 3, (255, 0, 0), -1)
                     print "Center at", coord
    
            # Visualize
            cv2.imshow("Frame", image)
            cv2.imshow("hsv", hsv)
            cv2.imshow("hsvMask", mask)    
    
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print image
                break

            # Cleanup
            rawCapture.truncate(0)
    
### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
thread = CameraThread()
thread.start()
try:
    while 1:
        robot.setSpeeds(30)
        robot.forward(1)
        robot.turnLeft(1)
        robot.turnRight(1)
        robot.backward(1)
#FR, FL = reverse; RR, RL = normal # cw is faster
	#robot.rearLeft.ccw()
	#print robot.rearRight.mode
	#time.sleep(5)
        #robot.forward()
        #robot.stop(5)
        #robot.turnLeft()
        #robot.stop(5)
        #robot.turnRight()
        runTime = 0 #time.time()-startTime
	if (runTime > TIMEOUT):
	    print "timeout:",runTime 
            robot.stop()
            pwm.stop() # stop PWM
            PIO.cleanup() # cleanup all GPIO
            break


# If CTRL+C is pressed, exit cleanly:
except KeyboardInterrupt: 
    robot.stop()
    pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO

