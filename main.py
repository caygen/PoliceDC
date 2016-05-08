#!/usr/bin/python

# External module imports
import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import threading

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
pwm_range = 25

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

## Motor pins

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

hasTarget = False
target = 0

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
        
    def run(self):
        global target

        #print "Camera thread is running"
        red_lower = np.array([124, 0, 210])
        red_upper = np.array([180, 255, 255])
        blue_lower = np.array([110, 50, 54])
        blue_upper = np.array([130, 255, 255])
        Blow = 210
        Bhi = 255
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Grab frame
    	    image = cv2.flip(frame.array,0)
    	    image = cv2.resize(image, (100, int(image.shape[0]*100/image.shape[1])), interpolation = cv2.INTER_AREA)
    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    	    # Filter by HSV color
    	    mask_red = cv2.inRange(hsv, red_lower, red_upper)
            mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
    	    
            _, contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
            _, contours, _ = cv2.findContours(mask_red, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    	    #print "Found", len(contours), "red contours"
            #print "Found", len(contours_blue), "blue contours"
            
            # Find centers of blobs and draw blue circle
            index = -1
    	    maxArea = 0
    	    for i in range(len(contours)):
	        area = cv2.contourArea(contours[i])
                if area > maxArea:
                    index = i
                    maxArea = area
            red_found = False
            blue_found = False
    	    coord = (0, 0)
            coord_blue = (0, 0)
            if index > -1:
            #for i in range(0, min(len(contours),5)): # only draw top 5
                cv2.drawContours(image, [contours[index]], 0, (0, 0, 255))
                moments = cv2.moments(contours[index])
                coord = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
                #centers.append(coord)
                if (coord[0] is not 0 and coord[1] is not 0):
	             cv2.circle(image, coord, 3, (0, 0, 255), -1)
                     print "Red center at", coord
                     #height, width = image.shape[:2]
                     #target = width/2 - coord[0]
                     red_found = True
            if len(contours_blue) > 0:
               cv2.drawContours(image, [contours_blue[0]], 0, (255, 0, 0))
               moments = cv2.moments(contours_blue[0])
               coord_blue = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
               if (coord_blue[0] is not 0 and coord[1] is not 0):
                     cv2.circle(image, coord_blue, 3, (255, 0, 0), -1)
                     #print "Blue center at", coord_blue
                     blue_found = True
            if blue_found and red_found:
               if (abs(coord[0]-coord_blue[0]) < 10):
                   height, width = image.shape[:2]
                   target = width/2 - (coord[0]+coord_blue[0])/2
                   print "target", target
                   hasTarget = true
            else:
               hasTarget = false
            # Visualize
            cv2.imshow("Red", image)    
    
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                #print image
                break

            # Cleanup
            self.rawCapture.truncate(0)
    
### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
thread = CameraThread()
thread.daemon = True
thread.start()
try:
    while 1:
        robot.setSpeeds(90)
        robot.forward()
        if hasTarget and target < 10 and target > -10:
            robot.stop()
        elif hasTarget and target > 10:
            #robot.setSpeeds(30)
            robot.turnRight(1)
        elif hasTarget and target < -10:
            robot.turnLeft(1)
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

