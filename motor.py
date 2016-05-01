#!/usr/bin/python

# External module imports
import RPi.GPIO as GPIO
import time
from PoliceLibrary import *

### FUNCTIONS
class Motor:
    def __init__(self, A, B):
        self.A = A
        self.B = B
        self.mode = "brake"
        GPIO.setup(A, GPIO.OUT)
        GPIO.setup(B, GPIO.OUT)
        self.brake()
    
    def __str__(self):
        return "Pin A: " + str(self.A) + ", Pin B: " + str(self.B) + ", Mode: " + self.mode

    def cw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "cw - high, low"

    def ccw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "ccw - high, high"

    def brake(self):
        GPIO.output(self.A, GPIO.LOW)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"

class Robot:
    def __init__(self):
        self.frontLeft = None
        self.frontRight = None
        self.rearLeft = None
        self.rearRight = None

    def forward(self, t=5):
        print "MOVING FORWARD"
        self.frontLeft.ccw()
        self.frontRight.cw()
        self.rearLeft.ccw()
        self.rearRight.cw()
        time.sleep(t)

    def turnLeft(self, t=5):
        print "TURNING LEFT"
        self.frontLeft.cw()
        self.frontRight.cw()
        self.rearLeft.cw()
        self.rearRight.cw()
        time.sleep(t)

    def turnRight(self, t=5):
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
   

### SETUP

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

# PWM setup
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, freq)  # Initialize PWM
pwm.start(dc) # Initial state

# Motor pins
m1 = Motor(23, 24)
m2 = Motor(20, 21)
m3 = Motor(13, 19)
m4 = Motor(22, 27)

robot = Robot()
robot.frontLeft = m1
robot.frontRight = m2
robot.rearLeft = m3
robot.rearRight = m4    

# Timer
TIMEOUT = 120
startTime = time.time()

### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
try:
    while 1:
	robot.frontLeft.cw()
	robot.frontRight.cw()
	robot.rearLeft.cw()
	robot.rearRight.cw()
	time.sleep(5)
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
    pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO

