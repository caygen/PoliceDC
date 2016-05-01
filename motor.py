#!/usr/bin/python

# External module imports
import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *

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

    def ccw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "ccw"

    def cw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "cw"

    def brake(self):
        GPIO.output(self.A, GPIO.LOW)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"
    
    def set_speed(self, duty):
	self.duty = duty
	self.val = int(duty/100.0*self.range)
	wiringpi.softPwmWrite(self.softPWM, self.val)

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

## Constants
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
range = 25

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

## Motor pins

robot = Robot()

# frontLeft: A = gray, B = blue, PWM = green
robot.frontLeft = Motor(A=23, B=24, pwmPin=25, duty=50, range=range)

# frontRight: A = black, B = yellow, PWM = white
robot.frontRight = Motor(A=20, B=21, pwmPin=16, duty=50, range=range)

# rearLeft: A = white, B = gray, PWM = purple
robot.rearLeft = Motor(A=13, B=19, pwmPin=26, duty=50, range=range)

# rearRight: A = purple, B = blue, PWM = green
robot.rearRight = Motor(A=22, B=27, pwmPin=17, duty=50, range=range)

## PWM setup

# Hardware PWM
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, freq)  # Initialize PWM
pwm.start(dc) # Initial state

# Software PWM
wiringpi.wiringPiSetupGpio()
wiringpi.softPwmCreate(m1.softPWM, m1.val, m1.range)
print m1.val
wiringpi.softPwmCreate(m2.softPWM, m2.val, m2.range)
wiringpi.softPwmCreate(m3.softPWM, m3.val, m3.range)
wiringpi.softPwmCreate(m4.softPWM, m4.val, m4.range)

## Timer
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

