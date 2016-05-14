#!/usr/bin/python

# External module imports
import RPi.GPIO as GPIO
import time
import wiringpi
from PoliceLibrary import *

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
robot.rearLeft = Motor(A=25, B=24, pwmPin=23, duty=10, range=range)

# frontRight: A = black, B = yellow, PWM = white
robot.rearRight = Motor(A=16, B=21, pwmPin=20, duty=10, range=range)

# rearLeft: A = white, B = gray, PWM = purple
robot.frontLeft = Motor(A=26, B=19, pwmPin=13, duty=10, range=range)

# rearRight: A = purple, B = blue, PWM = green
robot.frontRight = Motor(A=17, B=27, pwmPin=22, duty=10, range=range)

## PWM setup

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

### MAIN FUNCTION
print("Here we go! Press CTRL+C to exit")
try:
    while 1:
        robot.setAllSpeeds(30)
        robot.forward()
        #robot.turnLeft()
        #robot.turnRight()
        #robot.backward()
        
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

