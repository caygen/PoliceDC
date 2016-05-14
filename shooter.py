#!/usr/bin/python

import RPi.GPIO as GPIO
import time

start = 45
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 50)
pwm.start(10)

class Servo:

    def __init__(self, angle=start):
        self.angle = angle

    def shoot(self):
        self.update(180)
	print "shot(s) fired"
        time.sleep(0.13)
        self.update(start)
        print "reload"
        time.sleep(0.1)

    def update(self, angle):
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)

s = Servo(start)
s.update(start)
while 1:
    s.shoot()
    time.sleep(1)
