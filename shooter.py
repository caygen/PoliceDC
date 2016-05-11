#!/usr/bin/python

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 100)
pwm.start(30)

class Servo:

    def __init__(self, angle=30):
        self.angle = angle

    def shoot(self):
        self.update(180)
	print "shot(s) fired"
        time.sleep(0.12)
        self.update(30)
        print "reload"
        time.sleep(0.1)

    def update(self, angle):
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)

s = Servo(90)
s.update(90)
while 1:
    s.shoot()
    time.sleep(2)