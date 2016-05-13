#!/usr/bin/python
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.IN)
GPIO.setup(6, GPIO.IN)

while 1:
	if not (GPIO.input(5)):
		print "Left"
	if not (GPIO.input(6)):
		print "Right"
	#print "I work"
	sleep(1)
