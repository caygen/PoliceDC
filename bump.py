#!/usr/bin/python
from time import sleep
import RPi.GPIO as GPIO

bump = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(bump, GPIO.IN)

def BumpCb(channel):
    isBump = not GPIO.input(bump)
    print "BUMP", isBump
    
GPIO.add_event_detect(bump, GPIO.FALLING, callback=BumpCb)

print "started"
while 1:
	pass
	#left = GPIO.input(5)
	#right = GPIO.input(6)
	#if left is 0:
		#print "Left off"
	#else:
		#print "Left safe"
	#if right is 0:
		#print "Right off"
	#else:
		#print "Right safe"
	#sleep(1)
