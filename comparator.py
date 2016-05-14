#!/usr/bin/python
from time import sleep
import RPi.GPIO as GPIO

left_comp = 5
right_comp = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_comp, GPIO.IN)
GPIO.setup(right_comp, GPIO.IN)

def LeftComparatorCb(channel):
    leftEdge = not GPIO.input(left_comp)
    print "LEFT EDGE", leftEdge
    

def RightComparatorCb(channel):
    rightEdge = not GPIO.input(right_comp)
    print "RIGHT EDGE", rightEdge

GPIO.add_event_detect(left_comp, GPIO.BOTH, callback=LeftComparatorCb)
GPIO.add_event_detect(right_comp, GPIO.BOTH, callback=RightComparatorCb)

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
