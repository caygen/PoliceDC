#!/usr/bin/python
from time import sleep
import RPi.GPIO as GPIO

left_comp = 9
right_comp = 11

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
	
	print "left",GPIO.input(9)
	print "right",GPIO.input(11)
	#if GPIO.input(9) is 0:
		#print "left"
	#if GPIO.input(11) is 0:
		#print "right"
		
	#if left is 0:
		#print "Left off"
	#else:
		#print "Left safe"
	#if right is 0:
		#print "Right off"
	#else:
		#print "Right safe"
	#sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup() # cleanup all GPIO
