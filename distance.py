#!/usr/bin/python
from time import sleep
import RPi.GPIO as GPIO
import wiringpi

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.IN)

while 1:
	print wiringpi.analogRead(2)
	
	
