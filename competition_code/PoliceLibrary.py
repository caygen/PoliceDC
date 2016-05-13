import RPi.GPIO as GPIO
import time
import wiringpi
import cv2
import numpy as np

########################################

### FUNCTIONS

def t_sleep(t):
    """ non-blocking time.sleep function """ 
    for i in range(t/0.1):
        time.sleep(0.1)

####################

def ColorFilter(image, hsv, lower, upper, color, err):
    """ masks hsv image by hsv color range & draws contours on original image """
    if image is None or hsv is None:
        return False, [], [], image
    height, width = image.shape[:2]

    # Filter by HSV color
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.dilate(mask, np.ones((11, 11)))
    maskImg = np.zeros((height,width,3), np.uint8)
    res = cv2.bitwise_and(image,image,maskImg,mask=mask)
    
    ### CALIBRATION ###
    cv2.imshow("mask"+str(color[0]==255),maskImg)
    cv2.waitKey(1)
    
    found, coords, targets = findContours(image, mask, color, err)

    return found, coords, targets

########################################

def ColorFilter2(image, hsv, lowerList, upperList, color):
    """ ColorFilter for multiple range lists """
    if image is None or hsv is None:
        return False, [], [], image
    height, width = image.shape[:2]

    totalMask = np.ones((height, width), np.uint8)
    # Filter by HSV color
    for i in range(len(lowerList)):
        mask = cv2.inRange(hsv, lowerList[i], upperList[i])
        mask = cv2.dilate(mask, np.ones((11, 11)))
        res = cv2.bitwise_and(totalMask, mask, totalMask)
        
        ### CALIBRATION ###
        #cv2.imshow("mask"+str(i),mask)
        #cv2.waitKey(1)

    maskImg = np.zeros((height,width,3), np.uint8)
    res = cv2.bitwise_and(image,image,maskImg,mask=totalMask)
 
    found, coords, targets = findContours(image, totalMask, color)
    
    return found, coords, targets, image

########################################

def findContours(image, mask, color, n=3, err=None):
    """ find n contours in mask and draw on image """
    
    height, width = image.shape[:2]

    # Find contours
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    
    # Find centers of blobs and draw blue circle
    index = -1
    maxArea = 0
            
    found = False
    coords = []
    targets = []

    for i in range(len(contours)):
	area = cv2.contourArea(contours[i])
        if area > maxArea:
            index = i
            maxArea = area

    # Draw first n contours
    #for i in range(0, min(len(contours),n)):
    if index > -1:
        cv2.drawContours(image, [contours[index]], 0, color)
        moments = cv2.moments(contours[index])
        coord = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
        if (coord[0] is not 0 and coord[1] is not 0):
            cv2.circle(image, coord, 3, color, -1)
            coords += [coord]
            exact = int(width/2 - coord[0])
            if err is None:
                targets += [exact]
            else:
                targets += [x+exact for x in range(-err, err+1)] # exact +/- err
            found = True

    return found, coords, targets

########################################

### CLASSES

########################################

class ColorObj:
    def __init__(self, color=""):
        self.where = 0
        self.there = False
        self.color = color
    def __str__(self):
        if self.there:
            return self.color + " object at " + str(self.where)
        else:
            return "no " + self.color + " objects"

########################################

class Motor:
    """ class to handle individual motors """
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

    def cw(self):
	self.set_speed(self.duty)
        GPIO.output(self.A, GPIO.HIGH)
	GPIO.output(self.B, GPIO.LOW)
        self.mode = "cw"

    def ccw(self):
	self.set_speed(self.duty)
	GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "ccw"

    def brake(self):
	self.set_speed(0)
	GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"
    
    def set_speed(self, speed):
	self.val = int(speed/100.0*self.range)
	wiringpi.softPwmWrite(self.softPWM, self.val)

    def set_duty(self, duty):
        self.duty = duty

########################

class Robot:
    """ class for high level robot control """
    def __init__(self):
        self.frontLeft = None
        self.frontRight = None
        self.rearLeft = None
        self.rearRight = None
        self.action = None
    
    def __str__(self):
        return self.action

    def backward(self, t=0.5):
        print "MOVING BACKWARD"
        self.action = "backward"
        self.frontLeft.cw()
        self.frontRight.ccw()
        self.rearLeft.cw()
        self.rearRight.ccw()
        time.sleep(t)

    def forward(self, t=0.5):
        print "MOVING FORWARD"
        self.action = "forward"
        self.frontLeft.ccw()
        self.frontRight.cw()
        self.rearLeft.ccw()
        self.rearRight.cw()
        time.sleep(t)

    def turnLeft(self, t=0.5):
        print "TURNING LEFT"
        self.action = "left"
        self.frontLeft.cw()
        self.frontRight.cw()
        self.rearLeft.cw()
        self.rearRight.cw()
        time.sleep(t)

    def turnRight(self, t=0.5):
        print "TURNING RIGHT"
        self.action = "right"
        self.frontLeft.ccw()
        self.frontRight.ccw()
        self.rearLeft.ccw()
        self.rearRight.ccw()
        time.sleep(t)
    
    def stop(self, t=0.5):
        print "STOPPING"
        self.action = "stop"
        self.frontLeft.brake()
        self.frontRight.brake()
        self.rearLeft.brake()
        self.rearRight.brake()
        time.sleep(t)

    def setAllSpeeds(self, speed):
        self.frontLeft.set_duty(speed)
        self.frontRight.set_duty(speed)
        self.rearLeft.set_duty(speed)
        self.rearRight.set_duty(speed)

    def setSpeeds(self, (fl, fr, rl, rr)):
        self.frontLeft.set_duty(fl)
        self.frontRight.set_duty(fr)
        self.rearLeft.set_duty(rl)
        self.rearRight.set_duty(rr)
