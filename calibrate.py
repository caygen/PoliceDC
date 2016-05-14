#!/usr/bin/python

import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import threading

###
# RED (0, 106, 210), (81, 178, 255)
# BLUE (97, 50, 160), (120, 255, 255) - Wed
# exposure 6660
# Blue obstacle values: (107, 107-133, 140-210)
# Target values: (

red_lower = np.array([0, 106, 210])
red_upper = np.array([81, 178, 255])
blue_lower = np.array([97, 50, 160])
blue_upper = np.array([120, 255, 255])
res_var = 300 # image width
center_range = res_var/10
min_area = 500

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

########################

## Constants

dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
pwm_range = 25
targetErr = 2
center_range = 20

########################

## Globals Variables

ALLSTOP = False
target = ColorObj("target")
redObs = ColorObj("red")
blueObs = ColorObj("blue")
greenObs = ColorObj("green")
yellowObs = ColorObj("yellow")
orangeObs = ColorObj("orange")
shootNow = False
hasTarget_red = False
targetPos_red = []
hasTarget_blue = False
targetPos_blue = []
image = None
hsv = None
image_red = None
image_blue = None
cor_x = 0
cor_y = 0

########################

## Camera Tuning

def tunerCb_red(x):
    global red_lower
    red_lower[0] = cv2.getTrackbarPos('r_Hlow', 'mask0')
    red_lower[1] = cv2.getTrackbarPos('r_Slow', 'mask0')
    red_lower[2] = cv2.getTrackbarPos('r_Vlow', 'mask0')
    red_upper[0] = cv2.getTrackbarPos('r_Hhi', 'mask0')
    red_upper[1] = cv2.getTrackbarPos('r_Shi', 'mask0')
    red_upper[2] = cv2.getTrackbarPos('r_Vhi', 'mask0')
    return

#cv2.namedWindow('mask0', cv2.WINDOW_NORMAL)
#cv2.createTrackbar('r_Hlow', 'mask0', red_lower[0], 180, tunerCb_red)
#cv2.createTrackbar('r_Slow', 'mask0', red_lower[1], 255, tunerCb_red)
#cv2.createTrackbar('r_Vlow', 'mask0', red_lower[2], 255, tunerCb_red)
#cv2.createTrackbar('r_Hhi', 'mask0', red_upper[0], 180, tunerCb_red)
#cv2.createTrackbar('r_Shi', 'mask0', red_upper[1], 255, tunerCb_red)
#cv2.createTrackbar('r_Vhi', 'mask0', red_upper[2], 255, tunerCb_red)

def tunerCb_blue(x):
    global blue_lower
    blue_lower[0] = cv2.getTrackbarPos('b_Hlow', 'mask1')
    blue_lower[1] = cv2.getTrackbarPos('b_Slow', 'mask1')
    blue_lower[2] = cv2.getTrackbarPos('b_Vlow', 'mask1')
    blue_upper[0] = cv2.getTrackbarPos('b_Hhi', 'mask1')
    blue_upper[1] = cv2.getTrackbarPos('b_Shi', 'mask1')
    blue_upper[2] = cv2.getTrackbarPos('b_Vhi', 'mask1')
    return

cv2.namedWindow('mask1', cv2.WINDOW_NORMAL)
cv2.createTrackbar('b_Hlow', 'mask1', blue_lower[0], 180, tunerCb_blue)
cv2.createTrackbar('b_Slow', 'mask1', blue_lower[1], 255, tunerCb_blue)
cv2.createTrackbar('b_Vlow', 'mask1', blue_lower[2], 255, tunerCb_blue)
cv2.createTrackbar('b_Hhi', 'mask1', blue_upper[0], 180, tunerCb_blue)
cv2.createTrackbar('b_Shi', 'mask1', blue_upper[1], 255, tunerCb_blue)
cv2.createTrackbar('b_Vhi', 'mask1', blue_upper[2], 255, tunerCb_blue)


########################################

def ColorFilter2(image, hsv, lowerList, upperList, color):
    """ ColorFilter for multiple range lists """
    if image is None or hsv is None:
        return False, [], [], image
    height, width = image.shape[:2]

    totalMask = np.ones((height, width), np.uint8)
    # Filter by HSV color
    mask = []
    for i in range(len(lowerList)):
        mask = cv2.inRange(hsv, lowerList[1], upperList[1])
        #mask = cv2.dilate(mask, np.ones((11, 11)))
        res = cv2.bitwise_and(totalMask, mask, totalMask)
        
        ### CALIBRATION ###
        cv2.imshow("mask"+str(1),mask)
    
    cv2.waitKey(1)
    maskImg = np.zeros((height,width,3), np.uint8)
    res = cv2.bitwise_and(image,image,maskImg,mask=totalMask)
    cv2.imshow("total", maskImg)
 
    found, coords, targets = findContours(image, totalMask, color)
    
    return found, coords, targets, image

########################################

def findContours(image, mask, color, n=3, err=None):
    """ find n contours in mask and draw on image """
    
    height, width = image.shape[:2]
    print "height", height, "width", width

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
        if area > maxArea and area > min_area:
            index = i
            maxArea = area
    print maxArea
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

## Camera Processes

def mouseCb(event,x,y,flags,param):
    global cor_x
    global cor_y
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cor_x = x
        cor_y = y

class CameraThread (threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)
    	
        # Camera Settings
        self.camera = PiCamera()
        self.camera.resolution = (600, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(600, 480))
        self.camera.vflip = True
        self.camera.hflip = True
        self.camera.video_stabilization = True
        
        
        # Camera warmup
        time.sleep(0.1)
        
        # Camera modes
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'
        self.camera.shutter_speed = 6660
        self.camera.iso = 100
        self.camera.awb_gains = 1.5

        # MouseCb
        self.x = 0
        self.y = 0
        
    def run(self):
        global ALLSTOP
        global target
        global hasTarget_red
        global targetPos_red
        global hasTarget_blue
        global targetPos_blue
        global hasRedObs
        global hasBlueObs
        global blueObsPos
        global redObsPos
        global cor_x
        global cor_y
        
        print "Camera thread is running"
        
        ### CALIBRATION ###
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("mask0", cv2.WINDOW_NORMAL)
        cv2.namedWindow("mask1", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("image", mouseCb)
        
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Grab frame
    	    big = frame.array
    	    image = cv2.resize(big, (res_var, int(big.shape[0]*res_var/big.shape[1])), interpolation = cv2.INTER_AREA)
    	    res_var_b = int(res_var/200)
            image = image[res_var_b*30:res_var_b*127, :] #img[y:y+h, x:x+w]
            # Cleanup
            self.rawCapture.truncate(0)
            
    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    	    
    	    # Print HSV Mouse Values
            s = hsv[cor_y, cor_x]
            print "(", cor_x, cor_y, ") H:",s[0],"S:",s[1],"V:",s[2]
            cv2.putText(image,"H: "+str(s[0])+" S: "+str(s[1])+" V: "+str(s[2]),(cor_x,cor_y), cv2.FONT_HERSHEY_SIMPLEX, 0.2, 255)
    	    cv2.circle(image,(cor_x,cor_y),5,(255,0,0))
            target.there, coords, targetPosList, image = ColorFilter2(image, hsv, [red_lower, blue_lower], [red_upper, blue_upper], (0,255,0))
            
            targetPosList = list(set(targetPosList))
            
            #if target.there and target.where < center_range and target.where > -center_range:
                #print "I see you at", targetPosList
            #else:
                #print "Target",target.there,"at",targetPosList, "(Outside",center_range,")"
            
            if len(targetPosList) > 0:
                 target.where = targetPosList[0]

            if ALLSTOP:
                 break

            cv2.imshow("image", image)
            cv2.waitKey(1)

cameraThread = CameraThread()
cameraThread.daemon = True
cameraThread.start()
try:
	while 1:
		pass
except KeyboardInterrupt: 
    ALLSTOP = True
    cv2.destroyAllWindows()
