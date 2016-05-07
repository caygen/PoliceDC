#!/usr/bin/python

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# Camera Setup
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Camera warmup
time.sleep(0.1)

# Set color filter

# blue - bgr 
#lower = np.array([110, 50, 50])
#upper = np.array([130, 255, 255])

# white
#lower = np.array([0, 0, 200])
#upper = np.array([180, 255, 255])

#red
#lower = np.array([121, 0, 200])
#upper = np.array([180, 255, 255])

# red bright
lower = np.array([124, 0, 210])
upper = np.array([180, 255, 255])
Blow = 210
Bhi = 255

def tunerCb(x):
    global lower
    lower[0] = cv2.getTrackbarPos('Hlow', 'hsv')
    lower[1] = cv2.getTrackbarPos('Slow', 'hsv')
    lower[2] = cv2.getTrackbarPos('Vlow', 'hsv')
    upper[0] = cv2.getTrackbarPos('Hhi', 'hsv')
    upper[1] = cv2.getTrackbarPos('Shi', 'hsv')
    upper[2] = cv2.getTrackbarPos('Vhi', 'hsv')
    Bhi = cv2.getTrackbarPos('Bhi', 'hsv')
    Blow = cv2.getTrackbarPos('Blow', 'hsv')
    return

cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
cv2.namedWindow('Frame', cv2.WINDOW_NORMAL)
cv2.createTrackbar('Hlow', 'hsv', lower[0], 180, tunerCb)
cv2.createTrackbar('Slow', 'hsv', lower[1], 255, tunerCb)
cv2.createTrackbar('Vlow', 'hsv', lower[2], 255, tunerCb)
cv2.createTrackbar('Hhi', 'hsv', upper[0], 180, tunerCb)
cv2.createTrackbar('Shi', 'hsv', upper[1], 255, tunerCb)
cv2.createTrackbar('Vhi', 'hsv', upper[2], 255, tunerCb)
cv2.createTrackbar('Blow', 'hsv', Blow, 255, tunerCb)
cv2.createTrackbar('Bhi', 'hsv', Bhi, 255, tunerCb)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab frame
    image = frame.array
    image = cv2.resize(image, (100, int(image.shape[0]*100/image.shape[1])), interpolation = cv2.INTER_AREA)
    
    # Get HSV image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Get Gray image
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #_ , grayMask = cv2.threshold(gray, Blow , Bhi, cv2.THRESH_BINARY)

    #res = cv2.bitwise_and(hsv, hsv, grayMask)
    
    # Filter by HSV color & blur mask
    #hsv = cv2.GaussianBlur(hsv, (21, 21), 0)
    mask = cv2.inRange(hsv, lower, upper)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    print "Found", len(contours), "contours"
    
    # Find centers of blobs and draw blue circle
    centers = []
    
    index = -1
    maxArea = 0
    for i in range(len(contours)):
	area = cv2.contourArea(contours[i])
        if area > maxArea:
            index = i
            maxArea = area
    if index > -1:
    #for i in range(0, min(len(contours),5)): # only draw top 5
        cv2.drawContours(image, [contours[index]], 0, (0, 255, 0))
        moments = cv2.moments(contours[index])
        coord = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
        centers.append(coord)
        if (coord[0] is not 0 and coord[1] is not 0):
	    cv2.circle(image, centers[-1], 3, (255, 0, 0), -1)
            print "Center at", coord
    
    # Visualize
    cv2.imshow("Frame", image)
    cv2.imshow("hsv", hsv)
    cv2.imshow("hsvMask", mask)    
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        print image
        break

    # Cleanup
    rawCapture.truncate(0)

