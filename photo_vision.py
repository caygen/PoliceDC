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

# blue
lower = np.array([110, 50, 50])
upper = np.array([130, 255, 255])

# white
w_lower = np.array([0, 0, 250])
w_upper = np.array([0, 0, 255])

#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab frame
    image = frame.array
    
    # Get HSV image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Filter by HSV color & blur mask
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.GaussianBlur(mask, (21, 21), 0)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    print "Found", len(contours), "contours"
    
    # Find centers of blobs and draw blue circle
    centers = []
    for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        coord = (int(moments['m10']/max(moments['m00'], 1)), int(moments['m01']/max(moments['m00'], 1)))
        centers.append(coord)
        cv2.circle(image, centers[-1], 3, (255, 0, 0), -1)
        print "Center at", coord
    
    # Visualize
    cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

    # Cleanup
    rawCapture.truncate(0)