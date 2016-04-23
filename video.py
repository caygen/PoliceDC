#!/usr/bin/python

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# Camera Setup
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Camera warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    
    # Visualize
    cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

    # Cleanup
    rawCapture.truncate(0)

camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cv2.imshow("Image", image)
cv2.waitKey(0)
