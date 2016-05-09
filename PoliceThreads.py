from PoliceLibrary import *
from PoliceGlobals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import threading

####################
globalInit()
####################

class CameraThread (threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)
        ## Camera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        
        # Camera warmup
        time.sleep(0.1)

        # Init red & blue threads
        redThread = RedFilterThread(red_lower, red_upper, "red")
        redThread.daemon = True
        redThread.start()
        blueThread = BlueFilterThread(blue_lower, blue_upper, "blue")
        blueThread.daemon = True
        blueThread.start()
        
    def run(self):
        global ALLSTOP
        global targetPos
        global hasTarget
        global image
        global hsv

        print "Camera thread is running"

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Grab frame
    	    image = cv2.flip(frame.array,0)
    	    image = cv2.resize(image, (100, int(image.shape[0]*100/image.shape[1])), interpolation = cv2.INTER_AREA)

    	    # Get HSV image
    	    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Cleanup
            self.rawCapture.truncate(0)

            # Compare
            targetPosList = list(set(targetPos_red).intersection(targetPos_blue))
            if len(targetPosList) > 0:
                targetPos = targetPosList[0]
                hasTarget = True
                print "Match at", targetPos
            else:
                targetPos = 0
                hasTarget = False
                print "No match"
                print "Red", targetPos_red
                print "Blue", targetPos_blue

            if ALLSTOP:
                 break

            time.sleep(1)

class RedFilterThread (threading.Thread):
    def __init__(self, lower, upper, id):
    	threading.Thread.__init__(self)
        self.lower = lower
        self.upper = upper
        self.id = id

    def run(self):
        global hasTarget_red
        global targetPos_red
        global image_red
        print "Red thread is running"
        while not ALLSTOP:
            hasTarget_red, coords_red, targetPos_red, image_red = ColorFilter(image, hsv, self.lower, self.upper)
            
####################

class BlueFilterThread (threading.Thread):
    def __init__(self, lower, upper, id):
    	threading.Thread.__init__(self)
        self.lower = lower
        self.upper = upper
        self.id = id

    def run(self):
        global hasTarget_blue
        global targetPos_blue
        global image_blue
        print "Blue thread is running"
        while not ALLSTOP:
            hasTarget_blue, coords_blue, targetPos_blue, image_blue = ColorFilter(image, hsv, self.lower, self.upper)
          