import numpy as np

## GLOBALS
#ALLSTOP = None
#hasTarget = None
#targetPos = None
#shootNow = None
#hasTarget_red = None
#targetPos_red = None
#hasTarget_blue = None
#targetPos_blue = None
#image = None
#hsv = None
#image_red = None
#image_blue = None
#red_lower = None
#red_upper = None
#blue_lower = None
#blue_upper = None

def globalInit():
    global ALLSTOP
    global hasTarget
    global targetPos
    global shootNow
    global hasTarget_red
    global targetPos_red
    global hasTarget_blue
    global targetPos_blue
    global image
    global hsv
    global image_red
    global image_blue
    global red_lower
    global red_upper
    global blue_lower
    global blue_upper
    ALLSTOP = False
    hasTarget = False
    targetPos = 0
    shootNow = False
    hasTarget_red = False
    targetPos_red = []
    hasTarget_blue = False
    targetPos_blue = []
    image = None
    hsv = None
    image_red = None
    image_blue = None
    red_lower = np.array([124, 0, 210])
    red_upper = np.array([180, 255, 255])
    blue_lower = np.array([110, 50, 54])
    blue_upper = np.array([130, 255, 255])