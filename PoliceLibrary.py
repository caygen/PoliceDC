import RPi.GPIO as GPIO
### FUNCTIONS
class Motor:
    def __init__(self, A, B):
        self.A = A
        self.B = B
        self.mode = "brake"
        GPIO.setup(A, GPIO.OUT)
        GPIO.setup(B, GPIO.OUT)
        self.brake()
    
    def __str__(self):
        return "Pin A: " + str(self.A) + ", Pin B: " + str(self.B) + ", Mode: " + self.mode

    def ccw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "ccw"

    def cw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "cw"

    def brake(self):
        GPIO.output(self.A, GPIO.LOW)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"

class Robot:
    def __init__(self):
        self.frontLeft = None
        self.frontRight = None
        self.rearLeft = None
        self.rearRight = None

    def forward(self, t=5):
        print "MOVING FORWARD"
        self.frontLeft.ccw()
        self.frontRight.cw()
        self.rearLeft.ccw()
        self.rearRight.cw()
        time.sleep(t)

    def turnLeft(self, t=5):
        print "TURNING LEFT"
        self.frontLeft.cw()
        self.frontRight.cw()
        self.rearLeft.cw()
        self.rearRight.cw()
        time.sleep(t)

    def turnRight(self, t=5):
        print "TURNING RIGHT"
        self.frontLeft.ccw()
        self.frontRight.ccw()
        self.rearLeft.ccw()
        self.rearRight.ccw()
        time.sleep(t)
    
    def stop(self, t=1):
        print "STOPPING"
        self.frontLeft.brake()
        self.frontRight.brake()
        self.rearLeft.brake()
        self.rearRight.brake()
