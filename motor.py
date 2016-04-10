# External module imports
import RPi.GPIO as GPIO
import time


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

    def cw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "cw"

    def ccw(self):
        GPIO.output(self.A, GPIO.HIGH)
        GPIO.output(self.B, GPIO.HIGH)
        self.mode = "ccw"

    def brake(self):
        GPIO.output(self.A, GPIO.LOW)
        GPIO.output(self.B, GPIO.LOW)
        self.mode = "brake"

### SETUP
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

# PWM setup
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
dc = 95 # duty cycle (0-100) for PWM pin
freq = 20000
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, freq)  # Initialize PWM
pwm.start(dc) # Initial state

# Motor pins
m1 = Motor(23, 24)
m2 = Motor(20, 21)
m3 = Motor(13, 19)
m4 = Motor(22, 27)

print("Here we go! Press CTRL+C to exit")
try:
    while 1:
        m1.ccw()
        m2.ccw()
        m3.ccw()
        m4.ccw()
        time.sleep(5)
        m1.cw()
        m2.cw()
        m3.cw()
        m4.cw()
        time.sleep(5)
        #pwm.stop()
        m1.brake()
        m2.brake()
        m3.brake()
        m4.brake()
        #pwm.start(dc)
        time.sleep(5)
        #pwm.ChangeDutyCycle(dc)

# If CTRL+C is pressed, exit cleanly:
except KeyboardInterrupt: 
    pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO

