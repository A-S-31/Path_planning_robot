import RPi.GPIO as GPIO
import time

# Motor Driver Pins (adjust to your wiring)
motor_A_enable = 17  # Enable pin for Motor A
motor_A_forward = 27 # Input 1 for Motor A
motor_A_backward = 22 # Input 2 for Motor A

motor_B_enable = 23  # Enable pin for Motor B
motor_B_forward = 24 # Input 3 for Motor B
motor_B_backward = 25 # Input 4 for Motor B

# Ultrasonic Sensor Pins (adjust to your wiring)
TRIG= 6
ECHO = 5

TRIGL= 10
ECHOL=9

TRIGR=26
ECHOR=16





# Constants
SPEED = 30  # Motor speed (0-100)
DISTANCE_THRESHOLD = 20  # Distance in cm to trigger obstacle avoidance

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Setup
GPIO.setup(motor_A_enable, GPIO.OUT)
GPIO.setup(motor_A_forward, GPIO.OUT)
GPIO.setup(motor_A_backward, GPIO.OUT)
GPIO.setup(motor_B_enable, GPIO.OUT)
GPIO.setup(motor_B_forward, GPIO.OUT)
GPIO.setup(motor_B_backward, GPIO.OUT)

# Ultrasonic Sensor Setup
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)

GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)

# PWM Setup for Motor Speed Control
pwm_A = GPIO.PWM(motor_A_enable, 100)  # 100Hz frequency
pwm_B = GPIO.PWM(motor_B_enable, 100)
pwm_A.start(0)  # Start with 0% duty cycle (stopped)
pwm_B.start(0)

def forward():
    GPIO.output(motor_A_forward, GPIO.HIGH)
    GPIO.output(motor_A_backward, GPIO.LOW)
    GPIO.output(motor_B_forward, GPIO.HIGH)
    GPIO.output(motor_B_backward, GPIO.LOW)
    pwm_A.ChangeDutyCycle(SPEED)
    pwm_B.ChangeDutyCycle(SPEED)

def backward():
    GPIO.output(motor_A_forward, GPIO.LOW)
    GPIO.output(motor_A_backward, GPIO.HIGH)
    GPIO.output(motor_B_forward, GPIO.LOW)
    GPIO.output(motor_B_backward, GPIO.HIGH)
    pwm_A.ChangeDutyCycle(SPEED)
    pwm_B.ChangeDutyCycle(SPEED)

def left():
    GPIO.output(motor_A_forward, GPIO.LOW)
    GPIO.output(motor_A_backward, GPIO.HIGH)
    GPIO.output(motor_B_forward, GPIO.HIGH)
    GPIO.output(motor_B_backward, GPIO.LOW)
    pwm_A.ChangeDutyCycle(SPEED)
    pwm_B.ChangeDutyCycle(SPEED)

def right():
    GPIO.output(motor_A_forward, GPIO.HIGH)
    GPIO.output(motor_A_backward, GPIO.LOW)
    GPIO.output(motor_B_forward, GPIO.LOW)
    GPIO.output(motor_B_backward, GPIO.HIGH)
    pwm_A.ChangeDutyCycle(SPEED)
    pwm_B.ChangeDutyCycle(SPEED)

def stop():
    pwm_A.ChangeDutyCycle(0)
    pwm_B.ChangeDutyCycle(0)

def distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    dist = (time_elapsed * 34300) / 2
    return dist

def distanceL():
    GPIO.output(TRIGL, True)
    time.sleep(0.00001)
    GPIO.output(TRIGL, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHOL) == 0:
        start_time = time.time()

    while GPIO.input(ECHOL) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    dist = (time_elapsed * 34300) / 2
    
    return dist
    
def distanceR():
    GPIO.output(TRIGR, True)
    time.sleep(0.00001)
    GPIO.output(TRIGR, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHOR) == 0:
        start_time = time.time()

    while GPIO.input(ECHOR) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    dist = (time_elapsed * 34300) / 2
    
    return dist
    
