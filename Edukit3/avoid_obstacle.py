# CamJam Edukit 3 - Robots
# Worksheet 9 - Obstacle Avoidance


# load the libraries
import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 7
pinMotorBBackward = 8
pinTrigger = 17
pinEcho = 18



# PWM parameters
Frequency = 20
DutyCycleA = 70
DutyCycleB = 70
Stop = 0

# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# Distance variables
hownear = 10.0
reversetime = 0.5
turntime = 0.75


# Set the GPIO to Software PWM at 'Frequency' Hertz
pwmMotorAForward = GPIO.PWM(pinMotorAForward, Frequency)
pwmMotorABackward = GPIO.PWM(pinMotorABackward, Frequency)
pwmMotorBForward = GPIO.PWM(pinMotorBForward, Frequency)
pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, Frequency)

# set the duty cycle for software PWM - initially to 0
pwmMotorAForward.start(Stop)
pwmMotorABackward.start(Stop)
pwmMotorBForward.start(Stop)
pwmMotorBBackward.start(Stop)

def stopmotors():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def forward():
    print('Moving Forward')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward():
    print('Moving backward')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
    print('Turning Left')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
    print('Turning Right')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)



def measure():
        
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)
    
    starttime = time.time()
    stoptime = starttime

    while GPIO.input(pinEcho) == 0:
        starttime = time.time()
        stoptime = starttime

    while GPIO.input(pinEcho) == 1:
        stoptime = time.time()
        if stoptime - starttime >= 0.04:
            print('Hold on there! you are too close to me to see.')
            stoptime = starttime
            break
    elapsedtime = stoptime - starttime
    distance = (elapsedtime * 34300) / 2

    return distance
    
def isnearobstacle(localhownear):
    distance = measure()

    print('Is near obstacle: ' + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

def avoidobstacle():
    backward() 
    time.sleep(reversetime)
    stopmotors()

    turnright()
    time.sleep(turntime)
    stopmotors()

############
if __name__ == '__main__':
    try:
        GPIO.output(pinTrigger, False)

        # Allow module to settle
        time.sleep(0.1)

        while True:
            forward()
            time.sleep(0.1)
            if isnearobstacle(hownear):
                stopmotors()
                avoidobstacle()
    except KeyboardInterrupt:
        GPIO.cleanup()
    
        
