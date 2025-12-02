#controlling motors with PWM
import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO motor Pins
pinMotorAForward = 10
pinMotorABackward = 9
pinMotorBForward = 8
pinMotorBBackward = 7


# PWM parameters
Frequency = 20
DutyCycleA = 30
DutyCycleB = 30
Stop = 0


# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)

# Set the GPIO to Software PWM at 'Frequency' Hertz
pwmMotorAForward = GPIO.PWM(pinMotorAForward, Frequency)
pwmMotorABackward = GPIO.PWM(pinMotorABackward, Frequency)
pwmMotorBForward = GPIO.PWM(pinMotorBForward, Frequency)
pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, Frequency)

# set the duty cycle for software PWM - initially to 0

def start_pwm():
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
	print('Moving Backward')
	pwmMotorAForward.ChangeDutyCycle(Stop)
	pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
	pwmMotorBForward.ChangeDutyCycle(Stop)
	pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
	print('Turn left') 
	pwmMotorAForward.ChangeDutyCycle(Stop)
	pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
	pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
	pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
	print('Turn right')
	pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
	pwmMotorABackward.ChangeDutyCycle(Stop)
	pwmMotorBForward.ChangeDutyCycle(Stop)
	pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)
    
    
def stop_pwm():
	# stop pwm motors
	pwmMotorAForward.stop()
	pwmMotorABackward.stop()
	pwmMotorBForward.stop()
	pwmMotorBBackward.stop()
	

def delete_pwm():
	global pwmMotorABackward, pwmMotorAForward
	global pwmMotorBBackward, pwmMotorBForward
	# Delete PWM objects 
	del pwmMotorAForward
	del pwmMotorABackward
	del pwmMotorBForward
	del pwmMotorBBackward
	

######
if __name__ == '__main__':
	
	# start pwm
	start_pwm()
		
	forward()
	time.sleep(1)

	turnleft()
	time.sleep(0.5)

	forward()
	time.sleep(1)

	turnright()
	time.sleep(0.5)

	stopmotors()

	# stop pwm
	stop_pwm()
	delete_pwm()
	GPIO.cleanup()
