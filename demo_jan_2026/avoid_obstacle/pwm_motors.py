#controlling motors with PWM
import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO motor Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7


# PWM parameters
Frequency = 20
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

def forward(DutyCycle=70):
	print('Moving Forward')
	pwmMotorAForward.ChangeDutyCycle(DutyCycle)
	pwmMotorABackward.ChangeDutyCycle(Stop)
	pwmMotorBForward.ChangeDutyCycle(DutyCycle)
	pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward(DutyCycle=70):
	print('Moving Backward')
	pwmMotorAForward.ChangeDutyCycle(Stop)
	pwmMotorABackward.ChangeDutyCycle(DutyCycle)
	pwmMotorBForward.ChangeDutyCycle(Stop)
	pwmMotorBBackward.ChangeDutyCycle(DutyCycle)

def turnleft(DutyCycle=70):
	print('Turn left') 
	pwmMotorAForward.ChangeDutyCycle(Stop)
	pwmMotorABackward.ChangeDutyCycle(DutyCycle)
	pwmMotorBForward.ChangeDutyCycle(DutyCycle)
	pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright(DutyCycle=70):
	print('Turn right')
	pwmMotorAForward.ChangeDutyCycle(DutyCycle)
	pwmMotorABackward.ChangeDutyCycle(Stop)
	pwmMotorBForward.ChangeDutyCycle(Stop)
	pwmMotorBBackward.ChangeDutyCycle(DutyCycle)
    
    
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
	
	duty_cycle=50
	# start pwm
	start_pwm()
		
	forward(duty_cycle)
	time.sleep(1)

	turnleft(duty_cycle)
	time.sleep(0.5)

	forward(duty_cycle)
	time.sleep(1)

	turnright(duty_cycle)
	time.sleep(1)

	stopmotors()

	# stop pwm
	stop_pwm()
	delete_pwm()
	GPIO.cleanup()
