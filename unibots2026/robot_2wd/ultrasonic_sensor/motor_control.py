# Code for controlling motors

import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Pins (CamJam EduKit 3 uses GPIO 10, 9, 8, 7 for motor control)
MotorA_Forward = 10
MotorA_Backward = 9
MotorB_Forward = 8
MotorB_Backward = 7


# Set motor pins as output
GPIO.setup(MotorA_Forward, GPIO.OUT)
GPIO.setup(MotorA_Backward, GPIO.OUT)
GPIO.setup(MotorB_Forward, GPIO.OUT)
GPIO.setup(MotorB_Backward, GPIO.OUT)


# PWM setup
Frequency = 20
Stop = 0

global pwmMotorA_Forward, pwmMotorA_Backward
global pwmMotorB_Forward, pwmMotorB_Backward
# create PWM objects
pwmMotorA_Forward = GPIO.PWM(MotorA_Forward, Frequency)
pwmMotorA_Backward = GPIO.PWM(MotorA_Backward, Frequency)
pwmMotorB_Forward = GPIO.PWM(MotorB_Forward, Frequency)
pwmMotorB_Backward = GPIO.PWM(MotorB_Backward, Frequency)

# start PWM objects
def start_pwm():
	pwmMotorA_Forward.start(Stop)
	pwmMotorA_Backward.start(Stop)
	pwmMotorB_Forward.start(Stop)
	pwmMotorB_Backward.start(Stop)
	

def stop_pwm():
	pwmMotorA_Forward.stop()
	pwmMotorA_Backward.stop()
	pwmMotorB_Forward.stop()
	pwmMotorB_Backward.stop()
	
def delete_pwm():
	global pwmMotorA_Forward, pwmMotorA_Backward
	global pwmMotorB_Forward, pwmMotorB_Backward
	del pwmMotorA_Forward
	del pwmMotorA_Backward
	del pwmMotorB_Forward
	del pwmMotorB_Backward
	

# Motor control functions
def stop_motors():
	print('Stopping Motors')
	pwmMotorA_Forward.ChangeDutyCycle(Stop)
	pwmMotorA_Backward.ChangeDutyCycle(Stop)
	pwmMotorB_Forward.ChangeDutyCycle(Stop)
	pwmMotorB_Backward.ChangeDutyCycle(Stop)

def move_forward(dutyCycle=50):
	print('Moving Forward')
	pwmMotorA_Forward.ChangeDutyCycle(dutyCycle)
	pwmMotorA_Backward.ChangeDutyCycle(Stop)
	pwmMotorB_Forward.ChangeDutyCycle(dutyCycle)
	pwmMotorB_Backward.ChangeDutyCycle(Stop)

def move_backward(dutyCycle=50):
	print('Moving Backward')
	pwmMotorA_Forward.ChangeDutyCycle(Stop)
	pwmMotorA_Backward.ChangeDutyCycle(dutyCycle)
	pwmMotorB_Forward.ChangeDutyCycle(Stop)
	pwmMotorB_Backward.ChangeDutyCycle(dutyCycle)
    
def turn_left(dutyCycle=50):
	print('Turning Left')
	pwmMotorA_Forward.ChangeDutyCycle(Stop)
	pwmMotorA_Backward.ChangeDutyCycle(dutyCycle)
	pwmMotorB_Forward.ChangeDutyCycle(dutyCycle)
	pwmMotorB_Backward.ChangeDutyCycle(Stop)
	
def turn_right(dutyCycle=50):
	print('Turning Right')
	pwmMotorA_Forward.ChangeDutyCycle(dutyCycle)
	pwmMotorA_Backward.ChangeDutyCycle(Stop)
	pwmMotorB_Forward.ChangeDutyCycle(Stop)
	pwmMotorB_Backward.ChangeDutyCycle(dutyCycle)

if __name__ == '__main__':
	try:
		start_pwm()
		
		# move_forward
		move_forward()
		time.sleep(1)
		move_backward()
		time.sleep(1)
		turn_left()
		time.sleep(1)
		turn_right()
		time.sleep(1)
		stop_motors()
	except KeyboardInterrupt:
		print('User stopped the robot')
	finally:
		stop_pwm()
		delete_pwm()
		GPIO.cleanup()
		
