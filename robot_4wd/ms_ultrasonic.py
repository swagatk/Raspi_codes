"""
It uses the Ultrasonic sensors pin provided on the MotorShield HAT
"""

import RPi.GPIO as GPIO
import time

import PiMotor

TRIG = 29
ECHO = 31

def initialize_sensors():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	
def measure_distance():		
	avgDistance=0
	for i in range(5):
		GPIO.output(TRIG, False)
		time.sleep(0.01)

		GPIO.output(TRIG, True)
		time.sleep(0.00001)
		GPIO.output(TRIG, False)
		
		pulse_start = time.time()
		
		while GPIO.input(ECHO)==0:
			pulse_start = time.time()

		while GPIO.input(ECHO)==1:
			pulse_end = time.time()
			pulse_duration = pulse_end - pulse_start
			if pulse_duration > 0.01: # taking too long
				pulse_end = pulse_start
				break

		distance = (pulse_duration * 34300)/2
		avgDistance=avgDistance+distance

	avgDistance = round(avgDistance/5)
	return avgDistance
	
if __name__ == '__main__':
	initialize_sensors()
	
	try:
		while True:
			dist = measure_distance()
			print('Distance: ', dist)
	except KeyboardInterrupt:
		print('Stopped by User')
	finally:
		GPIO.cleanup()
		
