"""
Avoid obstacle using Motorshield ultrasonic sensors
"""

import RPi.GPIO as GPIO
import time
import motor_control_4wd as mc
import ms_ultrasonic as msu


#set GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

def avoid_obstacle(dist_threshold=15):
	try:
		left_flag = True
		while True:
			# measure distance using ultrasonic sensor
			dist = msu.measure_distance()
			
			# if obstacle is detected, turn to avoid collision
			if dist < dist_threshold:
				if left_flag:
					mc.turn_left()  #turn left
					time.sleep(3)
					left_flag = False
				else:
					mc.turn_right()
					time.sleep(3)
					left_flag = True
			else: # no obstacle detected
				mc.move_forward() # turn right
				time.sleep(1)
				
	except KeyboardInterrupt:
		print('Stopped by User')
	finally:
		mc.stop()
		GPIO.cleanup()
		
if __name__ == '__main__':
	# initialize sensors
	msu.initialize_sensors()
	
	# initialize motors
	mc.initialize_motors()
	
	avoid_obstacle()
		
					
				
			
			
