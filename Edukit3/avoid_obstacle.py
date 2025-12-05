import RPi.GPIO as GPIO
import pwm_motors as pm
import distance_measure as dm
import time


#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Distance variables
hownear = 15.0
reverse_time = 0.5
turn_time = 0.5
duty_cycle = 50  # (0 - 100)

def is_near_obstacle(localhownear):
    distance = dm.measure()

    print('Is near obstacle: ' + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

def avoid_obstacle(left=False):
    pm.backward(duty_cycle) 
    time.sleep(reverse_time)
    pm.stopmotors()

    if left:
        pm.turnright(duty_cycle)
        left = False
    else:
        pm.turnleft(duty_cycle)
        left = True
    time.sleep(turn_time)
    pm.stopmotors()
    return left

############
if __name__ == '__main__':
	try:
		# start PWM control
		pm.start_pwm()
		

		# Allow module to settle
		time.sleep(0.1)
		left = False
		while True:
			pm.forward(duty_cycle)
			time.sleep(0.1)
			if is_near_obstacle(hownear):
				pm.stopmotors()
				print('Turning Left:', left)
				left = avoid_obstacle(left)
	except KeyboardInterrupt:
		pm.stopmotors()
		print("User stopped the robot")
	finally:
		pm.stop_pwm() # stop motors
		
		# delete local copies
		try:
			del pwmMotorAForward
			del pwmMotorABackward
			del pwmMotorBForward
			del pwmMotorBBackward
		except:
			pass # Ignore if they are already gone
			
		try: # delete original file's copies
			pm.delete_pwm()
		except:
			pass
		GPIO.cleanup() # close GPIO connection

		

