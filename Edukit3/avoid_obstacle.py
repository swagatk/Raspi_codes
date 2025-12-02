from pwm_motors import *
from distance_measure import * 

# Distance variables
hownear = 40.0
reverse_time = 0.5
turntime = 0.75


def is_near_obstacle(localhownear):
    distance = measure()

    print('Is near obstacle: ' + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

def avoid_obstacle(left=False):
    backward() 
    time.sleep(reverse_time)
    stopmotors()

    if left:
        turnright()
        left = False
    else:
        turnleft()
        left = True
    time.sleep(turntime)
    stopmotors()
    return left

############
if __name__ == '__main__':
	try:
		# start PWM control
		start_pwm()
		

		# Allow module to settle
		time.sleep(0.1)
		left = False
		while True:
			forward()
			time.sleep(0.1)
			if is_near_obstacle(hownear):
				stopmotors()
				left = avoid_obstacle()
	except KeyboardInterrupt:
		stopmotors()
		print("User stopped the robot")
	finally:
		stop_pwm() # stop motors
		
		# delete local copies
		try:
			del pwmMotorAForward
			del pwmMotorABackward
			del pwmMotorBForward
			del pwmMotorBBackward
		except:
			pass # Ignore if they are already gone
			
		try: # delete original file's copies
			delete_pwm()
		except:
			pass
		GPIO.cleanup() # close GPIO connection

		

