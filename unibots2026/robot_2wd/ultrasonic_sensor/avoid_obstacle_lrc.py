
import RPi.GPIO as GPIO
import time
import motor_control as mc


# Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)



# Ultrasonic Sensor Pins
Trig_Center = 17
Echo_Center = 18
Trig_Right = 27
Echo_Right = 22
Trig_Left = 23
Echo_Left = 24



# Set sensor pins as input/output
for pin in [Trig_Center, Trig_Right, Trig_Left]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)  # Ensure trigger is low

for pin in [Echo_Center, Echo_Right, Echo_Left]:
    GPIO.setup(pin, GPIO.IN)


# Distance Threshold
safe_distance = 15  # cm
reverse_time = 0.5
turn_time = 0.75



# Function to measure distance
def measure_distance(trigger, echo):
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    start_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()
        
    while GPIO.input(echo) == 1:
        stop_time = time.time()
        interval = stop_time - start_time
        if interval >= 0.02:
            #print("Too close to measure!")
            stop_time = start_time
            break

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance


# Obstacle avoidance logic
def avoid_obstacle():
	center_distance = measure_distance(Trig_Center, Echo_Center)
	left_distance = measure_distance(Trig_Left, Echo_Left)
	right_distance = measure_distance(Trig_Right, Echo_Right)
	print(f"Center: {center_distance} cm, Left: {left_distance} cm, Right: {right_distance} cm")
	if center_distance < safe_distance:
		mc.stop_motors()
		time.sleep(0.1)
		if left_distance > right_distance and left_distance > safe_distance:
			mc.turn_left()
			time.sleep(turn_time)
		elif right_distance > left_distance and right_distance > safe_distance:
			mc.turn_right()
			time.sleep(turn_time)
		else:
			mc.move_backward()
			time.sleep(reverse_time)
			mc.turn_right()
			time.sleep(turn_time)
			mc.stop_motors()
	else:
		mc.move_forward()

if __name__ == '__main__':
	try:
		print("Starting obstacle avoidance...")
		time.sleep(0.5)

		while True:
			avoid_obstacle()
			time.sleep(0.1)
	except KeyboardInterrupt:
		print("Stopping robot...")
	finally:
		mc.stop_pwm()
		mc.delete_pwm()
		GPIO.cleanup()
