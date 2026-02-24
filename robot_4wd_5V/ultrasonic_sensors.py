import RPi.GPIO as GPIO
import time

#set GPIO modes


# Pin configuration for ultrasonic sensors
TRIG_CENTER = 11  # gpio 17 in BCM
ECHO_CENTER = 13  # gpio 27

TRIG_LEFT = 15 # gpio 22
ECHO_LEFT = 16 # gpio 23 

TRIG_RIGHT = 18 # gpio 24
ECHO_RIGHT = 22 # gpio 25


def initialize_sensors():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(TRIG_CENTER, GPIO.OUT)
        GPIO.setup(ECHO_CENTER, GPIO.IN)

        GPIO.setup(TRIG_LEFT, GPIO.OUT)
        GPIO.setup(ECHO_LEFT, GPIO.IN)

        GPIO.setup(TRIG_RIGHT, GPIO.OUT)
        GPIO.setup(ECHO_RIGHT, GPIO.IN)
        print('Sensor initialization completed')
	
def measure_distance(trigger, echo):
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    start_time = time.time()

    while GPIO.input(echo) == 0:
            start_time = time.time()

    
    while GPIO.input(echo) == 1:
            stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to cm

    return distance
    
 
def sensor_readings():
	left = measure_distance(TRIG_LEFT, ECHO_LEFT)
	#print('left = ', left)
	right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
	#print('right = ', right)
	center = measure_distance(TRIG_CENTER, ECHO_CENTER)
	#print('center=', center)
	
	return (left, right, center)

def release_pins():
        GPIO.cleanup()
	
if __name__ == '__main__':
	initialize_ultrasonic_sensors()
	
	try:
		while True:
			l, r, c = sensor_readings()
			print(f'Left: {l:.2f}, Center: {c:.2f}, Right: {r:.2f}')
	except KeyboardInterrupt:
		GPIO.cleanup()
