
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

# Ultrasonic Sensor Pins
Trig_Center = 17
Echo_Center = 18
Trig_Right = 27
Echo_Right = 22
Trig_Left = 23
Echo_Left = 24

# Set motor pins as output
GPIO.setup(MotorA_Forward, GPIO.OUT)
GPIO.setup(MotorA_Backward, GPIO.OUT)
GPIO.setup(MotorB_Forward, GPIO.OUT)
GPIO.setup(MotorB_Backward, GPIO.OUT)

# Set sensor pins as input/output
for pin in [Trig_Center, Trig_Right, Trig_Left]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)  # Ensure trigger is low

for pin in [Echo_Center, Echo_Right, Echo_Left]:
    GPIO.setup(pin, GPIO.IN)

# PWM setup
Frequency = 20
DutyCycle = 40  # Adjust as needed
Stop = 0

# create pwm objects
pwmMotorA_Forward = GPIO.PWM(MotorA_Forward, Frequency)
pwmMotorA_Backward = GPIO.PWM(MotorA_Backward, Frequency)
pwmMotorB_Forward = GPIO.PWM(MotorB_Forward, Frequency)
pwmMotorB_Backward = GPIO.PWM(MotorB_Backward, Frequency)

# start pwm
pwmMotorA_Forward.start(Stop)
pwmMotorA_Backward.start(Stop)
pwmMotorB_Forward.start(Stop)
pwmMotorB_Backward.start(Stop)

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

# Motor control functions
def stop_motors():
    pwmMotorA_Forward.ChangeDutyCycle(Stop)
    pwmMotorA_Backward.ChangeDutyCycle(Stop)
    pwmMotorB_Forward.ChangeDutyCycle(Stop)
    pwmMotorB_Backward.ChangeDutyCycle(Stop)

def move_forward():
    pwmMotorA_Forward.ChangeDutyCycle(DutyCycle)
    pwmMotorA_Backward.ChangeDutyCycle(Stop)
    pwmMotorB_Forward.ChangeDutyCycle(DutyCycle)
    pwmMotorB_Backward.ChangeDutyCycle(Stop)

def move_backward():
    pwmMotorA_Forward.ChangeDutyCycle(Stop)
    pwmMotorA_Backward.ChangeDutyCycle(DutyCycle)
    pwmMotorB_Forward.ChangeDutyCycle(Stop)
    pwmMotorB_Backward.ChangeDutyCycle(DutyCycle)

def turn_left():
    pwmMotorA_Forward.ChangeDutyCycle(Stop)
    pwmMotorA_Backward.ChangeDutyCycle(DutyCycle)
    pwmMotorB_Forward.ChangeDutyCycle(DutyCycle)
    pwmMotorB_Backward.ChangeDutyCycle(Stop)

def turn_right():
    pwmMotorA_Forward.ChangeDutyCycle(DutyCycle)
    pwmMotorA_Backward.ChangeDutyCycle(Stop)
    pwmMotorB_Forward.ChangeDutyCycle(Stop)
    pwmMotorB_Backward.ChangeDutyCycle(DutyCycle)

# Obstacle avoidance logic
def avoid_obstacle():
    center_distance = measure_distance(Trig_Center, Echo_Center)
    left_distance = measure_distance(Trig_Left, Echo_Left)
    right_distance = measure_distance(Trig_Right, Echo_Right)

    print(f"Center: {center_distance} cm, Left: {left_distance} cm, Right: {right_distance} cm")

    if center_distance < safe_distance:
        stop_motors()
        time.sleep(0.1)
       
        if left_distance > right_distance and left_distance > safe_distance:
            print("Turning left")
            turn_left()
            time.sleep(turn_time)
        elif right_distance > left_distance and right_distance > safe_distance:
            print("Turning right")
            turn_right()
            time.sleep(turn_time)
        else:
            print("Reversing")
            move_backward()
            time.sleep(reverse_time)
            turn_right()
            time.sleep(turn_time)

        stop_motors()
    else:
        move_forward()

# Main loop
try:
    print("Starting obstacle avoidance...")
    time.sleep(0.5)

    while True:
        avoid_obstacle()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping robot...")
    GPIO.cleanup()
