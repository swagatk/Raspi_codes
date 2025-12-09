import PiMotor
import time
import RPi.GPIO as GPIO



def initialize_motors():
    GPIO.setmode(GPIO.BOARD)                    
    GPIO.setwarnings(False)
    global m1, m2, m3, m4, ab, al, af, ar, motorAll
    # Initialize motor objects
    m1 = PiMotor.Motor("MOTOR1", 2)
    m2 = PiMotor.Motor("MOTOR2", 2)
    m3 = PiMotor.Motor("MOTOR3", 1)
    m4 = PiMotor.Motor("MOTOR4", 1)

    #To drive all motors together
    motorAll = PiMotor.LinkedMotors(m1,m2,m3,m4)

    # Arrow indicators
    ab = PiMotor.Arrow(1)  # Back
    al = PiMotor.Arrow(2)  # Left
    af = PiMotor.Arrow(3)  # Forward
    ar = PiMotor.Arrow(4)  # Right
    print('Motors initialized')

# Function to move forward
def move_forward():
    global motorAll, af
    print("Robot Moving Forward ")
    af.on()
    al.off()
    ar.off()
    ab.off()
    motorAll.forward(100)

# Function to move backward
def move_backward():
    global motorAll, af, ab
    print("Robot Moving Backward ")
    af.off()
    al.off()
    ar.off()
    ab.on()
    motorAll.reverse(100)


# Function to turn left
def turn_left():
    print("Robot Turning Left ")
    global ab, al, m1, m2, m3, m4
    ab.off()
    af.off()
    ar.off()
    al.on()
    m1.stop()
    m2.stop()
    m3.forward(100)
    m4.forward(100)


# Function to turn right
def turn_right():
    global ar, al, m1, m2, m3, m4
    print("Robot Turning Right ")
    ar.on()
    al.off()
    ab.off()
    af.off()
    m1.forward(100)
    m2.forward(100)
    m3.stop()
    m4.stop()
    

# Function to stop
def stop():
    global al, ar, ab, af, m1, m2, m3, m4
    print("Robot Stop ")
    al.off()
    af.off()
    ar.off()
    ab.off()
    motorAll.stop()

def release_pins():
    global m1, m2, m3, m4

    m1.stop_pwm()
    m2.stop_pwm()
    m3.stop_pwm()
    m4.stop_pwm()
    del m1, m2, m3, m4
    GPIO.cleanup()




if __name__ == '__main__':
    initialize_motors()

    try:
        while True:
            move_forward()
            time.sleep(1)
            move_backward()
            time.sleep(1)
            turn_left()
            time.sleep(1)
            turn_right()
            time.sleep(1)
            stop()
    except KeyboardInterrupt:
        GPIO.cleanup()


