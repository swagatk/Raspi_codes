# CamJam EduKit 1 - Basics
# Worksheet 7 - Traffic Lights - Solution

# Import Libraries
import os # Gives Python access to Linux commands
import time # Proves time related commands
import RPi.GPIO as GPIO # Gives access to the GPIO Pins

# Set the GPIO pin naming mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up variables for the LED, Buzzer and switch pins

# LEDs for Cars
Car_Red = 18
Car_Amber = 23
Car_Green = 24

Button = 25
Buzzer = 22


# LEDs for Pedestrians
Ped_Red = 10
Ped_Green = 11



# Set up each of the input (switch) and output (LEDs, Buzzer) pins
GPIO.setup(Buzzer, GPIO.OUT)
GPIO.setup(Car_Red, GPIO.OUT)
GPIO.setup(Car_Amber, GPIO.OUT)
GPIO.setup(Car_Green, GPIO.OUT)

GPIO.setup(Ped_Red, GPIO.OUT)
GPIO.setup(Ped_Green, GPIO.OUT)


GPIO.setup(Button, GPIO.IN)

GPIO.output(Buzzer, GPIO.LOW)
GPIO.output(Car_Red, GPIO.LOW)
GPIO.output(Car_Amber, GPIO.LOW)
GPIO.output(Car_Green, GPIO.LOW)

GPIO.output(Ped_Red, GPIO.LOW)
GPIO.output(Ped_Green, GPIO.LOW)

# Define a function for the initial state (Green LED on, rest off)
# (If you have the second 'pedestrian LEDs, turn the red on & green
# off)
def startgreen():
    # Remember all code in the function is indented
    GPIO.output(Car_Green, GPIO.HIGH)
    GPIO.output(Car_Amber, GPIO.LOW)
    GPIO.output(Car_Red, GPIO.LOW)
    
    GPIO.output(Ped_Green, GPIO.LOW)
    GPIO.output(Ped_Red, GPIO.HIGH)
    
 
# Turn the green off and the amber on for 3 seconds
# ('Pedestrian' red LED stays lit)
def steadyamber():
    # Remember all code in the function is indented
    GPIO.output(Car_Green, GPIO.LOW)
    GPIO.output(Car_Red, GPIO.LOW)
    GPIO.output(Car_Amber, GPIO.HIGH)
    
    GPIO.output(Ped_Red, GPIO.HIGH)
    GPIO.output(Ped_Green, GPIO.LOW)
    
    time.sleep(3)
 
# Turn the amber off, and then the red on for 1 second
def steadyred():
    # Remember all code in the function is indented
    GPIO.output(Car_Green, GPIO.LOW)
    GPIO.output(Car_Amber, GPIO.LOW)
    GPIO.output(Car_Red, GPIO.HIGH)
    time.sleep(1)

# Sound the buzzer for 4 seconds
# (If you have the 'pedestrian' LEDs, turn the red off and green on)
def startwalking():
# Make the buzzer buzz on and off, half a second of
# sound followed by half a second of silence

    GPIO.output(Ped_Red, GPIO.LOW)
    GPIO.output(Ped_Green, GPIO.HIGH)
    for _ in range(4):
        GPIO.output(Buzzer, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(Buzzer, GPIO.LOW)
        time.sleep(0.5)
# Turn the buzzer off and wait for 2 seconds
# (If you have a second green 'pedestrian' LED, make it flash on and
# off for the two seconds)
def dontwalk():
    # Remember all code in the function is indented
    GPIO.output(Car_Red, GPIO.HIGH)
    GPIO.output(Car_Amber, GPIO.LOW)
    GPIO.output(Ped_Red, GPIO.LOW)
    
    GPIO.output(Buzzer, GPIO.LOW)
    for _ in range(2):
        GPIO.output(Ped_Green, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(Ped_Green, GPIO.HIGH)
        time.sleep(0.5)
        
# Flash the amber on and off for 6 seconds
# (And the green 'pedestrian' LED too)
def flashingambergreen():
    # Remember all code in the function is indented
    GPIO.output(Car_Red, GPIO.HIGH)
    GPIO.output(Ped_Red, GPIO.LOW)
    
    for _ in range(6):
        GPIO.output(Car_Amber, GPIO.HIGH)
        GPIO.output(Ped_Green, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(Car_Amber, GPIO.LOW)
        GPIO.output(Ped_Green, GPIO.LOW)
        time.sleep(0.5)
 
# Flash the amber for one more second
# (Turn the green 'pedestrian' LED off and the red on)
def flashingamber():
    # Remember all code in the function is indented
    GPIO.output(Ped_Green, GPIO.LOW)
    GPIO.output(Ped_Red, GPIO.HIGH)
    
    GPIO.output(Car_Amber, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(Car_Amber, GPIO.HIGH)
    time.sleep(0.5)

def button_is_pressed():
    
    if GPIO.input(Button) == False:
        print("Button is Pressed. Wait for Signal Change")
        return True
    else:
        return False
    
 
# Go through the traffic light sequence by calling each function
# one after the other.
def trafficlightqequence():
    # Remember all code in the function is indented
    steadyamber()
    steadyred()
    startwalking()
    dontwalk()
    flashingambergreen()
    flashingamber()
    startgreen()

#############
 
os.system('clear') # Clears the terminal

print("Traffic Lights")
# Initialise the traffic lights
print('Original Button State:', GPIO.input(Button))

startgreen()
# Here is the loop that waits at least 20 seconds before
# stopping the cars if the button has been pressed
while True: # Loop around forever
    buttonnotpressed = True # Button has not been pressed
    start = time.time() # Records the current time
    while buttonnotpressed: # While the button has not been pressed
        time.sleep(0.1) # Wait for 0.1s
        if button_is_pressed(): # If the button is pressed
            now = time.time()
            buttonnotpressed = False # Button has been pressed
            if (now - start) <= 20: # If under 20 seconds
                time.sleep(20 - (now - start)) # Wait until 20s is up
                trafficlightqequence() # Run the traffic light sequence
GPIO.cleanup()
