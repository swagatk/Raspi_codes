import RPi.GPIO as GPIO
import time

# Use BCM GPIO numbering
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin
BUTTON_PIN = 25 # Pin position 10 & 11 on the top row of the GPIO header

# Setup the pin as an input with an internal pull-up resistor
# This means the pin will read HIGH when not pressed, 
# and you should wire the button to connect pin 25 to GND when pressed.
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Testing push button on GPIO 25. Press Ctrl+C to exit.")

try:
    while True:
        # Since we use pull-up, a pressed button connects to GND (LOW)
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            print("Button is PRESSED!")
        else:
            print("Button is RELEASED")
            
        time.sleep(0.1) # Small delay to prevent terminal flooding and debounce

except KeyboardInterrupt:
    print("\nExiting and cleaning up GPIO...")
    GPIO.cleanup()
