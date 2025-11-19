import RPi.GPIO as GPIO
import time

# Setup
LED_PIN = 18
GPIO.setmode(GPIO.BCM)  # Use BCM numbering (matches 'pinout' command)
GPIO.setup(LED_PIN, GPIO.OUT)

print(f"Blinking LED on GPIO {LED_PIN} (Press CTRL+C to stop)")

try:
    while True:
        # Turn LED ON
        GPIO.output(LED_PIN, GPIO.HIGH)
        print("LED ON")
        time.sleep(1)
        
        # Turn LED OFF
        GPIO.output(LED_PIN, GPIO.LOW)
        print("LED OFF")
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up GPIO on CTRL+C exit
    print("\nExiting and cleaning up GPIO...")
    GPIO.cleanup()
