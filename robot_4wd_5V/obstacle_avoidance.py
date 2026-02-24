import RPi.GPIO as GPIO
import time
import motor_control_4wd as mc
import ultrasonic_sensors as us

#set GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

def avoid_obstacle(threshold=30):
    try:
        while True:
            us.initialize_sensors()
            left, right, center = us.sensor_readings()
            print(f'left: {left:.2f}, center: {center:.2f}, right:{right:.2f}') 
            us.release_pins()
            mc.initialize_motors()
            if center < threshold and left < threshold and right < threshold:
                # obstacle ahead, take a u-turn
                mc.turn_right()
                time.sleep(3)
            elif center < threshold and left < threshold and right > threshold:
                # obstacle on left, turn right
                mc.turn_right()
                time.sleep(1)
            elif center < threshold and right < threshold and left > threshold:
                # obstacle on left, turn right
                mc.turn_left()
                time.sleep(1)
            elif center < threshold and left > threshold and right > threshold:
                # narrow obstacle (e.g., leg of a table), take U turn
                mc.turn_left()
                time.sleep(3)
            elif center > threshold and left > threshold and right > threshold:
                # no obstacle, move forward
                mc.move_forward()
            else: # any other case - this will never be executed.
                mc.stop()
            mc.release_pins()
    except KeyboardInterrupt:
        print(" Keyboard interrupt. Stopping robot...")
        mc.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    avoid_obstacle()
