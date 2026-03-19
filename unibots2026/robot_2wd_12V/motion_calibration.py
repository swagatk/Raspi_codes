import serial
import time
import sys

# =========================================================================
# CALIBRATION VARIABLES 
# Tune these values so that the robot moves exactly as requested
# =========================================================================

# Time (in seconds) required to rotate 1 degree. 
# Increase if robot under-rotates, decrease if it over-rotates.
TIME_PER_DEGREE = 0.006  

# Time (in seconds) required to drive 1 meter. 
# Increase if robot under-drives, decrease if it over-drives.
TIME_PER_METER = 1.5     

# Default speed to use during calibration
DEFAULT_SPEED = '2' 

# =========================================================================

# Setup Serial connection to Arduino
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2) # Wait for Arduino to reset
    print("Connected to Arduino on /dev/ttyACM0")
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

def send_cmd(cmd):
    """Sends a character command to the Arduino."""
    try:
        if isinstance(cmd, str):
            cmd_bytes = cmd.encode()
        else:
            cmd_bytes = cmd
            
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        ser.write(cmd_bytes)
        ser.flush()
    except Exception as e:
        print(f"Serial write error: {e}")

def rotate(angle):
    """Rotates the robot based on degrees."""
    if angle == 0:
        return
    
    # Positive angle = Anti-clockwise (Left)
    # Negative angle = Clockwise (Right)
    direction = 'L' if angle > 0 else 'R'
    
    # Calculate duration based on the scaling variable
    duration = abs(angle) * TIME_PER_DEGREE
    
    print(f" -> Rotating {abs(angle)} degrees ({direction}) for {duration:.3f} seconds...")
    send_cmd(DEFAULT_SPEED)
    send_cmd(direction)
    time.sleep(duration)
    send_cmd('S') # STOP
    time.sleep(0.5) # Pause to let momentum settle before next move

def move_linear(distance):
    """Moves the robot straight linearly based on meters."""
    if distance == 0:
        return

    # Positive distance = Forward
    # Negative distance = Backward
    direction = 'F' if distance > 0 else 'B'
    
    # Calculate duration based on the scaling variable 
    duration = abs(distance) * TIME_PER_METER
    
    print(f" -> Moving {abs(distance)} meters ({direction}) for {duration:.3f} seconds...")
    send_cmd(DEFAULT_SPEED)
    send_cmd(direction)
    time.sleep(duration)
    send_cmd('S') # STOP
    time.sleep(0.5) # Pause to let momentum settle

def main():
    print("\n==================================")
    print("    ROBOT MOTION CALIBRATION      ")
    print("==================================")
    print("Type 'q' to quit at any time.\n")
    
    while True:
        try:
            # 1. Ask for Rotation
            rot_input = input("Enter rotation in degrees (+ for Anti-Clockwise, - for Clockwise): ").strip()
            if rot_input.lower() == 'q':
                break
            angle = float(rot_input)
            
            # 2. Ask for Linear Distance
            dist_input = input("Enter linear distance in meters (+ for Forward, - for Backward): ").strip()
            if dist_input.lower() == 'q':
                break
            distance = float(dist_input)
            
            print("\n--- Executing Motion ---")
            rotate(angle)
            move_linear(distance)
            print("--- Motion Complete ---\n")
            
        except ValueError:
            print("\n[!] Invalid input. Please enter numbers only.\n")
        except KeyboardInterrupt:
            print("\nEmergency Stop Triggered!")
            send_cmd('S')
            break

if __name__ == "__main__":
    try:
        main()
    finally:
        print("Cleaning up and stopping robot...")
        send_cmd('S')
        ser.close()
