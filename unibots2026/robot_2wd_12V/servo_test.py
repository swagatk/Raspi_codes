import serial
import time
import sys

# Configure the serial port (adjust '/dev/ttyACM0' if your Arduino is on a different port like USB0)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

try:
    print(f"Connecting to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Wait for the Arduino to reset after establishing the serial connection
    time.sleep(2) 
    
    # Clear any junk data out of the buffer
    ser.reset_input_buffer() 
    print("Connected successfully!\n")
    
except serial.SerialException as e:
    print(f"Error connecting to Arduino: {e}")
    print("Check if the port is correct and the Arduino is plugged in.")
    sys.exit(1)

print("=== SERVO MOTOR TEST UTILITY ===")
print("Commands:")
print("  U : Shoulder Right UP (130 -> 0)")
print("  D : Shoulder Right DOWN (0 -> 130)")
print("  E : Elbow UP (Left: 90 -> 0, Right: 90 -> 180)")
print("  e : Elbow DOWN (Left: 0 -> 90, Right: 180 -> 90)")
print("  A : ARM UP (Shoulder Right UP + Elbow DOWN, palm stays horizontal)")
print("  a : ARM DOWN (Shoulder Right home + Elbow UP, return to start)")
print("  O : Gate/Wrist OPEN")
print("  C : Gate/Wrist CLOSED")
print("  v <id> <angle> : Move specific servo to angle (e.g. 'v 0 90')")
print("                   IDs: 0=ShoulderL, 1=ShoulderR, 2=ElbowL, 3=ElbowR, 4=Wrist")
print("  Q : Quit test utility")
print("================================\n")

try:
    while True:
        # Get user input
        cmd = input("Enter command: ").strip()
        
        # Check for quit command
        if cmd.upper() == 'Q':
            print("Stopping motors and exiting...")
            ser.write(b'S') # Send stop signal for both DC and safety
            ser.write(b'x') # Send detach signal specifically to kill servos power
            time.sleep(0.1) # Give Arduino a moment to process the detach
            break
            
        # Send valid servo commands to the Arduino
        if cmd in ['U', 'D', 'E', 'e', 'O', 'C', 'A', 'a']:
            # Clear any accumulated sensor data to prevent buffer overflow
            ser.reset_input_buffer()
            # Make sure it resets watchdog quickly by appending a newline
            ser.write((cmd + '\n').encode('utf-8'))
            print(f"-> Sent preset command: '{cmd}'")
        elif cmd.startswith('v '):
            # Validate servo command format and angle range
            try:
                parts = cmd.split()
                if len(parts) == 3:
                    servo_id = int(parts[1])
                    angle = int(parts[2])
                    if servo_id < 0 or servo_id > 4:
                        print(f"Invalid servo ID {servo_id}! Must be 0-4.")
                        continue
                    if angle < 0 or angle > 180:
                        print(f"Invalid angle {angle}! Must be 0-180 degrees.")
                        continue
                    # Clear any accumulated sensor data to prevent buffer overflow
                    ser.reset_input_buffer()
                    # Send custom servo command format 'v <id> <angle>'
                    ser.write((cmd + '\n').encode('utf-8'))
                    print(f"-> Sent individual command: '{cmd}'")
                else:
                    print("Invalid format! Use: v <id> <angle>")
            except ValueError:
                print("Invalid numbers! Use: v <id> <angle>")
        else:
            print("Invalid command! Please use U, D, E, e, O, C, 'v <id> <angle>', or Q.")
            
except KeyboardInterrupt:
    print("\nTest interrupted by user. Stopping and exiting...")
    if 'ser' in locals() and ser.is_open:
        ser.write(b'S') # Ensure safety stop gets sent
        ser.write(b'x') # Kill servos explicitly
        time.sleep(0.1)
finally:
    # Ensure the serial port is cleanly closed on exit
    if ser.is_open:
        ser.close()