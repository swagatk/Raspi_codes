import serial
import time
import threading
import sys

# --- CONFIGURATION ---
# Thresholds in centimeters
STOP_DIST = 25   # If obstacle is closer than this, STOP and Turn
SIDE_DIST = 20   # If side wall is closer than this, Nudge away
SAFE_DIST = 30   # Clear distance to move forward

# --- CONNECT TO ARDUINO ---
try:
    # Use the port that worked for you (likely /dev/ttyACM0)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    print("Connected to Robot - Starting Autonomous Mode...")
    time.sleep(2) # Wait for Arduino reboot
    
    # Set Speed to LOW (1) for safer autonomous driving
    ser.write(b'1') 
    print("Speed set to SLOW")
    
except Exception as e:
    print(f"Error: {e}")
    sys.exit()

# --- SHARED VARIABLES ---
# We store the latest sensor readings here. 
# Default to 999 (Far away) so robot moves forward if sensors fail initially.
latest_L = 999
latest_C = 999
latest_R = 999
running = True

# --- BACKGROUND THREAD (The Listener) ---
def update_sensors():
    global latest_L, latest_C, latest_R
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    # Format is D, Left, Center, Right
                    latest_L = int(parts[1])
                    latest_C = int(parts[2])
                    latest_R = int(parts[3])
            except:
                pass # Ignore corrupt packets
        time.sleep(0.01)

# Start the listener thread
t = threading.Thread(target=update_sensors)
t.daemon = True
t.start()

# --- MAIN LOOP (The Driver) ---
print("Robot is THINKING. Press Ctrl+C to Stop.")

try:
    while True:
        # 1. READ SENSORS
        # (Variables are updated automatically by the thread)
        
        # 2. DECISION LOGIC
        
        # CASE A: OBSTACLE DEAD AHEAD
        if latest_C < STOP_DIST:
            print(f"Blocked ({latest_C}cm)! Avoiding...")
            ser.write(b'S') # Stop
            time.sleep(0.2)
            
            ser.write(b'B') # Reverse briefly to un-stuck
            time.sleep(0.3)
            
            # Check which way is clearer
            if latest_L > latest_R:
                print("Turning LEFT (Left side has more space)")
                ser.write(b'L') 
            else:
                print("Turning RIGHT (Right side has more space)")
                ser.write(b'R')
                
            time.sleep(0.5) # Spin for 0.5 seconds
            ser.write(b'S') # Stop spinning
            time.sleep(0.2) # Stabilize
            
        # CASE B: TOO CLOSE TO LEFT WALL
        elif latest_L < SIDE_DIST:
            print("Too close to LEFT -> Nudging Right")
            ser.write(b'R') # Spin Right
            time.sleep(0.1) # Short nudge
            ser.write(b'F') # Resume Forward
            
        # CASE C: TOO CLOSE TO RIGHT WALL
        elif latest_R < SIDE_DIST:
            print("Too close to RIGHT -> Nudging Left")
            ser.write(b'L') # Spin Left
            time.sleep(0.1) # Short nudge
            ser.write(b'F') # Resume Forward

        # CASE D: PATH CLEAR
        else:
            # Only print occasionally to keep console clean
            # print(f"Clear: L={latest_L} C={latest_C} R={latest_R}")
            ser.write(b'F')

        # 3. LOOP DELAY
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nEmergency Stop!")
    ser.write(b'S')
    running = False
