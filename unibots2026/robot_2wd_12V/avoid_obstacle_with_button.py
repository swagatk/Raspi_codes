import serial
import time
import threading
import sys
from gpiozero import Button

# --- CONFIGURATION ---
BUTTON_PIN = 25  # The GPIO pin for your button
STOP_DIST = 35
SIDE_DIST = 30

# --- SETUP BUTTON ---
# bounce_time prevents one press from registering as two
run_button = Button(BUTTON_PIN, bounce_time=0.2) 

# --- CONNECT TO ARDUINO ---
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    print("Connected to Robot.")
    time.sleep(2)
except Exception as e:
    print(f"Error: {e}")
    sys.exit()

# --- GLOBAL VARIABLES ---
latest_L = 999
latest_C = 999
latest_R = 999
running = True
robot_active = False  # Start in PAUSED mode for safety

# --- BUTTON TOGGLE FUNCTION ---
def toggle_mode():
    global robot_active
    robot_active = not robot_active # Flip True/False
    
    if robot_active:
        print("\n>>> GO! Robot Started. <<<")
        # Optional: Send '3' to ensure speed is Fast when starting
        ser.write(b'3') 
    else:
        print("\n>>> PAUSE! Robot Stopped. <<<")
        ser.write(b'S') # Force Stop immediately

# Attach the function to the button press
run_button.when_pressed = toggle_mode

# --- BACKGROUND THREAD (Sensor Listener) ---
def update_sensors():
    global latest_L, latest_C, latest_R
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    latest_L = int(parts[1])
                    latest_C = int(parts[2])
                    latest_R = int(parts[3])
            except:
                pass
        time.sleep(0.01)

t = threading.Thread(target=update_sensors)
t.daemon = True
t.start()

# --- MAIN LOOP ---
print("--------------------------------")
print(f"READY: Press Button on GPIO {BUTTON_PIN} to Start/Stop.")
print("--------------------------------")

try:
    while True:
        if robot_active:
            # === AUTONOMOUS LOGIC ===
            
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
                
        else:
            # === PAUSED MODE ===
            # We send 'S' repeatedly just to be safe, or just do nothing
            # Sending it once in the toggle function is usually enough,
            # but this acts as a "safety brake"
            time.sleep(0.1)

        # Loop delay
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting Program.")
    ser.write(b'S')
    running = False
