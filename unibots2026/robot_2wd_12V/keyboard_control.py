import serial
import time
import sys
import termios
import tty
import threading

# --- GLOBAL VARIABLES ---
# We use this to stop the background thread when we quit
running = True 

# --- CONNECT TO ARDUINO ---
try:
    # Double check your port: /dev/ttyACM0 or /dev/ttyUSB0
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    print("Connected to Robot!")
    time.sleep(2) # Wait for Arduino reboot
except Exception as e:
    print(f"Error: {e}")
    sys.exit()

# --- BACKGROUND THREAD: READ SENSORS ---
def listen_to_arduino():
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    # Print over the same line (\r) to avoid scrolling mess
                    # "end=''" prevents a new line
                    sys.stdout.write(f"\rSensors -> L:{parts[1]} C:{parts[2]} R:{parts[3]}   ")
                    sys.stdout.flush()
            except:
                pass
        time.sleep(0.05) # Small rest to save CPU

# --- KEYBOARD FUNCTION (Standard/Blocking) ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- MAIN PROGRAM ---
print("\n--- CONTROLS ---")
print("WASD: Move | SPACE: Stop | 1-3: Speed")
print("Q: Quit")
print("----------------")

# 1. Start the Background Listener
sensor_thread = threading.Thread(target=listen_to_arduino)
sensor_thread.daemon = True # Kills thread if program crashes
sensor_thread.start()

# 2. Start the Main Keyboard Loop
try:
    while True:
        # This waits here until you actually press a key
        key = get_key().lower()
        
        if key == 'w': ser.write(b'F')
        elif key == 's': ser.write(b'B')
        elif key == 'a': ser.write(b'L')
        elif key == 'd': ser.write(b'R')
        elif key == ' ': ser.write(b'S')
        elif key == '1': ser.write(b'1')
        elif key == '2': ser.write(b'2')
        elif key == '3': ser.write(b'3')
        elif key == 'q': 
            ser.write(b'S')
            running = False # Stop the background thread
            print("\nQuitting...")
            break

except KeyboardInterrupt:
    print("\nEmergency Stop")
    ser.write(b'S')
    running = False
