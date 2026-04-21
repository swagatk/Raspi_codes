import serial
import serial.tools.list_ports
import time
import sys
import termios
import tty
import threading
import select
import subprocess

# --- GLOBAL VARIABLES ---
# We use this to stop the background thread when we quit
running = True 
camera_process = None # Tracks the video stream process

# --- CONNECT TO ARDUINO ---
def find_arduino_port():
    print("Scanning for available serial ports...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'ttyACM' in port.device or 'ttyUSB' in port.device:
            return port.device
    return None

try:
    # Auto-detect your port: /dev/ttyACM* or /dev/ttyUSB*
    port = find_arduino_port()
    if not port:
        print("Error: Could not find any connected Arduino (checked ttyACM* and ttyUSB*).")
        print("Please check the USB connection.")
        sys.exit(1)
        
    ser = serial.Serial(port, 115200, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to Robot on {port}!")
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

# --- KEYBOARD FUNCTION (Non-Blocking with Timeout) ---
def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- MAIN PROGRAM ---
print("\n--- CONTROLS ---")
print("WASD: Move | SPACE: Stop | 1-3: Speed")
print("U: Arm UP | J: Arm DOWN | H: Arm VERTICAL")
print("O: Gripper OPEN | C: Gripper CLOSE")
print("F: Elbow DROP | G: ARM DROP (+ OPEN)")
print("B: Ball Catch Manoeuvre")
print("V: Toggle Live Video")
print("Q: Quit")
print("----------------")

# 1. Start the Background Listener
sensor_thread = threading.Thread(target=listen_to_arduino)
sensor_thread.daemon = True # Kills thread if program crashes
sensor_thread.start()

# 2. Start the Main Keyboard Loop
current_cmd = b'S\n'
try:
    while True:
        # Increase timeout to 0.55s to bridge the OS keyboard auto-repeat delay
        # This prevents the robot from stopping internally while the key is held down
        key = get_key(0.55)
        cmd = None
        
        if key:
            key = key.lower()
            if key == 'w': cmd = b'F\n'
            elif key == 's': cmd = b'B\n'
            elif key == 'a': cmd = b'L\n'
            elif key == 'd': cmd = b'R\n'
            elif key == ' ': cmd = b'S\n'
            elif key == '1': cmd = b'1\n'
            elif key == '2': cmd = b'2\n'
            elif key == '3': cmd = b'3\n'
            elif key == 'u': cmd = b'A\n'
            elif key == 'j': cmd = b'a\n'  # Now mapped to the new staggered 'J' command
            elif key == 'h': cmd = b'H\n'
            elif key == 'f': cmd = b'f\n'
            elif key == 'g': cmd = b'g\n'
            elif key == 'b': 
                print("\n[BALL CATCH] Executing Manoeuvre...")
                ser.write(b'S\n') # Ensure stopped first
                time.sleep(0.1)
                ser.write(b'a\n') # Arm DOWN
                time.sleep(2.0)
                ser.write(b'O\n') # Gripper OPEN
                time.sleep(1.0)
                
                ser.write(b'F\n') # Move forward
                time.sleep(1.5) # Short interval
                ser.write(b'C\n') # Close gripper
                time.sleep(1.0) # Wait for close
                ser.write(b'S\n') # Stop
                time.sleep(0.5)
                
                for _ in range(3):
                    ser.write(b'F\n') # Move forwards
                    time.sleep(1.0)
                    ser.write(b'S\n') # Stop to scoop
                    time.sleep(0.2)
                    ser.write(b'O\n') # Open gripper
                    time.sleep(1.0)
                    ser.write(b'C\n') # Close gripper
                    time.sleep(1.0)
                    
                cmd = b'S\n'
                current_cmd = b'S\n'
                
            elif key == 'o': cmd = b'O\n'
            elif key == 'c': cmd = b'C\n'
            elif key == 'v': 
                cmd = current_cmd # Don't send anything new to Arduino
                if camera_process is None or camera_process.poll() is not None:
                    # Start camera script as a separate background process
                    # Use sys.executable to ensure we use the same Python environment (YOLO env)
                    camera_process = subprocess.Popen([sys.executable, '/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/arduino_motor_test/picamera_test.py'])
                    print("\n[CAMERA] Started Live Video!")
                else:
                    # Terminate if it's already running
                    camera_process.terminate()
                    camera_process.wait() # Wait for it to close cleanly
                    camera_process = None
                    print("\n[CAMERA] Stopped Live Video!")
            elif key == 'q': 
                ser.write(b'S\n')
                print("\nExecuting ARM DOWN (a) and Gripper OPEN (O)...")
                ser.write(b'a\n')
                time.sleep(2.0)
                ser.write(b'O\n')
                time.sleep(1.0)
                
                running = False # Stop the background thread
                if camera_process and camera_process.poll() is None:
                    camera_process.terminate()
                break # Exit the while loop
        
        # If a valid command was generated, send it and update current_cmd
        if key and cmd is not None:
            if cmd != current_cmd:
                # Debug line to prove Python is reading the key
                sys.stdout.write(f"\n[KEY PRESSED] Sending: {cmd.decode('utf-8').strip()}\n")
                sys.stdout.flush()
                ser.write(cmd)
                current_cmd = cmd
        elif not key:
            # If no key was pressed (timeout reached), stop the robot
            if current_cmd in [b'F\n', b'B\n', b'L\n', b'R\n']:
                sys.stdout.write("\n[TIMEOUT OR NO KEY] Sending: S\n")
                sys.stdout.flush()
                ser.write(b'S\n')
                current_cmd = b'S\n'

except KeyboardInterrupt:
    print("\nEmergency Stop")
    ser.write(b'S\n')
    
    print("Executing ARM DOWN (a) and Gripper OPEN (O)...")
    ser.write(b'a\n')
    time.sleep(2.0)
    ser.write(b'O\n')
    time.sleep(1.0)
    
    running = False
    if camera_process and camera_process.poll() is None:
        camera_process.terminate()

finally:
    ser.write(b'S\n')
    time.sleep(0.1)
    ser.close()
    print("Serial port closed. Exiting.")
