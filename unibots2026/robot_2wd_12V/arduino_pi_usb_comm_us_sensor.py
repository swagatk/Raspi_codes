import serial
import time

# Connect to Arduino (usually /dev/ttyUSB0 or /dev/ttyACM0)
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
except:
    print("Arduino not found. Check USB connection.")

def get_sensor_data():
    if ser.in_waiting > 0:
        try:
            # Read line, strip whitespace, decode bytes to string
            line = ser.readline().decode('utf-8').rstrip()
            
            # Split "10,25,5" into [10, 25, 5]
            distances = [int(x) for x in line.split(",")]
            
            if len(distances) == 3:
                return distances # Returns [Left, Center, Right]
        except ValueError:
            pass # Ignore corrupted lines
    return None

# --- Main Robot Loop ---
while True:
    data = get_sensor_data()
    
    if data:
        left, center, right = data
        print(f"L: {left}cm | C: {center}cm | R: {right}cm")
        
        # Simple Obstacle Avoidance Logic
        if center < 20:
            print("OBSTACLE AHEAD! Stopping.")
            # stop_motors()
        elif left < 15:
            print("Too close to Left Wall - Turn Right")
    
    time.sleep(0.1)
