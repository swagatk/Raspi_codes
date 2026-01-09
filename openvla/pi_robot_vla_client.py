import time
import cv2
import requests
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURATION ---
SERVER_URL = "http://192.168.1.XX:8000/control"  # <--- Update IP
INSTRUCTION = "reach forward"
SENSITIVITY = 100.0   # Keep this high to boost the signal

# Thresholds (Tweak these based on your logs)
TURN_THRESHOLD = 0.20   # How strong the turn signal needs to be to trigger State 1 or 2
MOVE_THRESHOLD = 0.15   # How strong the fwd signal needs to be to trigger State 0

# --- DISCRETE ROBOT FUNCTIONS ---
def move_forward():
    print("🚗 ACTION: [0] FORWARD")
    # Add your real code: e.g., robot.forward()

def turn_right():
    print("↪️ ACTION: [1] TURN RIGHT")
    # Add your real code: e.g., robot.right()

def turn_left():
    print("↩️ ACTION: [2] TURN LEFT")
    # Add your real code: e.g., robot.left()

def stop_robot():
    print("🛑 ACTION: STOP")
    # Add your real code: e.g., robot.stop()

# --- DECISION ENGINE ---
def execute_discrete_command(linear, angular):
    """
    Decides which SINGLE function to call based on linear/angular values.
    Prioritizes turning for safety.
    """
    # 1. Check for Turning (Highest Priority)
    # In our previous math (Left = Lin + Ang), Positive Angular means Left Wheel > Right Wheel -> Turn Right
    if angular > TURN_THRESHOLD:
        turn_right()  # State 1
        return
    elif angular < -TURN_THRESHOLD:
        turn_left()   # State 2
        return

    # 2. Check for Forward (Secondary Priority)
    if linear > MOVE_THRESHOLD:
        move_forward() # State 0
        return
    
    # 3. Default to Stop
    # This catches deadzones or backward movements (which we ignore for now)
    stop_robot()


# --- SMOOTHING ---
history_linear = [0.0, 0.0, 0.0]
history_angular = [0.0, 0.0, 0.0]

def get_smooth_action(new_linear, new_angular):
    global history_linear, history_angular
    history_linear.pop(0); history_linear.append(new_linear)
    history_angular.pop(0); history_angular.append(new_angular)
    return sum(history_linear)/3, sum(history_angular)/3

# --- MAIN LOOP ---
def main():
    print("Initializing Picamera2 for Discrete Control...")
    picam2 = Picamera2()
    config = picam2.create_configuration(main={"size": (320, 240), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()
    
    print(f"✅ Connected to {SERVER_URL}")
    time.sleep(2)

    try:
        while True:
            frame = picam2.capture_array()
            _, img_encoded = cv2.imencode('.jpg', frame)
            
            try:
                response = requests.post(
                    SERVER_URL, 
                    files={"file": img_encoded.tobytes()},
                    params={"instruction": INSTRUCTION},
                    timeout=2.0 
                )
                
                if response.status_code == 200:
                    data = response.json()
                    
                    # 1. Get & Smooth Data
                    raw_lin = data["linear"] * SENSITIVITY
                    raw_ang = data["angular"] * SENSITIVITY
                    
                    # Note: We boost forward signal slightly to bias towards driving
                    smooth_lin, smooth_ang = get_smooth_action(raw_lin * 1.2, raw_ang)
                    
                    # 2. Debug Print (Optional, to help tune thresholds)
                    # print(f"DEBUG: Lin={smooth_lin:.2f} | Ang={smooth_ang:.2f}")

                    # 3. Execute Discrete Action
                    execute_discrete_command(smooth_lin, smooth_ang)
                
            except Exception as e:
                print(f"⚠️ Network Lag: {e}")
                stop_robot()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()
