import cv2
import requests
import numpy as np

# --- CONFIGURATION ---
SERVER_URL = "http://192.168.1.188:8000/control"  # <--- REPLACE with Laptop IP
INSTRUCTION = "reach forward" # "Reach" often works better than "Move" for X-axis
SENSITIVITY = 60.0            # High multiplier because raw values are tiny (~0.002)

# --- SMOOTHING VARIABLES ---
# We store the last 3 actions to average them
history_linear = [0.0, 0.0, 0.0]
history_angular = [0.0, 0.0, 0.0]

def get_smooth_action(new_linear, new_angular):
    """Averages the last 3 frames to stop jitter."""
    global history_linear, history_angular
    
    # 1. Update History
    history_linear.pop(0)
    history_linear.append(new_linear)
    history_angular.pop(0)
    history_angular.append(new_angular)
    
    # 2. Calculate Average
    avg_linear = sum(history_linear) / len(history_linear)
    avg_angular = sum(history_angular) / len(history_angular)
    
    return avg_linear, avg_angular

def drive_motors(linear, angular):
    """
    CONVERT AI OUTPUT TO PHYSICAL MOTOR SIGNALS
    linear:  Positive = Forward, Negative = Backward
    angular: Positive = Right, Negative = Left (depending on wiring)
    """
    
    # 1. Apply Deadzone (Prevent motor humming)
    # If the signal is too weak, just stop.
    if abs(linear) < 0.15 and abs(angular) < 0.15:
        print("🛑 Stop (Deadzone)")
        # STOP MOTORS HERE (e.g. GPIO.output(LOW))
        return

    # 2. Mixing (Differential Drive Logic)
    # Left Wheel = Forward + Turn
    # Right Wheel = Forward - Turn
    left_speed = linear + angular
    right_speed = linear - angular

    # 3. Clamping (Ensure we don't exceed 100% speed)
    left_speed = max(min(left_speed, 1.0), -1.0)
    right_speed = max(min(right_speed, 1.0), -1.0)

    print(f"🚀 L: {left_speed:.2f} | R: {right_speed:.2f}")
    
    # --- ENTER YOUR GPIO CODE HERE ---
    # Example for common L298N driver:
    # if left_speed > 0:
    #    GPIO.output(IN1, HIGH); GPIO.output(IN2, LOW); pwm_left.ChangeDutyCycle(left_speed * 100)
    # ... etc

def main():
    cap = cv2.VideoCapture(0)
    # Low resolution for low latency over WiFi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print(f"Connecting to {SERVER_URL}...")

    while True:
        ret, frame = cap.read()
        if not ret: break

        _, img_encoded = cv2.imencode('.jpg', frame)
        
        try:
            # Send to Laptop
            response = requests.post(
                SERVER_URL, 
                files={"file": img_encoded.tobytes()},
                params={"instruction": INSTRUCTION},
                timeout=2.0 # Don't wait forever
            )
            
            if response.status_code == 200:
                data = response.json()
                
                # 1. Get Raw Data & Amplify
                raw_linear = data["linear"] * SENSITIVITY
                raw_angular = data["angular"] * SENSITIVITY
                
                # 2. Smooth the Data
                smooth_linear, smooth_angular = get_smooth_action(raw_linear, raw_angular)
                
                # 3. Drive
                drive_motors(smooth_linear, smooth_angular)
            
        except Exception as e:
            print(f"Lag spike or connection loss: {e}")

if __name__ == "__main__":
    main()
