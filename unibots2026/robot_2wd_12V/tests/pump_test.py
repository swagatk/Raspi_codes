from gpiozero import OutputDevice
from time import sleep

# --- CONFIGURATION ---
# We defined GPIO 26 for the relay earlier
# active_high=True means sending "1" turns it ON. 
# If your relay works backwards (ON when it should be OFF), change this to False.
relay = OutputDevice(26, active_high=True, initial_value=False)

try:
    print("Testing Vacuum Pump...")
    
    # 1. Turn Pump ON
    print("Pump ON - Suction started")
    relay.on()
    sleep(5)  # Let it run for 5 seconds

    # 2. Turn Pump OFF
    print("Pump OFF - Dropping object")
    relay.off()
    sleep(2)
    
    print("Test Complete.")

except KeyboardInterrupt:
    print("Emergency Stop!")
    relay.off()
