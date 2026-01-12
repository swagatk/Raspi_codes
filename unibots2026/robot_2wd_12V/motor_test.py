from gpiozero import Motor
from time import sleep

# --- CONFIGURATION ---
# We define the pins connected to the L298N
# 'forward' and 'backward' correspond to IN1/IN2 or IN3/IN4
left_motor = Motor(forward=17, backward=27)
right_motor = Motor(forward=22, backward=23)

print("Starting Motor Test...")

try:
    # 1. Move Forward at 50% Speed (PWM)
    print("Moving Forward - 50% Speed")
    left_motor.forward(speed=0.5)
    right_motor.forward(speed=0.5)
    sleep(2)

    # 2. Stop
    print("Stopping")
    left_motor.stop()
    right_motor.stop()
    sleep(1)

    # 3. Move Backward at 100% Speed
    print("Moving Backward - 100% Speed")
    left_motor.backward(speed=1.0)
    right_motor.backward(speed=1.0)
    sleep(2)

    # 4. Turn (Left Motor Forward, Right Motor Backward)
    print("Spinning Right")
    left_motor.forward(speed=0.6)
    right_motor.backward(speed=0.6)
    sleep(2)

    # 5. Stop Everything
    left_motor.stop()
    right_motor.stop()
    print("Test Complete.")

except KeyboardInterrupt:
    # This stops the motors if you press Ctrl+C
    print("Emergency Stop!")
    left_motor.stop()
    right_motor.stop()
