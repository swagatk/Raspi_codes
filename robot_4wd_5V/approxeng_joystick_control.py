from approxeng.input.selectbinder import ControllerResource
from motor_control_4wd import move_forward, move_backward, turn_left, turn_right, stop, initialize_motors
import time

def joystick_control():

	# --- Control Logic ---
	try:
		# ControllerResource automatically finds the Pi Hut controller
		with ControllerResource() as joystick:
			print("Controller found and connected!")
			
			while joystick.connected:
				# Get axis values (ranges from -1.0 to 1.0)
				# lx: -1.0 is full left, 1.0 is full right
				# ly: 1.0 is full forward (up), -1.0 is full backward (down)
				lx, ly = joystick['lx', 'ly']

				# Threshold mapping (Deadzone)
				# We use 0.5 to ensure intentional movement triggers the function
				if ly > 0.5:
					move_forward()
				elif ly < -0.5:
					move_backward()
				elif lx < -0.5:
					turn_left(power=100)
				elif lx > 0.5:
					turn_right(power=100)
				else:
					stop()

				# Small sleep to prevent high CPU usage
				time.sleep(0.05)

	except IOError:
		print("No controller found. Check the USB dongle and batteries.")

if __name__ == '__main__':
	initialize_motors()
	joystick_control()
