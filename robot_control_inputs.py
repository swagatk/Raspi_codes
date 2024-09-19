#!/usr/bin/env python3
"""
robot_control_inputs.py
www.bluetin.io
"""

__author__ = "Mark Heywood"
__version__ = "0.1.0"
__license__ = "MIT"

from inputs import get_gamepad

# Dictionary of game controller buttons we want to include.
controller_input = {'ABS_X': 0, 'ABS_RZ': 0, 'BTN_SOUTH': 0, 'BTN_WEST': 0}

#-----------------------------------------------------------

def gamepad_update():
    # Code execution stops at the following line until gamepad event occurs.
    events = get_gamepad()
    return_code = 'No Match'
    for event in events:
        event_test = controller_input.get(event.code, 'No Match')
        if event_test != 'No Match':
            controller_input[event.code] = event.state
            return_code = event.code
        else:
            return_code = 'No Match'

    return return_code

#-----------------------------------------------------------

def drive_control():
    # Function to drive robot motors
    print('Drive and Speed --> {} || Steering Angle --> {}' .format(controller_input['ABS_RZ'], controller_input['ABS_X']) )

#-----------------------------------------------------------

def fire_nerf_dart():
    # Function to fire Nerf dart gun on the robot
    print('Fire Nerf Dart --> {}' .format(controller_input['BTN_SOUTH']) )

#-----------------------------------------------------------

def led_beacon():
    # Function to switch led beacon on/off on the robot
    print('Switch LED Beacon --> {}' .format(controller_input['BTN_WEST']) )

#-----------------------------------------------------------

def main():
    """ Main entry point of the app """
    while True:
        # Get next controller Input
        control_code = gamepad_update()
        #print(control_code)
        
        # Gamepad button filter
        if control_code == 'ABS_X' or control_code == 'ABS_RZ':
            # Drive and steering
            drive_control()
        elif control_code == 'BTN_SOUTH':
            # Fire Nerf dart button
            fire_nerf_dart()
        elif control_code == 'BTN_WEST':
            # Switch the LED Beacon
            led_beacon()

#-----------------------------------------------------------

if __name__ == "__main__":
    """ This is executed when run from the command line """
    main()
