# Joystick interface with pygame
import pygame
import time
import sys
import os
pygame.init
pygame.joystick.init()

# Get count of joysticks
joystick_count = pygame.joystick.get_count()

# Wait until joystick is connected
while joystick_count < 1:
    print("Joystick not connected")
    time.sleep(1)

print('Found {} joysticks'.format(joystick_count))

joystick = pygame.joystick.Joystick(0)
joystick.init()

name = joystick.get_name()
print("Joystick name: {}".format(name))

axes = joystick.get_numaxes()
print("Number of axes: {}".format(axes))

buttons = joystick.get_numbuttons()
print("Number of buttons: ", buttons)

hats = joystick.get_numhats()
print("Number of hats: {}".format(hats))




for i in range(axes):
    axis = joystick.get_axis(i)
    print("Axis {} value: {:>6.3f}".format(i, axis))


for i in range(buttons):
    button = joystick.get_button(i)
    print("Button {:>2} value: {}".format(i, button))


for i in range(hats):
    hat = joystick.get_hat(i)
    print("Hat {} value: {}".format(i, str(hat)))


# quit
pygame.joystick.quit()
