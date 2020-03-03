# Testing Explorer Hat functionality

import explorerhat as eh
import time
import pygame

pygame.init()

# sets the window title
pygame.display.set_caption(u'Keyboard events')

# sets the window size
pygame.display.set_mode((400, 400))

def readkey():
    event = pygame.event.wait()
    
    # if the 'close' button of the window is pressed
    if event.type == pygame.QUIT:
        # stops the application
        return "STOP"
    
    # captures the 'KEYDOWN' and 'KEYUP' events
    if event.type in (pygame.KEYDOWN, pygame.KEYUP):
        # gets the key name
        key_name = pygame.key.name(event.key)

        # converts to uppercase the key name
        key_name = key_name.upper()

        # if any key is pressed
        if event.type == pygame.KEYDOWN:
            # prints on the console the key pressed
            #print(u'"{}" key pressed'.format(key_name))
            pass

        # if any key is released
        elif event.type == pygame.KEYUP:
            # prints on the console the released key
            #print(u'"{}" key released'.format(key_name))
            return(key_name.upper())

            
while True:
    
    key = readkey()

    if key == 'STOP':
        print("stop. Quit Program")
        eh.motor.stop()
        break
    elif key == 'ESCAPE':
        print("Quit Program")
        eh.motor.stop()
        break
    elif key == 'F':
        print("forward")
        eh.motor.forwards()
        time.sleep(0.5)
    elif key == 'B':
        print('Backward')
        eh.motor.backwards()
        time.sleep(0.5)
    elif key == 'L':
        print("turn left")
        eh.motor.one.forwards(30)
        eh.motor.two.forwards(100)
        time.sleep(0.5)
    elif key == 'R':
        eh.motor.one.forwards(100)
        eh.motor.two.forwards(30)
        time.sleep(0.5)
    elif key == 'SPACE':
        print('Stop')
        eh.motor.stop()
        time.sleep(0.5)
    else:
        print(key)
        #print('Valid Choices: F,B,L,R, Space, Esc')

    

pygame.quit()
