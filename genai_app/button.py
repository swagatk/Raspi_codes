from gpiozero import Button
from signal import pause

# GPIO pin setup
button = Button(16, bounce_time=0.1)

def button_pressed():
    print("Button is pressed")

if __name__ == '__main__':
    button.when_pressed = button_pressed
    try:
        pause()
    except KeyboardInterrupt:
        print('Exiting Program')
    finally:
        button.close()
        print('GPIO cleaned up')
