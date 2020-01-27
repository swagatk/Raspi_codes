# Morse code

# Import Libraries
import os # Gives Python access to Linux commands
import time # Proves time related commands
import RPi.GPIO as GPIO # Gives access to the GPIO Pins

# Set the GPIO pin naming mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# variables
PINBuzzer = 22 # Sets the buzzer pin 22
LEDPin = 18

# PIN configuration

GPIO.setup(PINBuzzer, GPIO.OUT)
GPIO.output(PINBuzzer, GPIO.LOW)

GPIO.setup(LEDPin, GPIO.OUT)
GPIO.output(LEDPin, GPIO.LOW)

def dot(): # A single Morse dot
    GPIO.output(PINBuzzer, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(PINBuzzer, GPIO.LOW)
    time.sleep(0.1)
    
def dash(): # A single Morse dash
    GPIO.output(PINBuzzer, GPIO.HIGH)
    time.sleep(0.3)
    GPIO.output(PINBuzzer, GPIO.LOW)
    time.sleep(0.1)
    
def letterspace(): # The space between letters
    time.sleep(0.2)
    
def wordspace(): # The space between words
    time.sleep(0.6)



def morse_a():
    dot()
    dash()

def morse_b():
    dash()
    dot()
    dot()
    dot()

def morse_c():
    dash()
    dot()
    dash()
    dot()

def morse_d():
    dash()
    dot()
    dot()

def morse_e():
    dot()

def morse_f():
    dot()
    dot()
    dash()
    dot()

def morse_g():
    dash()
    dash()
    dot()

def morse_h():
    dot()
    dot()
    dot()
    dot()

def morse_i():
    dot()
    dot()

def morse_j():
    dot()
    dash()
    dash()
    dash()

def morse_k():
    dash()
    dot()
    dash()

def morse_l():
    dot()
    dash()
    dot()
    dot()

def morse_m():
    dash()
    dash()

def morse_n():
    dash()
    dot()

def morse_o():
    dash()
    dash()
    dash()

def morse_p():
    dot()
    dash()
    dash()
    dot()

def morse_q():
    dash()
    dash()
    dot()
    dash()

def morse_r():
    dot()
    dash()
    dot()

def morse_s(): 
    dot()
    dot()
    dot()

def morse_t():
    dash()

def morse_u():
    dot()
    dot()
    dash()

def morse_v():
    dot()
    dot()
    dot()
    dash()

def morse_w():
    dot()
    dash()
    dash()

def morse_x():
    dash()
    dot()
    dot()
    dash()

def morse_y():
    dash()
    dot()
    dash()
    dash()

def morse_z():
    dash()
    dash()
    dot()
    dot()

def morse_0():
    dash()
    dash()
    dash()
    dash()
    dash()

def morse_1():
    dot()
    dash()
    dash()
    dash()
    dash()

def morse_2():
    dot()
    dot()
    dash()
    dash()
    dash()

def morse_3():
    dot()
    dot()
    dot()
    dash()
    dash()

def morse_4():
    dot()
    dot()
    dot()
    dot()
    dash()

def morse_5():
    dot()
    dot()
    dot()
    dot()
    dot()

def morse_6():
    dash()
    dot()
    dot()
    dot()
    dot()

def morse_7():
    dash()
    dash()
    dot()
    dot()
    dot()

def morse_8():
    dash()
    dash()
    dash()
    dot()
    dot()

def morse_9():
    dash()
    dash()
    dash()
    dash()
    dot()




        
