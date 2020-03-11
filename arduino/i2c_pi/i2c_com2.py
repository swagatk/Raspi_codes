import smbus
import time

bus = smbus.SMBus(1)
address = 0x08

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

# infinite loop
while True:
    var = input("Enter 1-9:")
    if int(var) > 9 or int(var) < 1:
        print("Invalid number. Exiting")
        break
    
    writeNumber(int(var))
    print("RPi: Hi Arduino, I sent you: ", var)
    time.sleep(1)
    number = readNumber()
    print("Arduino: Hey RPi, I received a digit: ",number)
    
