import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
while True:

    if(ser.in_waiting > 0):
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)
