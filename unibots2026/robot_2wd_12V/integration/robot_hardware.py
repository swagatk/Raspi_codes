import serial
import serial.tools.list_ports
import time
import threading
from config import *

class RobotController:
    def __init__(self):
        self.ser = None
        self.latest_L = 999
        self.latest_C = 999
        self.latest_R = 999
        self.running = False
        
        self.connect()

    def connect(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'ttyACM' in port.device or 'ttyUSB' in port.device:
                self.ser = serial.Serial(port.device, 115200, timeout=0)
                self.ser.reset_input_buffer()
                time.sleep(2)
                return True
        return False

    def start_sensors(self):
        self.running = True
        threading.Thread(target=self._sensor_loop, daemon=True).start()

    def _sensor_loop(self):
        while self.running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').rstrip()
                    if line.startswith("D,"):
                        parts = line.split(",")
                        if len(parts) == 4:
                            self.latest_L = int(parts[1])
                            self.latest_C = int(parts[2])
                            self.latest_R = int(parts[3])
                except:
                    pass
            time.sleep(0.01)

    def stop(self):
        self.running = False
        if self.ser:
            if self.ser.is_open:
                try:
                    self.ser.write(b'S\n')
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    def send_cmd(self, cmd):
        if self.ser:
            if isinstance(cmd, str):
                self.ser.write(cmd.encode() + b'\n')
            else:
                self.ser.write(cmd)

    def move(self, dir_char, speed='2'):
        self.send_cmd(speed)
        time.sleep(0.05)
        self.send_cmd(dir_char)

    def arm_up(self):
        self.send_cmd('A')

    def arm_down(self):
        self.send_cmd('a')

    def arm_vertical(self):
        self.send_cmd('H')

    def arm_drop(self):
        self.send_cmd('g')

    def gripper_open(self):
        self.send_cmd('O')

    def gripper_close(self):
        self.send_cmd('C')

    def halt(self):
        self.send_cmd('S')
