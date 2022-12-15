from smbus import SMBus
import sys

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
bus.write_byte(addr, 0x1) # switch it on

if sys.version_info[0] < 3:
    raw_input("Check that the Arduino LED is glowing.\n Now, Press return turn off the LED")
else:
    input('Check that the Arduino LED is glowing.\n Press return to turn off the LED')
bus.write_byte(addr, 0x0) # switch it off
