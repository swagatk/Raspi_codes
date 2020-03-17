import spidev
import time

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=3900000
while True:
    resp=spi.xfer([0xAB])
    print(resp)
    time.sleep(0.5)
    
