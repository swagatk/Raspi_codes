#!/usr/bin/env python3

import time

try:
	from smbus2 import SMBus
except ImportError:
	from smbus import SMBus


MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43

# Default full-scale gyro range is +/-250 deg/s after power-up.
GYRO_SENSITIVITY_LSB_PER_DPS = 131.0


def read_word_2c(bus, reg):
	high = bus.read_byte_data(MPU6050_ADDR, reg)
	low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
	value = (high << 8) | low
	if value >= 0x8000:
		value = -((65535 - value) + 1)
	return value


def main():
	bus_num = 1  # Raspberry Pi I2C-1 (SDA=GPIO2 pin 3, SCL=GPIO3 pin 5)

	with SMBus(bus_num) as bus:
		# Wake up MPU-6050 (it starts in sleep mode).
		bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
		time.sleep(0.1)

		print("Reading gyroscope from MPU-6050...")
		print("Units: raw (LSB), converted (deg/s)")
		print("Press Ctrl+C to stop.\n")

		while True:
			gx_raw = read_word_2c(bus, GYRO_XOUT_H)
			gy_raw = read_word_2c(bus, GYRO_XOUT_H + 2)
			gz_raw = read_word_2c(bus, GYRO_XOUT_H + 4)

			gx_dps = gx_raw / GYRO_SENSITIVITY_LSB_PER_DPS
			gy_dps = gy_raw / GYRO_SENSITIVITY_LSB_PER_DPS
			gz_dps = gz_raw / GYRO_SENSITIVITY_LSB_PER_DPS

			print(
				f"GX: {gx_raw:6d} LSB ({gx_dps:8.2f} deg/s) | "
				f"GY: {gy_raw:6d} LSB ({gy_dps:8.2f} deg/s) | "
				f"GZ: {gz_raw:6d} LSB ({gz_dps:8.2f} deg/s)"
			)
			time.sleep(0.2)


if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		print("\nStopped.")
