#!/usr/bin/env python3
"""
Simple line sensor test over Arduino serial output.

Expected Arduino output examples (one per line):
  0
  1
  LINE,0
  LINE,1
  LOW / HIGH
  BLACK / WHITE

If your sensor logic is inverted, use --black-high.
"""

import argparse
import re
import sys
import time

import serial
import serial.tools.list_ports


def find_arduino_port() -> str | None:
	"""Auto-detect likely Arduino serial port."""
	ports = serial.tools.list_ports.comports()
	for port in ports:
		if "ttyACM" in port.device or "ttyUSB" in port.device:
			return port.device
	return None


def parse_sensor_state(line: str) -> int | None:
	"""Return digital state as 0/1 when possible, else None."""
	cleaned = line.strip().upper()
	if not cleaned:
		return None

	if "BLACK" in cleaned:
		return 0
	if "WHITE" in cleaned:
		return 1
	if "LOW" in cleaned:
		return 0
	if "HIGH" in cleaned:
		return 1

	# Accept lines like "0", "1", "LINE,0", "S:1"
	match = re.search(r"(^|[^0-9])([01])($|[^0-9])", cleaned)
	if match:
		return int(match.group(2))

	return None


def state_to_surface(state: int, black_is_low: bool) -> str:
	is_black = (state == 0) if black_is_low else (state == 1)
	return "BLACK" if is_black else "WHITE"


def main() -> int:
	parser = argparse.ArgumentParser(description="Line sensor black/white serial test")
	parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyACM0)")
	parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")

	mapping = parser.add_mutually_exclusive_group()
	mapping.add_argument("--black-low", dest="black_is_low", action="store_true", default=True,
						 help="Interpret digital LOW (0) as BLACK (default)")
	mapping.add_argument("--black-high", dest="black_is_low", action="store_false",
						 help="Interpret digital HIGH (1) as BLACK")

	args = parser.parse_args()

	port = args.port or find_arduino_port()
	if not port:
		print("No Arduino serial port found. Use --port /dev/ttyACM0")
		return 1

	try:
		ser = serial.Serial(port, args.baud, timeout=1)
		ser.reset_input_buffer()
		print(f"Connected to {port} @ {args.baud}")
		print("Waiting for sensor values... Press Ctrl+C to stop.")
		print(f"Mapping: {'0=BLACK, 1=WHITE' if args.black_is_low else '1=BLACK, 0=WHITE'}")
		time.sleep(1.5)  # Let Arduino finish reset after opening serial.
	except Exception as exc:
		print(f"Failed to open serial port: {exc}")
		return 1

	try:
		while True:
			raw = ser.readline().decode("utf-8", errors="ignore").strip()
			if not raw:
				continue

			state = parse_sensor_state(raw)
			if state is None:
				continue

			surface = state_to_surface(state, args.black_is_low)
			print(surface)
	except KeyboardInterrupt:
		print("\nStopped.")
	finally:
		ser.close()

	return 0


if __name__ == "__main__":
	sys.exit(main())
