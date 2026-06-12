#!/usr/bin/env python3
"""
Simple line sensor test over Arduino serial output.

Expected Arduino output examples (one per line):
	LINE,0,1
	LINE,1,0
	(legacy single-sensor packets LINE,0 or LINE,1 are also accepted)

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


def parse_single_line_state(line: str) -> int | None:
	"""Parse legacy LINE,<state> packets and return 0/1."""
	cleaned = line.strip().upper()
	if not cleaned.startswith("LINE,"):
		return None

	parts = [p.strip() for p in cleaned.split(",")]
	if len(parts) != 2:
		return None

	if parts[1] in ("0", "1"):
		return int(parts[1])

	return None


def state_to_surface(state: int, black_is_low: bool) -> str:
	is_black = (state == 0) if black_is_low else (state == 1)
	return "BLACK" if is_black else "WHITE"


def parse_dual_line_states(line: str) -> tuple[int, int] | None:
	"""Parse LINE,left,right packets. Returns (left_state, right_state)."""
	cleaned = line.strip().upper()
	if not cleaned.startswith("LINE,"):
		return None

	parts = [p.strip() for p in cleaned.split(",")]
	if len(parts) != 3:
		return None

	if parts[1] in ("0", "1") and parts[2] in ("0", "1"):
		return int(parts[1]), int(parts[2])

	return None


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
		last_left_state = 1
		last_right_state = 1
		while True:
			raw = ser.readline().decode("utf-8", errors="ignore").strip()
			if not raw:
				continue

			dual_states = parse_dual_line_states(raw)
			if dual_states is not None:
				left_state, right_state = dual_states
				last_left_state, last_right_state = left_state, right_state
				left_surface = state_to_surface(left_state, args.black_is_low)
				right_surface = state_to_surface(right_state, args.black_is_low)
				print(f"LEFT={left_surface} RIGHT={right_surface}")
				continue

			single_state = parse_single_line_state(raw)
			if single_state is None:
				continue

			# Backward compatibility: mirror single state to both sensors.
			last_left_state = single_state
			last_right_state = single_state
			left_surface = state_to_surface(last_left_state, args.black_is_low)
			right_surface = state_to_surface(last_right_state, args.black_is_low)
			print(f"LEFT={left_surface} RIGHT={right_surface}")
	except KeyboardInterrupt:
		print("\nStopped.")
	finally:
		ser.close()

	return 0


if __name__ == "__main__":
	sys.exit(main())
