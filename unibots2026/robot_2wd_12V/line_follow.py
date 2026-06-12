#!/usr/bin/env python3
"""Autonomous line following using two digital line sensors over serial.

Expected Arduino line packet:
	LINE,left,right
where left/right are 0 or 1.

Logic:
1) Left=WHITE, Right=WHITE -> move forward.
2) Left=BLACK, Right=WHITE -> slight right nudge.
3) Left=WHITE, Right=BLACK -> slight left nudge.
4) Left=BLACK, Right=BLACK -> alternate left/right turns to find line edge.
"""

import argparse
import sys
import time

import serial
import serial.tools.list_ports


BAUD_RATE = 115200
POLL_DELAY_S = 0.03
ARM_MOVE_DELAY_S = 2.0
SERIAL_RESET_DELAY_S = 2.0
SENSOR_TIMEOUT_S = 0.8

DRIVE_SPEED_CMD = "1"
NUDGE_SECONDS = 0.10
NUDGE_PAUSE_SECONDS = 0.05
BOTH_BLACK_SWITCH_SECONDS = 0.30


def find_arduino_port() -> str | None:
	ports = serial.tools.list_ports.comports()
	for port in ports:
		if "ttyACM" in port.device or "ttyUSB" in port.device:
			return port.device
	return None


def parse_dual_line_states(line: str) -> tuple[int, int] | None:
	"""Parse LINE,left,right packet and return integer states."""
	cleaned = line.strip().upper()
	if not cleaned.startswith("LINE,"):
		return None

	parts = [p.strip() for p in cleaned.split(",")]
	if len(parts) != 3:
		return None

	if parts[1] in ("0", "1") and parts[2] in ("0", "1"):
		return int(parts[1]), int(parts[2])

	return None


def state_to_surface(state: int, black_is_low: bool) -> str:
	is_black = (state == 0) if black_is_low else (state == 1)
	return "BLACK" if is_black else "WHITE"


def read_dual_line_surfaces(ser: serial.Serial, black_is_low: bool) -> tuple[str, str] | None:
	if ser.in_waiting <= 0:
		return None

	raw = ser.readline().decode("utf-8", errors="ignore").strip()
	states = parse_dual_line_states(raw)
	if states is None:
		return None

	left_state, right_state = states
	return state_to_surface(left_state, black_is_low), state_to_surface(right_state, black_is_low)


def send_drive_cmd(ser: serial.Serial, cmd: str, current_cmd: str) -> str:
	if cmd != current_cmd:
		ser.write((cmd + "\n").encode("utf-8"))
		return cmd
	return current_cmd


def main() -> int:
	parser = argparse.ArgumentParser(description="Follow a black line on white surface")
	parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyACM0)")
	parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate")
	parser.add_argument(
		"--speed-cmd",
		default=DRIVE_SPEED_CMD,
		help="Speed command to send to Arduino (typically 1, 2, or 3)",
	)
	parser.add_argument(
		"--nudge-seconds",
		type=float,
		default=NUDGE_SECONDS,
		help="Duration for slight correction nudges when one sensor sees BLACK",
	)
	parser.add_argument(
		"--nudge-pause-seconds",
		type=float,
		default=NUDGE_PAUSE_SECONDS,
		help="Pause duration after each nudge",
	)
	parser.add_argument(
		"--both-black-switch-seconds",
		type=float,
		default=BOTH_BLACK_SWITCH_SECONDS,
		help="Switch interval while alternating L/R when both sensors are BLACK",
	)

	mapping = parser.add_mutually_exclusive_group()
	mapping.add_argument(
		"--black-low",
		dest="black_is_low",
		action="store_true",
		default=True,
		help="Interpret digital LOW (0) as BLACK (default)",
	)
	mapping.add_argument(
		"--black-high",
		dest="black_is_low",
		action="store_false",
		help="Interpret digital HIGH (1) as BLACK",
	)

	args = parser.parse_args()

	port = args.port or find_arduino_port()
	if not port:
		print("Error: No Arduino serial port found (checked ttyACM* and ttyUSB*).")
		return 1

	try:
		ser = serial.Serial(port, args.baud, timeout=0.05)
		time.sleep(SERIAL_RESET_DELAY_S)
		ser.reset_input_buffer()
	except Exception as exc:
		print(f"Error: Failed to open serial port: {exc}")
		return 1

	current_cmd = "S"
	last_valid_sensor_time = time.monotonic()
	both_black_cmd = "L"
	last_both_black_switch = time.monotonic()
	nudge_cmd = None
	nudge_until = 0.0
	pause_until = 0.0
	left_surface = "WHITE"
	right_surface = "WHITE"
	last_surface_time = time.monotonic()

	print(f"Connected to {port} @ {args.baud}")
	print(f"Mapping: {'0=BLACK, 1=WHITE' if args.black_is_low else '1=BLACK, 0=WHITE'}")
	print("Sending arm to UP pose...")
	ser.write(b"A\n")
	time.sleep(ARM_MOVE_DELAY_S)

	print("Setting speed and starting dual-sensor line follow...")
	ser.write((args.speed_cmd + "\n").encode("utf-8"))
	current_cmd = send_drive_cmd(ser, "S", current_cmd)
	print("Press Ctrl+C to stop.")

	try:
		while True:
			now = time.monotonic()
			surfaces = read_dual_line_surfaces(ser, args.black_is_low)
			if surfaces is not None:
				left_surface, right_surface = surfaces
				last_valid_sensor_time = now
				last_surface_time = now

				if left_surface == "WHITE" and right_surface == "WHITE":
					nudge_cmd = None
					nudge_until = 0.0
					pause_until = 0.0
					current_cmd = send_drive_cmd(ser, "F", current_cmd)

				elif left_surface == "BLACK" and right_surface == "WHITE":
					nudge_cmd = "R"
					nudge_until = now + args.nudge_seconds
					pause_until = nudge_until + args.nudge_pause_seconds

				elif left_surface == "WHITE" and right_surface == "BLACK":
					nudge_cmd = "L"
					nudge_until = now + args.nudge_seconds
					pause_until = nudge_until + args.nudge_pause_seconds

				else:
					nudge_cmd = None
					nudge_until = 0.0
					pause_until = 0.0
					if now - last_both_black_switch >= args.both_black_switch_seconds:
						both_black_cmd = "R" if both_black_cmd == "L" else "L"
						last_both_black_switch = now
					current_cmd = send_drive_cmd(ser, both_black_cmd, current_cmd)

			if now - last_valid_sensor_time > SENSOR_TIMEOUT_S:
				current_cmd = send_drive_cmd(ser, "S", current_cmd)
			else:
				if nudge_cmd is not None:
					if now < nudge_until:
						current_cmd = send_drive_cmd(ser, nudge_cmd, current_cmd)
					elif now < pause_until:
						current_cmd = send_drive_cmd(ser, "S", current_cmd)
					else:
						nudge_cmd = None
						current_cmd = send_drive_cmd(ser, "F", current_cmd)

			sys.stdout.write(
				f"\rLeft:{left_surface:<5} Right:{right_surface:<5} Cmd:{current_cmd}     "
			)
			sys.stdout.flush()

			time.sleep(POLL_DELAY_S)

	except KeyboardInterrupt:
		print("\nStopping (KeyboardInterrupt)...")
	finally:
		try:
			ser.write(b"S\n")
			time.sleep(0.2)
			ser.write(b"a\n")
			time.sleep(ARM_MOVE_DELAY_S)
			ser.close()
		except Exception:
			pass
		print("Robot stopped. Serial closed.")

	return 0


if __name__ == "__main__":
	sys.exit(main())
