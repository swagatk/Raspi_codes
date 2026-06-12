#!/usr/bin/env python3
"""Autonomous line following using two digital line sensors over serial.

Expected Arduino line packet:
	LINE,left,right
where left/right are 0 or 1.

Logic:
1) Left=WHITE, Right=WHITE -> move forward.
2) Left=BLACK, Right=WHITE -> slight left nudge.
3) Left=WHITE, Right=BLACK -> slight right nudge.
4) Left=BLACK, Right=BLACK -> move forward slowly (anti-oscillation on curves).
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

FORWARD_SPEED_CMD = "1"
TURN_SPEED_CMD = "1"
FORWARD_MOVE_SECONDS = 0.06
FORWARD_PAUSE_SECONDS = 0.12
NUDGE_SECONDS = 0.08
NUDGE_PAUSE_SECONDS = 0.08
BOTH_BLACK_SPEED_CMD = "1"
BOTH_BLACK_MOVE_SECONDS = 0.04
BOTH_BLACK_PAUSE_SECONDS = 0.14
OSCILLATION_THRESHOLD = 3
OSCILLATION_FORWARD_SECONDS = 0.20


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


def send_speed_cmd(ser: serial.Serial, speed_cmd: str, current_speed_cmd: str) -> str:
	if speed_cmd != current_speed_cmd:
		ser.write((speed_cmd + "\n").encode("utf-8"))
		return speed_cmd
	return current_speed_cmd


def main() -> int:
	parser = argparse.ArgumentParser(description="Follow a black line on white surface")
	parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyACM0)")
	parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate")
	parser.add_argument(
		"--forward-speed-cmd",
		"--speed-cmd",
		dest="forward_speed_cmd",
		default=FORWARD_SPEED_CMD,
		help="Forward speed command to send to Arduino (typically 1, 2, or 3)",
	)
	parser.add_argument(
		"--turn-speed-cmd",
		default=TURN_SPEED_CMD,
		help="Turn speed command for left/right nudges and edge search (typically 1, 2, or 3)",
	)
	parser.add_argument(
		"--both-black-speed-cmd",
		default=BOTH_BLACK_SPEED_CMD,
		help="Speed command when both sensors are BLACK (slow curve mode)",
	)
	parser.add_argument(
		"--forward-move-seconds",
		type=float,
		default=FORWARD_MOVE_SECONDS,
		help="Forward pulse duration when Left=WHITE and Right=WHITE",
	)
	parser.add_argument(
		"--forward-pause-seconds",
		type=float,
		default=FORWARD_PAUSE_SECONDS,
		help="Pause duration between forward pulses",
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
		"--both-black-move-seconds",
		type=float,
		default=BOTH_BLACK_MOVE_SECONDS,
		help="Forward pulse duration when both sensors are BLACK",
	)
	parser.add_argument(
		"--both-black-pause-seconds",
		type=float,
		default=BOTH_BLACK_PAUSE_SECONDS,
		help="Pause duration between forward pulses when both sensors are BLACK",
	)
	parser.add_argument(
		"--oscillation-threshold",
		type=int,
		default=OSCILLATION_THRESHOLD,
		help="Number of L/R correction flips before forward recovery is triggered",
	)
	parser.add_argument(
		"--oscillation-forward-seconds",
		type=float,
		default=OSCILLATION_FORWARD_SECONDS,
		help="Forward recovery duration after oscillation threshold is hit",
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
	current_speed_cmd = ""
	last_valid_sensor_time = time.monotonic()
	nudge_cmd = None
	nudge_until = 0.0
	pause_until = 0.0
	forward_phase = "MOVE"
	last_forward_phase_change = time.monotonic()
	both_black_phase = "MOVE"
	last_both_black_phase_change = time.monotonic()
	left_surface = "WHITE"
	right_surface = "WHITE"
	last_turn_dir = None
	oscillation_flips = 0
	oscillation_recovery_until = 0.0

	print(f"Connected to {port} @ {args.baud}")
	print(f"Mapping: {'0=BLACK, 1=WHITE' if args.black_is_low else '1=BLACK, 0=WHITE'}")
	print("Sending arm to UP pose...")
	ser.write(b"A\n")
	time.sleep(ARM_MOVE_DELAY_S)

	print("Setting speed and starting dual-sensor line follow...")
	current_speed_cmd = send_speed_cmd(ser, args.forward_speed_cmd, current_speed_cmd)
	current_cmd = send_drive_cmd(ser, "S", current_cmd)
	print("Press Ctrl+C to stop.")

	try:
		while True:
			now = time.monotonic()
			surfaces = read_dual_line_surfaces(ser, args.black_is_low)
			if surfaces is not None:
				left_surface, right_surface = surfaces
				last_valid_sensor_time = now

			if now - last_valid_sensor_time > SENSOR_TIMEOUT_S:
				current_cmd = send_drive_cmd(ser, "S", current_cmd)
			else:
				if now < oscillation_recovery_until:
					current_speed_cmd = send_speed_cmd(ser, args.forward_speed_cmd, current_speed_cmd)
					current_cmd = send_drive_cmd(ser, "F", current_cmd)
					sys.stdout.write(
						f"\rLeft:{left_surface:<5} Right:{right_surface:<5} Speed:{current_speed_cmd:<1} Cmd:{current_cmd} OscFlips:{oscillation_flips} RECOVER   "
					)
					sys.stdout.flush()
					time.sleep(POLL_DELAY_S)
					continue

				# 1) Left=WHITE, Right=WHITE -> move forward.
				if left_surface == "WHITE" and right_surface == "WHITE":
					last_turn_dir = None
					oscillation_flips = 0
					nudge_cmd = None
					nudge_until = 0.0
					pause_until = 0.0
					current_speed_cmd = send_speed_cmd(ser, args.forward_speed_cmd, current_speed_cmd)
					if forward_phase == "MOVE":
						if now - last_forward_phase_change >= args.forward_move_seconds:
							current_cmd = send_drive_cmd(ser, "S", current_cmd)
							forward_phase = "PAUSE"
							last_forward_phase_change = now
						else:
							current_cmd = send_drive_cmd(ser, "F", current_cmd)
					else:
						if now - last_forward_phase_change >= args.forward_pause_seconds:
							forward_phase = "MOVE"
							last_forward_phase_change = now
							current_cmd = send_drive_cmd(ser, "F", current_cmd)

				# 2) Left=BLACK, Right=WHITE -> slight left nudge.
				elif left_surface == "BLACK" and right_surface == "WHITE":
					if last_turn_dir == "R":
						oscillation_flips += 1
					elif last_turn_dir != "L":
						oscillation_flips = 0
					last_turn_dir = "L"

					if oscillation_flips >= max(1, args.oscillation_threshold):
						oscillation_recovery_until = now + max(0.01, args.oscillation_forward_seconds)
						oscillation_flips = 0
						last_turn_dir = None
						current_speed_cmd = send_speed_cmd(ser, args.forward_speed_cmd, current_speed_cmd)
						current_cmd = send_drive_cmd(ser, "F", current_cmd)
						continue

					forward_phase = "MOVE"
					last_forward_phase_change = now
					both_black_phase = "MOVE"
					last_both_black_phase_change = now
					if nudge_cmd != "L" or (now >= pause_until):
						nudge_cmd = "L"
						nudge_until = now + args.nudge_seconds
						pause_until = nudge_until + args.nudge_pause_seconds

					current_speed_cmd = send_speed_cmd(ser, args.turn_speed_cmd, current_speed_cmd)
					if now < nudge_until:
						current_cmd = send_drive_cmd(ser, "L", current_cmd)
					elif now < pause_until:
						current_cmd = send_drive_cmd(ser, "S", current_cmd)

				# 3) Left=WHITE, Right=BLACK -> slight right nudge.
				elif left_surface == "WHITE" and right_surface == "BLACK":
					if last_turn_dir == "L":
						oscillation_flips += 1
					elif last_turn_dir != "R":
						oscillation_flips = 0
					last_turn_dir = "R"

					if oscillation_flips >= max(1, args.oscillation_threshold):
						oscillation_recovery_until = now + max(0.01, args.oscillation_forward_seconds)
						oscillation_flips = 0
						last_turn_dir = None
						current_speed_cmd = send_speed_cmd(ser, args.forward_speed_cmd, current_speed_cmd)
						current_cmd = send_drive_cmd(ser, "F", current_cmd)
						continue

					forward_phase = "MOVE"
					last_forward_phase_change = now
					both_black_phase = "MOVE"
					last_both_black_phase_change = now
					if nudge_cmd != "R" or (now >= pause_until):
						nudge_cmd = "R"
						nudge_until = now + args.nudge_seconds
						pause_until = nudge_until + args.nudge_pause_seconds

					current_speed_cmd = send_speed_cmd(ser, args.turn_speed_cmd, current_speed_cmd)
					if now < nudge_until:
						current_cmd = send_drive_cmd(ser, "R", current_cmd)
					elif now < pause_until:
						current_cmd = send_drive_cmd(ser, "S", current_cmd)

				# 4) Left=BLACK, Right=BLACK -> move forward slowly to reduce oscillation.
				else:
					last_turn_dir = None
					oscillation_flips = 0
					forward_phase = "MOVE"
					last_forward_phase_change = now
					nudge_cmd = None
					nudge_until = 0.0
					pause_until = 0.0
					current_speed_cmd = send_speed_cmd(ser, args.both_black_speed_cmd, current_speed_cmd)
					if both_black_phase == "MOVE":
						if now - last_both_black_phase_change >= args.both_black_move_seconds:
							current_cmd = send_drive_cmd(ser, "S", current_cmd)
							both_black_phase = "PAUSE"
							last_both_black_phase_change = now
						else:
							current_cmd = send_drive_cmd(ser, "F", current_cmd)
					else:
						if now - last_both_black_phase_change >= args.both_black_pause_seconds:
							both_black_phase = "MOVE"
							last_both_black_phase_change = now
							current_cmd = send_drive_cmd(ser, "F", current_cmd)

			sys.stdout.write(
				f"\rLeft:{left_surface:<5} Right:{right_surface:<5} Speed:{current_speed_cmd:<1} Cmd:{current_cmd} OscFlips:{oscillation_flips}     "
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
