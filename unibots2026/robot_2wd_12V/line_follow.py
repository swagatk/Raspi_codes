#!/usr/bin/env python3
"""Autonomous line following using a single digital line sensor over serial.

Behavior:
1) On start, raises arm to UP pose.
2) Enters SEARCH mode and sweeps left/right slowly until BLACK is detected.
3) Enters FOLLOW mode and keeps moving forward while BLACK is detected.
4) If line is lost (WHITE), turns in the last turn direction to re-acquire line.
5) Stops safely on Ctrl+C.
"""

import argparse
import re
import sys
import time

import serial
import serial.tools.list_ports


BAUD_RATE = 115200
POLL_DELAY_S = 0.03
ARM_MOVE_DELAY_S = 2.0
SERIAL_RESET_DELAY_S = 2.0
SENSOR_TIMEOUT_S = 0.8

# --- USER TUNING (edit these values first) ---
# Speed command supported by your Arduino sketch: usually "1" (slow) to "3" (fast).
DRIVE_SPEED_CMD = "1"
# Search motion timings (seconds): shorter turn pulses, slightly longer forward nudges.
SEARCH_TURN_SECONDS = 0.1
SEARCH_FORWARD_SECONDS = 0.15
SEARCH_PAUSE_SECONDS = 0.15
# Follow motion timings (seconds): pulsed forward to keep motion gentle.
# These are intentionally longer so the robot moves slowly and straighter on the line.
FOLLOW_FORWARD_SECONDS = 0.08
FOLLOW_PAUSE_SECONDS = 0.15
# Curve recovery: give the line a brief grace window before switching into search mode.
CURVE_RECOVERY_SECONDS = 0.1
# Recovery search policy: try the remembered side for a few steps, then spend longer on the opposite side.
RECOVERY_PRIMARY_STEPS = 2
RECOVERY_SECONDARY_STEPS = 4


def build_recovery_pattern(primary_side: str, primary_steps: int, secondary_steps: int) -> list[str]:
	"""Build a sweep pattern that biases one side first, then opposite side.

	Pattern format: [TURN, F, TURN, F, ...]
	"""
	primary_side = "L" if primary_side != "R" else "R"
	secondary_side = "R" if primary_side == "L" else "L"
	primary_count = max(1, int(primary_steps))
	secondary_count = max(1, int(secondary_steps))

	pattern: list[str] = []
	for _ in range(primary_count):
		pattern.extend([primary_side, "F"])
	for _ in range(secondary_count):
		pattern.extend([secondary_side, "F"])
	return pattern


def find_arduino_port() -> str | None:
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

	# Accept lines like: "0", "1", "LINE,0", "S:1"
	match = re.search(r"(^|[^0-9])([01])($|[^0-9])", cleaned)
	if match:
		return int(match.group(2))

	return None


def state_to_surface(state: int, black_is_low: bool) -> str:
	is_black = (state == 0) if black_is_low else (state == 1)
	return "BLACK" if is_black else "WHITE"


def read_line_surface(ser: serial.Serial, black_is_low: bool) -> str | None:
	if ser.in_waiting <= 0:
		return None

	raw = ser.readline().decode("utf-8", errors="ignore").strip()
	state = parse_sensor_state(raw)
	if state is None:
		return None

	return state_to_surface(state, black_is_low)


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
		"--search-switch-seconds",
		type=float,
		default=SEARCH_TURN_SECONDS,
		help="Duration for search turn steps (L/R) while line is not found",
	)
	parser.add_argument(
		"--search-forward-seconds",
		type=float,
		default=SEARCH_FORWARD_SECONDS,
		help="Duration for search forward nudge steps (F) while line is not found",
	)
	parser.add_argument(
		"--search-pause-seconds",
		type=float,
		default=SEARCH_PAUSE_SECONDS,
		help="Pause duration between search nudge commands",
	)
	parser.add_argument(
		"--follow-forward-seconds",
		type=float,
		default=FOLLOW_FORWARD_SECONDS,
		help="Forward pulse duration while following BLACK",
	)
	parser.add_argument(
		"--follow-pause-seconds",
		type=float,
		default=FOLLOW_PAUSE_SECONDS,
		help="Pause duration between follow forward pulses",
	)
	parser.add_argument(
		"--curve-recovery-seconds",
		type=float,
		default=CURVE_RECOVERY_SECONDS,
		help="Grace period to keep moving forward on a curve before searching",
	)
	parser.add_argument(
		"--recovery-primary-steps",
		type=int,
		default=RECOVERY_PRIMARY_STEPS,
		help="Number of turn+forward recovery steps on remembered side",
	)
	parser.add_argument(
		"--recovery-secondary-steps",
		type=int,
		default=RECOVERY_SECONDARY_STEPS,
		help="Number of turn+forward recovery steps on opposite side",
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
	mode = "SEARCH"
	recovery_side = "L"
	search_pattern = build_recovery_pattern(
		recovery_side,
		args.recovery_primary_steps,
		args.recovery_secondary_steps,
	)
	search_index = 0
	search_cmd = search_pattern[search_index]
	search_step_duration = args.search_forward_seconds if search_cmd == "F" else args.search_switch_seconds
	search_phase = "MOVE"
	last_search_phase_change = time.monotonic()
	last_surface_time = time.monotonic()
	follow_phase = "MOVE"
	last_follow_phase_change = time.monotonic()
	recovery_started_at = None

	print(f"Connected to {port} @ {args.baud}")
	print(f"Mapping: {'0=BLACK, 1=WHITE' if args.black_is_low else '1=BLACK, 0=WHITE'}")
	print("Sending arm to UP pose...")
	ser.write(b"A\n")
	time.sleep(ARM_MOVE_DELAY_S)

	print("Setting speed and starting line search...")
	ser.write((args.speed_cmd + "\n").encode("utf-8"))
	current_cmd = send_drive_cmd(ser, "S", current_cmd)
	print("Press Ctrl+C to stop.")

	try:
		while True:
			surface = read_line_surface(ser, args.black_is_low)

			now = time.monotonic()
			if surface is not None:
				last_surface_time = now

				if mode == "SEARCH" and surface == "BLACK":
					mode = "FOLLOW"
					follow_phase = "MOVE"
					last_follow_phase_change = now
					recovery_side = search_cmd if search_cmd in ("L", "R") else recovery_side
					current_cmd = send_drive_cmd(ser, args.speed_cmd, current_cmd)
					current_cmd = send_drive_cmd(ser, "F", current_cmd)
					print("\n[SEARCH -> FOLLOW] Track found (BLACK).")

				elif mode == "FOLLOW":
					if surface == "BLACK":
						recovery_started_at = None
						pass
					else:
						if recovery_started_at is None:
							recovery_started_at = now
							current_cmd = send_drive_cmd(ser, "F", current_cmd)
							print("\n[FOLLOW] Curve/line-loss detected, recovering forward...")
						elif now - recovery_started_at >= args.curve_recovery_seconds:
							mode = "SEARCH"
							search_pattern = build_recovery_pattern(
								recovery_side,
								args.recovery_primary_steps,
								args.recovery_secondary_steps,
							)
							search_index = 0
							search_cmd = search_pattern[search_index]
							search_step_duration = (
								args.search_forward_seconds if search_cmd == "F" else args.search_switch_seconds
							)
							search_phase = "MOVE"
							last_search_phase_change = now
							recovery_started_at = None
							current_cmd = send_drive_cmd(ser, search_cmd, current_cmd)
							print("\n[FOLLOW -> SEARCH] Line lost too long, re-acquiring with sweep L/F/R/F...")

				sys.stdout.write(f"\rMode:{mode:<6} Sensor:{surface:<5} Cmd:{current_cmd}   ")
				sys.stdout.flush()

			if mode == "SEARCH":
				if now - last_surface_time > SENSOR_TIMEOUT_S or surface in ("WHITE", None):
					# Sweep through L -> F -> R -> F so the robot actively scans for the black patch.
					if search_phase == "MOVE":
						if now - last_search_phase_change >= search_step_duration:
							current_cmd = send_drive_cmd(ser, "S", current_cmd)
							search_phase = "PAUSE"
							last_search_phase_change = now
						else:
							current_cmd = send_drive_cmd(ser, search_cmd, current_cmd)
					else:
						if now - last_search_phase_change >= args.search_pause_seconds:
							search_index = (search_index + 1) % len(search_pattern)
							if search_index == 0:
								recovery_side = "R" if recovery_side == "L" else "L"
								search_pattern = build_recovery_pattern(
									recovery_side,
									args.recovery_primary_steps,
									args.recovery_secondary_steps,
								)
							search_cmd = search_pattern[search_index]
							search_step_duration = (
								args.search_forward_seconds if search_cmd == "F" else args.search_switch_seconds
							)
							search_phase = "MOVE"
							last_search_phase_change = now
							current_cmd = send_drive_cmd(ser, search_cmd, current_cmd)

			elif mode == "FOLLOW":
				if surface == "BLACK":
					recovery_started_at = None
					if follow_phase == "MOVE":
						if now - last_follow_phase_change >= args.follow_forward_seconds:
							current_cmd = send_drive_cmd(ser, "S", current_cmd)
							follow_phase = "PAUSE"
							last_follow_phase_change = now
						else:
							current_cmd = send_drive_cmd(ser, "F", current_cmd)
					else:
						if now - last_follow_phase_change >= args.follow_pause_seconds:
							current_cmd = send_drive_cmd(ser, "F", current_cmd)
							follow_phase = "MOVE"
							last_follow_phase_change = now
				else:
					if recovery_started_at is None:
						recovery_started_at = now
						current_cmd = send_drive_cmd(ser, "F", current_cmd)
					elif now - recovery_started_at >= args.curve_recovery_seconds:
						mode = "SEARCH"
						search_pattern = build_recovery_pattern(
							recovery_side,
							args.recovery_primary_steps,
							args.recovery_secondary_steps,
						)
						search_index = 0
						search_cmd = search_pattern[search_index]
						search_step_duration = (
							args.search_forward_seconds if search_cmd == "F" else args.search_switch_seconds
						)
						search_phase = "MOVE"
						last_search_phase_change = now
						recovery_started_at = None
						current_cmd = send_drive_cmd(ser, search_cmd, current_cmd)
					else:
						current_cmd = send_drive_cmd(ser, "F", current_cmd)

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
