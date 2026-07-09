#!/usr/bin/env python3

import argparse
import glob
import importlib
import sys
import time

try:
	RPLidar = importlib.import_module("rplidar").RPLidar
except ImportError:
	RPLidar = None


def detect_port():
	for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
		ports = sorted(glob.glob(pattern))
		if ports:
			return ports[0]
	return None


def format_scan(scan, limit):
	valid_points = []
	for quality, angle, distance in scan:
		if distance > 0:
			valid_points.append((quality, angle, distance))

	preview = valid_points[:limit]
	if not preview:
		return "no valid returns"

	formatted = ", ".join(
		f"{angle:6.1f} deg -> {distance:7.1f} mm (q={quality})"
		for quality, angle, distance in preview
	)
	return f"{len(valid_points)} points | {formatted}"


def main():
	parser = argparse.ArgumentParser(
		description="Simple live test for an RPLidar A2M8 connected over USB."
	)
	parser.add_argument(
		"--port",
		default=detect_port(),
		help="Serial device for the lidar, for example /dev/ttyUSB0",
	)
	parser.add_argument(
		"--baudrate",
		type=int,
		default=115200,
		help="Serial baud rate used by the lidar",
	)
	parser.add_argument(
		"--scans",
		type=int,
		default=0,
		help="Number of 360-degree scans to print before exiting; use 0 to run continuously",
	)
	parser.add_argument(
		"--points",
		type=int,
		default=8,
		help="How many points from each scan to preview",
	)
	args = parser.parse_args()

	if RPLidar is None:
		print("Missing dependency: rplidar")
		print("Install it with: python3 -m pip install rplidar")
		return 1

	if not args.port:
		print("No lidar serial port found.")
		print("Connect the RPLidar and retry, or pass --port /dev/ttyUSB0")
		return 1

	lidar = RPLidar(args.port, baudrate=args.baudrate)

	try:
		lidar.start_motor()
		time.sleep(0.5)

		info = lidar.get_info()
		health = lidar.get_health()

		print(f"Connected to {args.port}")
		print(f"Model: {info.get('model')} | Firmware: {info.get('firmware')} | Hardware: {info.get('hardware')}")
		print(f"Serial: {info.get('serialnumber')}")
		print(f"Health: {health}")
		if args.scans == 0:
			print("Reading continuous 360-degree scans. Press Ctrl+C to stop.\n")
		else:
			print(f"Reading {args.scans} 360-degree scan(s). Press Ctrl+C to stop.\n")

		for index, scan in enumerate(lidar.iter_scans(), start=1):
			print(f"Scan {index}: {format_scan(scan, args.points)}")
			if args.scans > 0 and index >= args.scans:
				break
			time.sleep(0.05)

		return 0
	except KeyboardInterrupt:
		print("\nStopped.")
		return 0
	except Exception as exc:
		print(f"Lidar test failed: {exc}")
		return 1
	finally:
		try:
			lidar.stop()
		except Exception:
			pass
		try:
			lidar.stop_motor()
		except Exception:
			pass
		lidar.disconnect()


if __name__ == "__main__":
	sys.exit(main())
