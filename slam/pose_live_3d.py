#!/usr/bin/env python3

import math
import os
import time
from errno import EREMOTEIO

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SENSITIVITY_LSB_PER_G = 16384.0   # +/-2g
GYRO_SENSITIVITY_LSB_PER_DPS = 131.0    # +/-250 deg/s
ALPHA = 0.95  # Complementary filter ratio
MAX_RETRIES = 3
CALIBRATION_SAMPLES = 120
SMOOTHING_BETA = 0.18  # 0..1, higher is more responsive and less smooth
STILL_ACCEL_TOLERANCE_G = 0.08
STILL_GYRO_TOLERANCE_DPS = 2.5


def read_word_2c(bus, reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def init_mpu(bus):
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)


def reopen_mpu_bus(retries=3):
    last_exc = None
    for _ in range(retries):
        new_bus = None
        try:
            new_bus = SMBus(1)
            init_mpu(new_bus)
            return new_bus
        except OSError as exc:
            last_exc = exc
            try:
                if new_bus is not None:
                    new_bus.close()
            except Exception:
                pass
            time.sleep(0.1)
    raise last_exc


def safe_read_imu(bus):
    last_exc = None
    for _ in range(MAX_RETRIES):
        try:
            gx, gy, gz = read_gyro_dps(bus)
            ax, ay, az = read_accel_g(bus)
            return gx, gy, gz, ax, ay, az
        except (OSError, TypeError) as exc:
            last_exc = exc
            time.sleep(0.01)
    raise last_exc


def calibrate_gyro_bias(bus, samples=CALIBRATION_SAMPLES):
    print("Keep the MPU-6050 still: calibrating gyro bias...")
    sx = 0.0
    sy = 0.0
    sz = 0.0
    collected = 0

    while collected < samples:
        try:
            gx, gy, gz = read_gyro_dps(bus)
            sx += gx
            sy += gy
            sz += gz
            collected += 1
        except (OSError, TypeError):
            # Skip transient read failures during calibration.
            continue
        time.sleep(0.002)

    bx = sx / samples
    by = sy / samples
    bz = sz / samples
    print(f"Gyro bias [deg/s] -> X: {bx:.3f}, Y: {by:.3f}, Z: {bz:.3f}")
    return bx, by, bz


def read_accel_g(bus):
    ax = read_word_2c(bus, ACCEL_XOUT_H) / ACCEL_SENSITIVITY_LSB_PER_G
    ay = read_word_2c(bus, ACCEL_XOUT_H + 2) / ACCEL_SENSITIVITY_LSB_PER_G
    az = read_word_2c(bus, ACCEL_XOUT_H + 4) / ACCEL_SENSITIVITY_LSB_PER_G
    return ax, ay, az


def read_gyro_dps(bus):
    gx = read_word_2c(bus, GYRO_XOUT_H) / GYRO_SENSITIVITY_LSB_PER_DPS
    gy = read_word_2c(bus, GYRO_XOUT_H + 2) / GYRO_SENSITIVITY_LSB_PER_DPS
    gz = read_word_2c(bus, GYRO_XOUT_H + 4) / GYRO_SENSITIVITY_LSB_PER_DPS
    return gx, gy, gz


def rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    # ZYX order: yaw -> pitch -> roll
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def main():
    if not os.path.exists("/dev/i2c-1"):
        print("Error: /dev/i2c-1 not found. Enable I2C first:")
        print("  sudo raspi-config nonint do_i2c 0")
        return

    bus = SMBus(1)  # Raspberry Pi I2C bus 1
    init_mpu(bus)
    gyro_bias_x, gyro_bias_y, gyro_bias_z = calibrate_gyro_bias(bus)

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    roll_vis = 0.0
    pitch_vis = 0.0
    yaw_vis = 0.0
    error_count = 0
    last_t = time.monotonic()

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("MPU-6050 Live Orientation (Roll/Pitch stabilized, Yaw drifts)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_box_aspect((1, 1, 1))

    # World axes (faint)
    ax.plot([0, 1], [0, 0], [0, 0], color="lightcoral", linestyle="--", linewidth=1)
    ax.plot([0, 0], [0, 1], [0, 0], color="lightgreen", linestyle="--", linewidth=1)
    ax.plot([0, 0], [0, 0], [0, 1], color="lightskyblue", linestyle="--", linewidth=1)

    x_line, = ax.plot([0, 1], [0, 0], [0, 0], color="red", linewidth=3, label="Body X")
    y_line, = ax.plot([0, 0], [0, 1], [0, 0], color="green", linewidth=3, label="Body Y")
    z_line, = ax.plot([0, 0], [0, 0], [0, 1], color="blue", linewidth=3, label="Body Z")
    ax.legend(loc="upper left")

    text = ax.text2D(0.03, 0.95, "", transform=ax.transAxes)

    def update(_frame):
        nonlocal roll, pitch, yaw, roll_vis, pitch_vis, yaw_vis, error_count, last_t, bus

        now = time.monotonic()
        dt = now - last_t
        last_t = now

        try:
            gx, gy, gz, ax_g, ay_g, az_g = safe_read_imu(bus)
            error_count = 0
        except (OSError, TypeError) as exc:
            error_count += 1
            if isinstance(exc, OSError) and exc.errno in (EREMOTEIO, 5):
                # Try to recover sensor/bus on transient I2C NACK conditions.
                try:
                    bus.close()
                except OSError:
                    pass
                time.sleep(0.05)
                try:
                    bus = reopen_mpu_bus()
                    text.set_text("I2C recovered")
                    return x_line, y_line, z_line, text
                except OSError as reopen_exc:
                    text.set_text(
                        f"I2C recover failed: {reopen_exc}\\n"
                        f"check wiring/power/address ({error_count})"
                    )
                    return x_line, y_line, z_line, text
            text.set_text(f"I2C read error: {exc}\\nrecovering... ({error_count})")
            return x_line, y_line, z_line, text

        gx -= gyro_bias_x
        gy -= gyro_bias_y
        gz -= gyro_bias_z

        roll_acc = math.degrees(math.atan2(ay_g, az_g))
        pitch_acc = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)))
        accel_mag = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        is_still = (
            abs(accel_mag - 1.0) < STILL_ACCEL_TOLERANCE_G
            and abs(gx) < STILL_GYRO_TOLERANCE_DPS
            and abs(gy) < STILL_GYRO_TOLERANCE_DPS
            and abs(gz) < STILL_GYRO_TOLERANCE_DPS
        )

        # Gyro integration + accelerometer correction for roll and pitch.
        roll = ALPHA * (roll + gx * dt) + (1.0 - ALPHA) * roll_acc
        pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * pitch_acc

        if is_still:
            # When the device is resting, trust the gravity vector more strongly
            # so the display returns to the original level pose faster.
            roll = 0.7 * roll + 0.3 * roll_acc
            pitch = 0.7 * pitch + 0.3 * pitch_acc

        yaw += gz * dt  # No magnetometer, so yaw drifts over time.

        # Smooth rendered angles to reduce visual jitter.
        roll_vis = (1.0 - SMOOTHING_BETA) * roll_vis + SMOOTHING_BETA * roll
        pitch_vis = (1.0 - SMOOTHING_BETA) * pitch_vis + SMOOTHING_BETA * pitch
        yaw_vis = yaw

        rmat = euler_to_rotation_matrix(roll_vis, pitch_vis, yaw_vis)
        ex = rmat @ np.array([1.0, 0.0, 0.0])
        ey = rmat @ np.array([0.0, 1.0, 0.0])
        ez = rmat @ np.array([0.0, 0.0, 1.0])

        x_line.set_data_3d([0, ex[0]], [0, ex[1]], [0, ex[2]])
        y_line.set_data_3d([0, ey[0]], [0, ey[1]], [0, ey[2]])
        z_line.set_data_3d([0, ez[0]], [0, ez[1]], [0, ez[2]])

        text.set_text(
            f"Roll: {roll_vis:7.2f} deg\\n"
            f"Pitch: {pitch_vis:7.2f} deg\\n"
            f"Yaw: {yaw_vis:7.2f} deg"
        )

        return x_line, y_line, z_line, text

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        _ = ani
        bus.close()


if __name__ == "__main__":
    main()
