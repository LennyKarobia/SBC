import time
import math
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
)


bno = None
heading_offset = None


def init_imu():
    global bno, heading_offset

    i2c = busio.I2C(board.SCL, board.SDA)

    for addr in [0x4A, 0x4B]:
        try:
            sensor = BNO08X_I2C(i2c, address=addr)
            sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            sensor.enable_feature(BNO_REPORT_GYROSCOPE)

            bno = sensor
            print(f"IMU connected at {hex(addr)}")

            # Let readings settle before capturing startup heading offset.
            time.sleep(1)
            heading_offset = get_raw_heading()
            print(f"IMU zero-heading offset set to {heading_offset:.4f}°")
            return
        except Exception:
            continue

    raise RuntimeError("IMU not found at 0x4A or 0x4B")


def get_raw_heading():
    quat_i, quat_j, quat_k, quat_real = bno.quaternion

    yaw = math.atan2(
        2.0 * (quat_real * quat_k + quat_i * quat_j),
        1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k),
    )

    return math.degrees(yaw) % 360


def get_heading():
    global heading_offset

    try:
        raw = get_raw_heading()
        if heading_offset is None:
            heading_offset = raw
        return (raw - heading_offset + 360) % 360
    except Exception:
        print("IMU read error")
        return 0


def get_linear_accel():
    try:
        return bno.linear_acceleration
    except Exception:
        return (0.0, 0.0, 0.0)


def get_gyro():
    try:
        return bno.gyro
    except Exception:
        return (0.0, 0.0, 0.0)
