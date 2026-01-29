import os
import time
import math
import csv
import datetime
import serial
import pynmea2

import board
import busio
import digitalio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR


# --------------------
# User settings
# --------------------
GPS_PORT = "/dev/ttyS0"     # or "/dev/serial0"
GPS_BAUD = 115200

I2C_FREQ = 50000           # 50 kHz (you said already set, but we also request it)
BNO_ADDR = 0x4B

# RST pin (change if needed)
RST_PIN = board.D25        # GPIO25 (physical pin 22)

# logging
DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")

# rates
IMU_INTERVAL_S = 0.05       # 20 Hz
GPS_READ_TIMEOUT = 0.05     # seconds
IMU_RETRY_COOLDOWN = 2.0    # seconds between IMU re-init attempts


# --------------------
# Helpers
# --------------------
def utc_now_iso():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")


def quat_to_yaw_pitch_roll_deg(qi, qj, qk, qr):
    """
    Adafruit BNO08x quaternion order is (i, j, k, real).
    Returns yaw, pitch, roll in degrees.
    """
    x, y, z, w = qi, qj, qk, qr

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)
    return yaw_deg, pitch_deg, roll_deg


class IMUManager:
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno = None

        # RST pin setup (active-low is typical)
        self.rst = digitalio.DigitalInOut(RST_PIN)
        self.rst.direction = digitalio.Direction.OUTPUT
        self.rst.value = True  # deassert

        self.last_ok_iso = None
        self.last_error = None

    def hw_reset(self):
        # Active-low reset (most boards)
        self.rst.value = True
        time.sleep(0.02)
        self.rst.value = False
        time.sleep(0.10)
        self.rst.value = True
        time.sleep(0.50)

    def init_bno(self, debug=False, max_tries=30):
        """
        Robust init: hardware reset + create device + enable feature.
        Retries especially for ('Unprocessable Batch bytes', N).
        """
        self.last_error = None
        self.bno = None

        for attempt in range(1, max_tries + 1):
            try:
                self.hw_reset()

                bno = BNO08X_I2C(self.i2c, address=BNO_ADDR, debug=debug)

                # IMPORTANT: report_interval must be integer (microseconds)
                # 50ms = 50000us
                bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)

                self.bno = bno
                self.last_ok_iso = utc_now_iso()
                return True

            except Exception as e:
                self.last_error = str(e)
                # short backoff; longer for repeated failures
                time.sleep(0.10 + 0.05 * attempt)

        return False

    def read(self):
        """
        Returns dict with quaternion + yaw/pitch/roll or None if down.
        """
        if self.bno is None:
            return None

        try:
            qi, qj, qk, qr = self.bno.quaternion
            yaw, pitch, roll = quat_to_yaw_pitch_roll_deg(qi, qj, qk, qr)
            self.last_ok_iso = utc_now_iso()
            return {
                "qi": qi, "qj": qj, "qk": qk, "qr": qr,
                "yaw": yaw, "pitch": pitch, "roll": roll
            }
        except Exception as e:
            self.last_error = str(e)
            self.bno = None
            return None


def create_i2c():
    # Some Blinka versions accept frequency=
    try:
        return busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ)
    except TypeError:
        return busio.I2C(board.SCL, board.SDA)


def parse_gps_line(line, gps_state):
    """
    Updates gps_state dict in-place when a useful sentence arrives.
    Uses GGA for fix/alt/sats/hdop. RMC for speed/course if present.
    """
    try:
        msg = pynmea2.parse(line)
    except pynmea2.nmea.ChecksumError:
        return
    except Exception:
        return

    st = getattr(msg, "sentence_type", "")

    if st == "GGA":
        # fix quality: 0 invalid, 1 GPS fix, 2 DGPS...
        fixq = getattr(msg, "gps_qual", "0")
        try:
            fixq_i = int(fixq)
        except Exception:
            fixq_i = 0

        gps_state["fix"] = (fixq_i > 0)
        gps_state["lat"] = getattr(msg, "latitude", None)
        gps_state["lon"] = getattr(msg, "longitude", None)
        gps_state["alt_m"] = getattr(msg, "altitude", None)
        gps_state["sats"] = getattr(msg, "num_sats", None)
        gps_state["hdop"] = getattr(msg, "horizontal_dil", None)
        gps_state["last_gga_iso"] = utc_now_iso()

    elif st == "RMC":
        # RMC status: A=active, V=void
        status = getattr(msg, "status", "V")
        gps_state["fix"] = (status == "A") or gps_state.get("fix", False)
        gps_state["cog_deg"] = getattr(msg, "true_course", None)
        gps_state["sog_knots"] = getattr(msg, "spd_over_grnd", None)
        gps_state["last_rmc_iso"] = utc_now_iso()


def ensure_csv_header(path, writer, fobj):
    if fobj.tell() == 0:
        writer.writerow([
            "time_utc",

            "gps_fix",
            "lat", "lon", "alt_m",
            "sats", "hdop",
            "cog_deg", "sog_knots",

            "imu_ok",
            "yaw_deg", "pitch_deg", "roll_deg",
            "qi", "qj", "qk", "qr",

            "imu_last_error"
        ])
        fobj.flush()


def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    # GPS state (latest values)
    gps_state = {
        "fix": False,
        "lat": None,
        "lon": None,
        "alt_m": None,
        "sats": None,
        "hdop": None,
        "cog_deg": None,
        "sog_knots": None,
        "last_gga_iso": None,
        "last_rmc_iso": None,
    }

    # I2C + IMU manager
    i2c = create_i2c()
    imu = IMUManager(i2c)

    # Try initial IMU init (don’t spam forever here)
    imu.init_bno(debug=False, max_tries=10)

    next_imu_time = time.monotonic()
    next_imu_retry_time = time.monotonic() + IMU_RETRY_COOLDOWN

    with serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_READ_TIMEOUT) as ser, \
         open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        ensure_csv_header(CSV_PATH, writer, f)

        print(f"Logging to: {CSV_PATH}")
        print("Running... Ctrl+C to stop")

        try:
            while True:
                # --- GPS read (one line per loop, updates latest state) ---
                line = ser.readline().decode(errors="ignore").strip()
                if line and (line.startswith("$GN") or line.startswith("$GP")):
                    parse_gps_line(line, gps_state)

                # --- IMU read at fixed interval ---
                now_mono = time.monotonic()
                imu_data = None

                if now_mono >= next_imu_time:
                    next_imu_time = now_mono + IMU_INTERVAL_S

                    imu_data = imu.read()

                    # If IMU is down, attempt re-init occasionally (with RST)
                    if imu_data is None and now_mono >= next_imu_retry_time:
                        ok = imu.init_bno(debug=False, max_tries=30)
                        next_imu_retry_time = now_mono + IMU_RETRY_COOLDOWN
                        if not ok:
                            # keep it down; will retry later
                            pass

                # --- Decide if we write a CSV row ---
                # Write when we got an IMU sample tick (or IMU down tick).
                if now_mono >= (next_imu_time - IMU_INTERVAL_S / 2):
                    t = utc_now_iso()

                    gps_fix = bool(gps_state.get("fix", False))
                    lat = gps_state.get("lat", None)
                    lon = gps_state.get("lon", None)

                    imu_ok = imu_data is not None

                    if imu_ok:
                        row = [
                            t,
                            "FIX" if gps_fix else "NOFIX",
                            lat, lon, gps_state.get("alt_m", None),
                            gps_state.get("sats", None), gps_state.get("hdop", None),
                            gps_state.get("cog_deg", None), gps_state.get("sog_knots", None),

                            "OK",
                            imu_data["yaw"], imu_data["pitch"], imu_data["roll"],
                            imu_data["qi"], imu_data["qj"], imu_data["qk"], imu_data["qr"],

                            ""
                        ]
                    else:
                        row = [
                            t,
                            "FIX" if gps_fix else "NOFIX",
                            lat, lon, gps_state.get("alt_m", None),
                            gps_state.get("sats", None), gps_state.get("hdop", None),
                            gps_state.get("cog_deg", None), gps_state.get("sog_knots", None),

                            "DOWN",
                            "", "", "",
                            "", "", "", "",

                            imu.last_error or ""
                        ]

                    writer.writerow(row)
                    f.flush()

                    # small console status (not too noisy)
                    if gps_fix:
                        print(f"{t} GPS=FIX IMU={'OK' if imu_ok else 'down'}")
                    else:
                        print(f"{t} GPS=NOFIX IMU={'OK' if imu_ok else 'down'}")

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
