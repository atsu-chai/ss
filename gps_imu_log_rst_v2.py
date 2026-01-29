import os
import time
import math
import csv
import datetime
import serial
import pynmea2
import contextlib

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

BNO_ADDR = 0x4B

# RST pin (GPIO25 = physical pin 22)
RST_PIN = board.D25

# !!! IMPORTANT: reset polarity !!!
# True  : Active-LOW reset (most breakout boards)
# False : Active-HIGH reset (some boards)
RESET_ACTIVE_LOW = True

DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")

IMU_INTERVAL_S = 0.05        # 20 Hz log
PRINT_INTERVAL_S = 1.0       # 1 Hz console print

GPS_READ_TIMEOUT = 0.05
IMU_RETRY_COOLDOWN = 2.0     # seconds between IMU init attempts
IMU_INIT_TRIES_PER_ATTEMPT = 3  # keep small to avoid long blocking


def utc_now_iso():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")


def quat_to_yaw_pitch_roll_deg(qi, qj, qk, qr):
    x, y, z, w = qi, qj, qk, qr

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)
    return yaw_deg, pitch_deg, roll_deg


def parse_gps_line(line, gps_state):
    try:
        msg = pynmea2.parse(line)
    except pynmea2.nmea.ChecksumError:
        return
    except Exception:
        return

    st = getattr(msg, "sentence_type", "")

    if st == "GGA":
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

    elif st == "RMC":
        status = getattr(msg, "status", "V")
        gps_state["fix"] = (status == "A") or gps_state.get("fix", False)
        gps_state["cog_deg"] = getattr(msg, "true_course", None)
        gps_state["sog_knots"] = getattr(msg, "spd_over_grnd", None)


def ensure_csv_header(fobj, writer):
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


class IMUManager:
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno = None
        self.last_error = None

        self.rst = digitalio.DigitalInOut(RST_PIN)
        self.rst.direction = digitalio.Direction.OUTPUT

        # deassert reset
        self._set_reset(deassert=True)

    def _set_reset(self, deassert: bool):
        # deassert means "not in reset"
        if RESET_ACTIVE_LOW:
            self.rst.value = True if deassert else False
        else:
            self.rst.value = False if deassert else True

    def hw_reset_pulse(self):
        # pulse reset safely
        self._set_reset(deassert=True)
        time.sleep(0.02)
        self._set_reset(deassert=False)
        time.sleep(0.12)
        self._set_reset(deassert=True)
        time.sleep(0.60)

    def init_once(self):
        """
        Try a small number of times; return quickly.
        Also silences library debug prints completely.
        """
        self.bno = None
        self.last_error = None

        # Silence any accidental prints from the library during init
        with open(os.devnull, "w") as dn, contextlib.redirect_stdout(dn), contextlib.redirect_stderr(dn):
            for k in range(1, IMU_INIT_TRIES_PER_ATTEMPT + 1):
                try:
                    self.hw_reset_pulse()

                    bno = BNO08X_I2C(self.i2c, address=BNO_ADDR, debug=False)

                    # just in case something flips it
                    try:
                        bno._debug = False
                    except Exception:
                        pass

                    # report_interval must be integer microseconds
                    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)

                    self.bno = bno
                    return True

                except Exception as e:
                    self.last_error = str(e)
                    time.sleep(0.10 * k)

        return False

    def read(self):
        if self.bno is None:
            return None
        try:
            qi, qj, qk, qr = self.bno.quaternion
            yaw, pitch, roll = quat_to_yaw_pitch_roll_deg(qi, qj, qk, qr)
            return {"qi": qi, "qj": qj, "qk": qk, "qr": qr, "yaw": yaw, "pitch": pitch, "roll": roll}
        except Exception as e:
            self.last_error = str(e)
            self.bno = None
            return None


def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    gps_state = {
        "fix": False,
        "lat": None,
        "lon": None,
        "alt_m": None,
        "sats": None,
        "hdop": None,
        "cog_deg": None,
        "sog_knots": None,
    }

    i2c = busio.I2C(board.SCL, board.SDA)  # freq warning is ok
    imu = IMUManager(i2c)

    # first init (quick)
    imu.init_once()

    next_imu = time.monotonic()
    next_print = time.monotonic()
    next_retry = time.monotonic() + IMU_RETRY_COOLDOWN

    with serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_READ_TIMEOUT) as ser, \
         open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        ensure_csv_header(f, writer)

        print(f"Logging to: {CSV_PATH}")
        print("Running... Ctrl+C to stop")

        try:
            while True:
                # GPS: read one line, update latest
                line = ser.readline().decode(errors="ignore").strip()
                if line and (line.startswith("$GN") or line.startswith("$GP")):
                    parse_gps_line(line, gps_state)

                # IMU: fixed interval
                now = time.monotonic()
                imu_data = None
                imu_ok = False

                if now >= next_imu:
                    next_imu = now + IMU_INTERVAL_S

                    imu_data = imu.read()
                    imu_ok = (imu_data is not None)

                    # retry init if down (cooldown)
                    if not imu_ok and now >= next_retry:
                        imu.init_once()
                        next_retry = now + IMU_RETRY_COOLDOWN

                    # write CSV at IMU rate (20Hz)
                    t = utc_now_iso()
                    gps_fix = "FIX" if gps_state.get("fix", False) else "NOFIX"

                    if imu_ok:
                        row = [
                            t, gps_fix,
                            gps_state.get("lat"), gps_state.get("lon"), gps_state.get("alt_m"),
                            gps_state.get("sats"), gps_state.get("hdop"),
                            gps_state.get("cog_deg"), gps_state.get("sog_knots"),
                            "OK",
                            imu_data["yaw"], imu_data["pitch"], imu_data["roll"],
                            imu_data["qi"], imu_data["qj"], imu_data["qk"], imu_data["qr"],
                            ""
                        ]
                    else:
                        row = [
                            t, gps_fix,
                            gps_state.get("lat"), gps_state.get("lon"), gps_state.get("alt_m"),
                            gps_state.get("sats"), gps_state.get("hdop"),
                            gps_state.get("cog_deg"), gps_state.get("sog_knots"),
                            "DOWN",
                            "", "", "",
                            "", "", "", "",
                            imu.last_error or ""
                        ]

                    writer.writerow(row)
                    f.flush()

                # console print at 1Hz
                if now >= next_print:
                    next_print = now + PRINT_INTERVAL_S
                    t = utc_now_iso()
                    gps_fix = "FIX" if gps_state.get("fix", False) else "NOFIX"
                    imu_status = "OK" if imu.bno is not None else "down"
                    print(f"{t} GPS={gps_fix} IMU={imu_status}")

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
