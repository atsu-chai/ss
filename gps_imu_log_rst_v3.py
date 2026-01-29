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


GPS_PORT = "/dev/ttyS0"
GPS_BAUD = 115200

# try both common addresses
BNO_ADDR_CANDIDATES = [0x4A, 0x4B]

# GPIO25 (physical pin 22)
RST_PIN = board.D25

# try both reset polarities automatically
RESET_ACTIVE_LOW_CANDIDATES = [True, False]

DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")

IMU_INTERVAL_S = 0.05        # 20 Hz log
PRINT_INTERVAL_S = 1.0       # 1 Hz console print
GPS_READ_TIMEOUT = 0.05

IMU_RETRY_COOLDOWN = 3.0     # seconds between IMU init attempts
IMU_INIT_TRIES_PER_ATTEMPT = 1  # keep short to avoid long blocking


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
            "imu_addr",
            "imu_reset_active_low",
            "imu_last_error"
        ])
        fobj.flush()


class IMUManager:
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno = None
        self.last_error = ""
        self.addr_in_use = None
        self.reset_active_low_in_use = None

        self.rst = digitalio.DigitalInOut(RST_PIN)
        self.rst.direction = digitalio.Direction.OUTPUT

    def _set_reset(self, deassert: bool, active_low: bool):
        if active_low:
            self.rst.value = True if deassert else False
        else:
            self.rst.value = False if deassert else True

    def hw_reset_pulse(self, active_low: bool):
        self._set_reset(deassert=True, active_low=active_low)
        time.sleep(0.02)
        self._set_reset(deassert=False, active_low=active_low)
        time.sleep(0.12)
        self._set_reset(deassert=True, active_low=active_low)
        time.sleep(0.60)

    def init_once(self):
        self.bno = None
        self.addr_in_use = None
        self.reset_active_low_in_use = None
        self.last_error = ""

        # try combinations
        for active_low in RESET_ACTIVE_LOW_CANDIDATES:
            for addr in BNO_ADDR_CANDIDATES:
                for _ in range(IMU_INIT_TRIES_PER_ATTEMPT):
                    try:
                        self.hw_reset_pulse(active_low=active_low)

                        # IMPORTANT: debug=False to avoid packet spam
                        bno = BNO08X_I2C(self.i2c, address=addr, debug=False)
                        try:
                            bno._debug = False
                        except Exception:
                            pass

                        # report_interval must be int microseconds
                        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)

                        self.bno = bno
                        self.addr_in_use = addr
                        self.reset_active_low_in_use = active_low
                        self.last_error = ""
                        return True

                    except Exception as e:
                        self.last_error = f"addr=0x{addr:02x} active_low={active_low} err={e}"
                        time.sleep(0.05)

        return False

    def read(self):
        if self.bno is None:
            return None
        try:
            qi, qj, qk, qr = self.bno.quaternion
            yaw, pitch, roll = quat_to_yaw_pitch_roll_deg(qi, qj, qk, qr)
            return {"qi": qi, "qj": qj, "qk": qk, "qr": qr, "yaw": yaw, "pitch": pitch, "roll": roll}
        except Exception as e:
            self.last_error = f"read err={e}"
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

    i2c = busio.I2C(board.SCL, board.SDA)
    imu = IMUManager(i2c)
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
                line = ser.readline().decode(errors="ignore").strip()
                if line and (line.startswith("$GN") or line.startswith("$GP")):
                    parse_gps_line(line, gps_state)

                now = time.monotonic()

                if now >= next_imu:
                    next_imu = now + IMU_INTERVAL_S

                    imu_data = imu.read()
                    imu_ok = (imu_data is not None)

                    if (not imu_ok) and now >= next_retry:
                        imu.init_once()
                        next_retry = now + IMU_RETRY_COOLDOWN

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
                            f"0x{imu.addr_in_use:02x}",
                            imu.reset_active_low_in_use,
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
                            "" if imu.addr_in_use is None else f"0x{imu.addr_in_use:02x}",
                            "" if imu.reset_active_low_in_use is None else imu.reset_active_low_in_use,
                            imu.last_error
                        ]

                    writer.writerow(row)
                    f.flush()

                if now >= next_print:
                    next_print = now + PRINT_INTERVAL_S
                    t = utc_now_iso()
                    gps_fix = "FIX" if gps_state.get("fix", False) else "NOFIX"
                    if imu.bno is None:
                        print(f"{t} GPS={gps_fix} IMU=down | {imu.last_error}")
                    else:
                        print(f"{t} GPS={gps_fix} IMU=OK addr=0x{imu.addr_in_use:02x} active_low={imu.reset_active_low_in_use}")

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
