#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import math
import csv
import threading
import datetime

import serial
import pynmea2

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR


# ========= 設定 =========
# GPS
GPS_PORT = "/dev/ttyS0"      # or "/dev/serial0"
GPS_BAUD = 115200

# 出力
DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")

# ログ周期（まずは軽め推奨）
LOG_HZ = 10.0                 # まず10Hz（安定したら20に）
LOG_PERIOD = 1.0 / LOG_HZ

# IMU(I2C)
IMU_ADDR = 0x4B
I2C_FREQ = 50_000             # 50kHz
IMU_REPORT_INTERVAL_US = 100_000  # 100,000us=10Hz（安定後に50,000us=20Hzへ）

# IMUリトライ
IMU_REINIT_COOLDOWN_SEC = 2.0
INIT_WAIT_SEC = 2.0           # ←重要：起動待ちを長めに
UNPROCESSABLE_WAIT_SEC = 1.5

# コンソール表示間隔（秒）※ログはLOG_HZで書くが、表示は間引く
PRINT_EVERY_SEC = 1.0


# ========= 便利関数 =========
def utc_now_iso():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")


def clamp_360(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def quat_to_euler_deg(qi, qj, qk, qr):
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

    return (
        math.degrees(roll),
        math.degrees(pitch),
        clamp_360(math.degrees(yaw)),
    )


# ========= GPSスレッド =========
class GPSState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {
            "gps_fix": False,
            "lat": None,
            "lon": None,
            "alt_m": None,
            "sats": None,
            "hdop": None,
            "cog_deg": None,
            "sog_kt": None,
            "last_gga_utc": None,
            "last_rmc_utc": None,
        }

    def update(self, **kwargs):
        with self.lock:
            self.latest.update(kwargs)

    def snapshot(self):
        with self.lock:
            return dict(self.latest)


class GPSReader(threading.Thread):
    def __init__(self, state: GPSState, port: str, baud: int):
        super().__init__(daemon=True)
        self.state = state
        self.port = port
        self.baud = baud
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    while not self._stop.is_set():
                        line = ser.readline().decode(errors="ignore").strip()
                        if not line:
                            continue
                        if not (line.startswith("$GN") or line.startswith("$GP")):
                            continue
                        try:
                            msg = pynmea2.parse(line)
                        except pynmea2.nmea.ChecksumError:
                            continue
                        except Exception:
                            continue

                        if msg.sentence_type == "RMC":
                            fix = (getattr(msg, "status", "") == "A")
                            try:
                                sog = float(msg.spd_over_grnd) if msg.spd_over_grnd != "" else None
                            except Exception:
                                sog = None
                            try:
                                cog = float(msg.true_course) if msg.true_course != "" else None
                            except Exception:
                                cog = None
                            self.state.update(
                                gps_fix=fix or self.state.snapshot().get("gps_fix", False),
                                sog_kt=sog,
                                cog_deg=clamp_360(cog) if cog is not None else None,
                                last_rmc_utc=utc_now_iso(),
                            )

                        elif msg.sentence_type == "GGA":
                            try:
                                fixq = int(getattr(msg, "gps_qual", 0) or 0)
                            except Exception:
                                fixq = 0
                            fix = fixq > 0

                            lat = msg.latitude if msg.latitude != 0 else None
                            lon = msg.longitude if msg.longitude != 0 else None
                            alt = getattr(msg, "altitude", None)
                            sats = getattr(msg, "num_sats", None)
                            hdop = getattr(msg, "horizontal_dil", None)

                            self.state.update(
                                gps_fix=fix,
                                lat=lat,
                                lon=lon,
                                alt_m=alt,
                                sats=sats,
                                hdop=hdop,
                                last_gga_utc=utc_now_iso(),
                            )

            except Exception:
                time.sleep(1.0)


# ========= IMU =========
def make_i2c():
    return busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ)


def init_imu():
    last_err = None
    for attempt in range(1, 31):
        i2c = None
        try:
            i2c = make_i2c()

            # ここで作った瞬間にsoft_resetが走る環境があるので待つ
            bno = BNO08X_I2C(i2c, address=IMU_ADDR, debug=False)

            # ★重要：起動直後は不安定なので長めに待つ
            time.sleep(INIT_WAIT_SEC)

            # report_interval は「整数マイクロ秒」必須
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=IMU_REPORT_INTERVAL_US)

            # 空読みを数回（パケットが揃うまで待つ）
            for _ in range(5):
                try:
                    _ = bno.quaternion
                    break
                except Exception:
                    time.sleep(0.1)

            return bno, i2c

        except Exception as e:
            last_err = e
            msg = str(e)
            print(f"IMU init fail {attempt}/30: {msg}")

            # よく出るやつは少し長めに待つ
            if "Unprocessable Batch bytes" in msg:
                time.sleep(UNPROCESSABLE_WAIT_SEC)
            else:
                time.sleep(0.4)

            try:
                if i2c is not None:
                    i2c.deinit()
            except Exception:
                pass

    raise RuntimeError(f"IMU init failed: {last_err}")


# ========= メイン =========
def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    gps_state = GPSState()
    gps_thread = GPSReader(gps_state, GPS_PORT, GPS_BAUD)
    gps_thread.start()

    with open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow([
                "time_utc",
                "gps_fix", "lat", "lon", "alt_m", "sats", "hdop", "cog_deg", "sog_kt",
                "last_gga_utc", "last_rmc_utc",
                "imu_ok",
                "qi", "qj", "qk", "qr",
                "roll_deg", "pitch_deg", "yaw_deg",
            ])

        print(f"Start logging -> {CSV_PATH}")
        print(f"LOG_HZ={LOG_HZ}, GPS={GPS_PORT}@{GPS_BAUD}, IMU=0x{IMU_ADDR:02X} I2C={I2C_FREQ}Hz")

        bno = None
        i2c = None
        next_imu_init_time = 0.0

        next_tick = time.monotonic()
        last_print = 0.0

        try:
            while True:
                # ---- タイマー（遅れたら追いつこうとしない）----
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                else:
                    # 遅れたら次の周期にリセット（print爆発防止）
                    next_tick = now
                next_tick += LOG_PERIOD

                # ---- IMU init ----
                if bno is None and time.monotonic() >= next_imu_init_time:
                    try:
                        bno, i2c = init_imu()
                        print(f"IMU ready at 0x{IMU_ADDR:02X}")
                    except Exception as e:
                        print(f"IMU glitch: {e} -> retry later")
                        bno = None
                        i2c = None
                        next_imu_init_time = time.monotonic() + IMU_REINIT_COOLDOWN_SEC

                # ---- IMU read ----
                imu_ok = False
                qi = qj = qk = qr = None
                roll = pitch = yaw = None

                if bno is not None:
                    try:
                        qi, qj, qk, qr = bno.quaternion
                        roll, pitch, yaw = quat_to_euler_deg(qi, qj, qk, qr)
                        imu_ok = True
                    except Exception as e:
                        print(f"IMU read error: {e} -> drop and re-init")
                        try:
                            if i2c is not None:
                                i2c.deinit()
                        except Exception:
                            pass
                        bno = None
                        i2c = None
                        next_imu_init_time = time.monotonic() + IMU_REINIT_COOLDOWN_SEC

                # ---- GPS snapshot ----
                g = gps_state.snapshot()

                # ---- 書き込み ----
                t_utc = utc_now_iso()
                writer.writerow([
                    t_utc,
                    g["gps_fix"], g["lat"], g["lon"], g["alt_m"], g["sats"], g["hdop"], g["cog_deg"], g["sog_kt"],
                    g["last_gga_utc"], g["last_rmc_utc"],
                    imu_ok,
                    qi, qj, qk, qr,
                    None if roll is None else round(roll, 2),
                    None if pitch is None else round(pitch, 2),
                    None if yaw is None else round(yaw, 2),
                ])
                f.flush()

                # ---- 表示は間引く ----
                pnow = time.monotonic()
                if pnow - last_print >= PRINT_EVERY_SEC:
                    last_print = pnow
                    if imu_ok and g["gps_fix"] and g["lat"] is not None:
                        print(f"{t_utc} lat={g['lat']:.6f}, lon={g['lon']:.6f} yaw={yaw:.1f}")
                    elif imu_ok:
                        print(f"{t_utc} GPS={'FIX' if g['gps_fix'] else 'no'} yaw={yaw:.1f}")
                    else:
                        print(f"{t_utc} GPS={'FIX' if g['gps_fix'] else 'no'} IMU=down")

        except KeyboardInterrupt:
            print("Stopped.")
        finally:
            gps_thread.stop()
            try:
                gps_thread.join(timeout=1.0)
            except Exception:
                pass
            try:
                if i2c is not None:
                    i2c.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()
