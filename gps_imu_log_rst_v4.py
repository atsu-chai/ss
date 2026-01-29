#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import csv
import math
import datetime
import serial
import pynmea2

# --- GPS settings ---
GPS_PORT = "/dev/ttyS0"   # or "/dev/serial0"
GPS_BAUD = 115200

# --- IMU (BNO08x) settings ---
IMU_ADDR = 0x4B           # i2cdetectで見えたアドレス
RST_GPIO = 17             # BCM番号。使わないなら None にする（例: RST_GPIO=None）

# --- logging ---
DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")
LOG_HZ = 5                # 1秒あたり何行書くか（例: 5Hz）
REINIT_COOLDOWN_SEC = 5.0 # IMU再初期化の最短間隔

# --- BNO08x import (inside to keep startup robust) ---
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# --- GPIO reset ---
def setup_gpio_reset(rst_gpio: int | None):
    if rst_gpio is None:
        return None
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(rst_gpio, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = 非リセット(通常動作)
    return GPIO

def hw_reset(GPIO, rst_gpio: int | None):
    """BNO08xのRSTは通常 active-low: LOWでリセット"""
    if GPIO is None or rst_gpio is None:
        return
    GPIO.output(rst_gpio, GPIO.LOW)
    time.sleep(0.12)
    GPIO.output(rst_gpio, GPIO.HIGH)
    time.sleep(0.7)  # 起動待ち（短いと不安定になりやすい）

def utc_now_iso_ms():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")

def quat_to_yaw_deg(qx, qy, qz, qw):
    # yaw(Z) = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    deg = math.degrees(yaw)
    if deg < 0:
        deg += 360.0
    return deg

def init_bno(i2c, GPIO, rst_gpio):
    # ハードリセット → 初期化
    hw_reset(GPIO, rst_gpio)

    bno = BNO08X_I2C(i2c, address=IMU_ADDR)  # debugは出したくないので指定しない/Falseのまま
    # このライブラリは report_interval が「整数(μs)」想定
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)  # 50ms = 20Hz
    return bno

def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    GPIO = setup_gpio_reset(RST_GPIO)

    gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.2)

    # I2Cは最初に作る（落ちたら作り直す）
    i2c = busio.I2C(board.SCL, board.SDA)

    bno = None
    imu_status = "down"
    imu_err = ""
    last_imu_try = 0.0

    # GPS state（最後に受けた値を保持）
    gps_fix = "NOFIX"
    lat = lon = alt = None
    sats = hdop = None
    cog = sog = None  # COG deg, SOG knots
    last_gps_rx = 0.0

    # IMU state
    yaw_deg = None
    last_imu_rx = 0.0

    # CSV open
    with open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow([
                "time_utc",
                "gps_fix", "lat", "lon", "alt_m", "sats", "hdop", "cog_deg", "sog_kt",
                "imu_status", "yaw_deg",
                "imu_err"
            ])

        print(f"Logging to: {CSV_PATH}")
        print("Running... Ctrl+C to stop")

        log_period = 1.0 / max(LOG_HZ, 1)
        next_log = time.monotonic()

        try:
            while True:
                now_mono = time.monotonic()

                # --- GPS: read one line (timeout短めなので回せる) ---
                try:
                    line = gps.readline().decode(errors="ignore").strip()
                    if line and (line.startswith("$GN") or line.startswith("$GP")):
                        msg = pynmea2.parse(line)
                        last_gps_rx = now_mono

                        if msg.sentence_type == "GGA":
                            # fix品質が0ならNOFIX扱い
                            q = getattr(msg, "gps_qual", None)
                            gps_fix = "FIX" if (q and str(q) != "0") else "NOFIX"

                            lat = msg.latitude if msg.latitude != 0 else None
                            lon = msg.longitude if msg.longitude != 0 else None
                            alt = getattr(msg, "altitude", None)
                            sats = getattr(msg, "num_sats", None)
                            hdop = getattr(msg, "horizontal_dil", None)

                        elif msg.sentence_type == "VTG":
                            # COG true / speed knots
                            try:
                                cog_v = float(msg.true_track) if msg.true_track not in (None, "") else None
                            except Exception:
                                cog_v = None
                            try:
                                sog_v = float(msg.spd_over_grnd_kts) if msg.spd_over_grnd_kts not in (None, "") else None
                            except Exception:
                                sog_v = None
                            if cog_v is not None:
                                cog = cog_v
                            if sog_v is not None:
                                sog = sog_v

                except pynmea2.nmea.ChecksumError:
                    pass
                except Exception:
                    # GPSは多少落ちても継続
                    pass

                # --- IMU: read yaw ---
                if bno is None and (now_mono - last_imu_try) >= REINIT_COOLDOWN_SEC:
                    last_imu_try = now_mono
                    try:
                        # I2C ready待ち（busio.I2Cはロックがある）
                        # ここで固まることがあるので短く回す
                        if not i2c.try_lock():
                            # 次ループで再試行
                            pass
                        else:
                            i2c.unlock()

                        bno = init_bno(i2c, GPIO, RST_GPIO)
                        imu_status = "ok"
                        imu_err = ""
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"init:{e}"

                if bno is not None:
                    try:
                        qx, qy, qz, qw = bno.quaternion  # (i, j, k, real)
                        yaw_deg = quat_to_yaw_deg(qx, qy, qz, qw)
                        last_imu_rx = now_mono
                        imu_status = "ok"
                        imu_err = ""
                    except Exception as e:
                        # ここで落ちたら一旦捨てて再初期化へ
                        bno = None
                        imu_status = "down"
                        imu_err = f"read:{e}"

                # --- periodic log ---
                if now_mono >= next_log:
                    next_log += log_period
                    t = utc_now_iso_ms()

                    # GPSの受信が途切れてたら状態をそれっぽく
                    if (now_mono - last_gps_rx) > 2.0:
                        gps_fix = "NOFIX"

                    row = [
                        t,
                        gps_fix,
                        f"{lat:.6f}" if isinstance(lat, (int, float)) else "",
                        f"{lon:.6f}" if isinstance(lon, (int, float)) else "",
                        alt if alt is not None else "",
                        sats if sats is not None else "",
                        hdop if hdop is not None else "",
                        f"{cog:.1f}" if isinstance(cog, (int, float)) else "",
                        f"{sog:.2f}" if isinstance(sog, (int, float)) else "",
                        imu_status,
                        f"{yaw_deg:.1f}" if isinstance(yaw_deg, (int, float)) else "",
                        imu_err
                    ]
                    writer.writerow(row)
                    f.flush()

                    # 画面表示（必要最小限）
                    print(
                        f"{t} GPS={gps_fix} IMU={imu_status}"
                        + (f" lat={lat:.6f} lon={lon:.6f}" if isinstance(lat, (int, float)) else "")
                        + (f" yaw={yaw_deg:.1f}" if isinstance(yaw_deg, (int, float)) else "")
                        + (f" err={imu_err}" if imu_err else "")
                    )

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            try:
                gps.close()
            except Exception:
                pass
            if GPIO is not None:
                try:
                    GPIO.cleanup()
                except Exception:
                    pass

if __name__ == "__main__":
    main()
