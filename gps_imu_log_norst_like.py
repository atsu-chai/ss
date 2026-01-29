#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import csv
import math
import datetime
import serial
import pynmea2

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# ========= Settings =========
GPS_PORT = "/dev/ttyS0"
GPS_BAUD = 115200

IMU_ADDR = 0x4B
REPORT_US = 200_000  # 200ms（負荷低め）

DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")
LOG_HZ = 5  # 5HzでCSV記録（GPS/IMUは内部的に随時更新）
# ============================


def utc_now_iso_ms():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")


def quat_to_yaw_deg(qx, qy, qz, qw):
    # yaw (Z) from quaternion
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    deg = math.degrees(yaw)
    return deg + 360.0 if deg < 0 else deg


def init_bno_like_yours(i2c):
    # あなたの「安定する」初期化を踏襲
    time.sleep(4.0)

    last = None
    for k in range(40):
        try:
            bno = BNO08X_I2C(i2c, address=IMU_ADDR, debug=False)
            time.sleep(1.0)

            for j in range(30):
                try:
                    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=REPORT_US)
                    print("✅ rotation vector enabled")
                    # 1回読めることを確認
                    qx, qy, qz, qw = bno.quaternion
                    _ = quat_to_yaw_deg(qx, qy, qz, qw)
                    return bno
                except Exception as e:
                    last = e
                    print(f"⚠️ enable retry {j+1}/30: {e}")
                    time.sleep(0.4)

            raise RuntimeError(f"enable_feature failed: {last}")

        except Exception as e:
            last = e
            print(f"⚠️ init retry {k+1}/40: {e}")
            time.sleep(0.6)

    raise RuntimeError(f"init failed: {last}")


def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.2)
    i2c = busio.I2C(board.SCL, board.SDA)

    # GPS state
    gps_fix = "NOFIX"
    lat = lon = alt = None
    sats = hdop = None
    cog = sog = None
    last_gps_rx = 0.0

    # IMU state
    bno = None
    imu_status = "down"
    imu_err = ""
    yaw_deg = None
    q = (None, None, None, None)

    fail = 0
    next_init_try = 0.0

    log_period = 1.0 / max(LOG_HZ, 1)
    next_log = time.monotonic()

    print(f"Logging to: {CSV_PATH}")
    print("Running... Ctrl+C to stop")

    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow([
                "time_utc",
                "gps_fix","lat","lon","alt_m","sats","hdop","cog_deg","sog_kt",
                "imu_status","yaw_deg","qx","qy","qz","qw","imu_err",
            ])

        try:
            while True:
                now = time.monotonic()

                # ===== GPS read =====
                try:
                    line = gps.readline().decode(errors="ignore").strip()
                    if line and (line.startswith("$GN") or line.startswith("$GP")):
                        msg = pynmea2.parse(line)
                        last_gps_rx = now

                        if msg.sentence_type == "GGA":
                            ql = getattr(msg, "gps_qual", None)
                            gps_fix = "FIX" if (ql and str(ql) != "0") else "NOFIX"
                            lat = msg.latitude if msg.latitude != 0 else None
                            lon = msg.longitude if msg.longitude != 0 else None
                            alt = getattr(msg, "altitude", None)
                            sats = getattr(msg, "num_sats", None)
                            hdop = getattr(msg, "horizontal_dil", None)

                        elif msg.sentence_type == "VTG":
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
                except Exception:
                    pass

                if (now - last_gps_rx) > 2.0:
                    gps_fix = "NOFIX"

                # ===== IMU init/read =====
                if bno is None and now >= next_init_try:
                    try:
                        print("init IMU...")
                        bno = init_bno_like_yours(i2c)
                        imu_status = "ok"
                        imu_err = ""
                        fail = 0
                        print(f"✅ IMU ready at 0x{IMU_ADDR:02X}")
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"init:{e}"
                        fail += 1
                        # あなたの元コードに合わせてゆっくり再試行
                        next_init_try = now + 1.0
                        if fail >= 3:
                            print("TIP: 0x4Bがi2cdetectで出るのに初期化できない時は、IMUの3.3Vだけ抜いて5〜10秒→挿し直しで復帰しやすいです。")
                            fail = 0

                if bno is not None:
                    try:
                        qx, qy, qz, qw = bno.quaternion
                        q = (qx, qy, qz, qw)
                        yaw_deg = quat_to_yaw_deg(qx, qy, qz, qw)
                        imu_status = "ok"
                        imu_err = ""
                    except (TimeoutError, OSError, RuntimeError, KeyError, ValueError) as e:
                        imu_status = "down"
                        imu_err = f"read:{e}"
                        bno = None
                        next_init_try = now + 1.0

                # ===== CSV log + console =====
                if now >= next_log:
                    next_log += log_period
                    t = utc_now_iso_ms()

                    w.writerow([
                        t, gps_fix,
                        f"{lat:.6f}" if isinstance(lat,(int,float)) else "",
                        f"{lon:.6f}" if isinstance(lon,(int,float)) else "",
                        alt if alt is not None else "",
                        sats if sats is not None else "",
                        hdop if hdop is not None else "",
                        f"{cog:.1f}" if isinstance(cog,(int,float)) else "",
                        f"{sog:.2f}" if isinstance(sog,(int,float)) else "",
                        imu_status,
                        f"{yaw_deg:.1f}" if isinstance(yaw_deg,(int,float)) else "",
                        f"{q[0]:.6f}" if isinstance(q[0],(int,float)) else "",
                        f"{q[1]:.6f}" if isinstance(q[1],(int,float)) else "",
                        f"{q[2]:.6f}" if isinstance(q[2],(int,float)) else "",
                        f"{q[3]:.6f}" if isinstance(q[3],(int,float)) else "",
                        imu_err,
                    ])
                    f.flush()

                    # 画面は軽めに
                    extra = f" err={imu_err}" if (imu_status == "down" and imu_err) else ""
                    print(f"{t} GPS={gps_fix} IMU={imu_status}{extra}")

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            try:
                gps.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
