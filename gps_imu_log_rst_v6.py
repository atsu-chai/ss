#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, csv, math, datetime
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
RST_GPIO = 17          # ★切り分け用：まずは None で試してOK
RST_ACTIVE_LOW = True  # True/False 切替（RST使う場合）

DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")
LOG_HZ = 5
# ============================

def utc_now_iso_ms():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")

def quat_to_yaw_deg(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    deg = math.degrees(yaw)
    return deg + 360.0 if deg < 0 else deg

def setup_gpio_reset(rst_gpio):
    if rst_gpio is None:
        return None
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # 非リセット状態にしておく
    idle_level = GPIO.HIGH if RST_ACTIVE_LOW else GPIO.LOW
    GPIO.setup(rst_gpio, GPIO.OUT, initial=idle_level)
    return GPIO

def hw_reset(GPIO, rst_gpio):
    if GPIO is None or rst_gpio is None:
        return

    assert_level = GPIO.LOW if RST_ACTIVE_LOW else GPIO.HIGH
    idle_level   = GPIO.HIGH if RST_ACTIVE_LOW else GPIO.LOW

    GPIO.output(rst_gpio, assert_level)
    time.sleep(0.30)
    GPIO.output(rst_gpio, idle_level)

    # BNO08xは起動待ち長めが安定
    time.sleep(2.5)

def i2c_has_addr(i2c, addr):
    try:
        if i2c.try_lock():
            addrs = i2c.scan()
            i2c.unlock()
            return addr in addrs, addrs
    except Exception:
        pass
    return False, []

def imu_init_try(i2c, GPIO, rst_gpio):
    # 先に scan して、見えてなければ即アウト
    ok, addrs = i2c_has_addr(i2c, IMU_ADDR)
    if not ok:
        raise RuntimeError(f"I2C scan: 0x{IMU_ADDR:02x} not found (found={','.join(hex(a) for a in addrs)})")

    # RSTがあるならハードリセット
    hw_reset(GPIO, rst_gpio)

    # リセット後もう一度scan（ここで消えるならRSTが怪しい）
    ok2, addrs2 = i2c_has_addr(i2c, IMU_ADDR)
    if not ok2:
        raise RuntimeError(f"After RST, 0x{IMU_ADDR:02x} missing (found={','.join(hex(a) for a in addrs2)})")

    bno = BNO08X_I2C(i2c, address=IMU_ADDR, debug=False)
    time.sleep(0.2)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)

    # 1回読めるかチェック
    qx, qy, qz, qw = bno.quaternion
    _ = quat_to_yaw_deg(qx, qy, qz, qw)
    return bno

def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    GPIO = setup_gpio_reset(RST_GPIO)
    gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.2)
    i2c = busio.I2C(board.SCL, board.SDA)

    gps_fix = "NOFIX"
    lat = lon = alt = None
    sats = hdop = None
    cog = sog = None
    last_gps_rx = 0.0

    bno = None
    imu_status = "down"
    imu_err = ""
    yaw_deg = None

    next_imu_try = 0.0
    backoff = 1.0

    log_period = 1.0 / max(LOG_HZ, 1)
    next_log = time.monotonic()

    last_print = ""
    def print_if_changed(s):
        nonlocal last_print
        if s != last_print:
            print(s)
            last_print = s

    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow([
                "time_utc",
                "gps_fix","lat","lon","alt_m","sats","hdop","cog_deg","sog_kt",
                "imu_status","yaw_deg","imu_err",
                "imu_addr","rst_gpio","rst_active_low"
            ])

        print(f"Logging to: {CSV_PATH}")
        print("Running... Ctrl+C to stop")

        try:
            while True:
                now = time.monotonic()

                # GPS
                try:
                    line = gps.readline().decode(errors="ignore").strip()
                    if line and (line.startswith("$GN") or line.startswith("$GP")):
                        msg = pynmea2.parse(line)
                        last_gps_rx = now

                        if msg.sentence_type == "GGA":
                            q = getattr(msg, "gps_qual", None)
                            gps_fix = "FIX" if (q and str(q) != "0") else "NOFIX"
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
                            if cog_v is not None: cog = cog_v
                            if sog_v is not None: sog = sog_v
                except Exception:
                    pass

                if (now - last_gps_rx) > 2.0:
                    gps_fix = "NOFIX"

                # IMU init/read
                if bno is None and now >= next_imu_try:
                    try:
                        bno = imu_init_try(i2c, GPIO, RST_GPIO)
                        imu_status = "ok"
                        imu_err = ""
                        backoff = 1.0
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"init:{e}"
                        next_imu_try = now + backoff
                        backoff = min(backoff * 2.0, 30.0)

                if bno is not None:
                    try:
                        qx, qy, qz, qw = bno.quaternion
                        yaw_deg = quat_to_yaw_deg(qx, qy, qz, qw)
                        imu_status = "ok"
                        imu_err = ""
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"read:{e}"
                        next_imu_try = now + backoff
                        backoff = min(backoff * 2.0, 30.0)

                # log
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
                        imu_err,
                        hex(IMU_ADDR),
                        RST_GPIO if RST_GPIO is not None else "",
                        RST_ACTIVE_LOW
                    ])
                    f.flush()

                    # 画面表示は「状態変化した時だけ」
                    extra = ""
                    if imu_status == "down" and imu_err:
                        extra = f" {imu_err} next_try={max(0.0, next_imu_try-now):.1f}s"
                    print_if_changed(f"{t} GPS={gps_fix} IMU={imu_status}{extra}")

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            try: gps.close()
            except Exception: pass
            if GPIO is not None:
                try: GPIO.cleanup()
                except Exception: pass

if __name__ == "__main__":
    main()
