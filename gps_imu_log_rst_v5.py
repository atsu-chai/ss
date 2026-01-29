#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import csv
import math
import datetime
import serial
import pynmea2

# ========= User settings =========
# --- GPS ---
GPS_PORT = "/dev/ttyS0"     # or "/dev/serial0"
GPS_BAUD = 115200

# --- IMU (BNO08x) ---
IMU_ADDR = 0x4B             # i2cdetectで見えたやつ
RST_GPIO = 17               # BCM番号。RST未接続なら None
RST_ACTIVE_LOW = True       # ★ここが重要：通常は True(LOWでリセット)。ダメなら False を試す

# --- Logging ---
DATA_DIR = "/home/atsu/data"
CSV_PATH = os.path.join(DATA_DIR, "gps_imu_log.csv")
LOG_HZ = 5                  # 1秒あたり何行ログするか
# ================================

# Adafruit BNO08x
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


def utc_now_iso_ms():
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds")


def quat_to_yaw_deg(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    deg = math.degrees(yaw)
    return deg + 360.0 if deg < 0 else deg


# ---------- GPIO Reset ----------
def setup_gpio_reset(rst_gpio):
    if rst_gpio is None:
        return None
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # 初期状態は「非リセット」
    idle = True
    GPIO.setup(rst_gpio, GPIO.OUT, initial=GPIO.HIGH if (RST_ACTIVE_LOW and idle) else GPIO.LOW)
    return GPIO


def hw_reset(GPIO, rst_gpio):
    """RSTをパルス。極性は RST_ACTIVE_LOW で切替。"""
    if GPIO is None or rst_gpio is None:
        return

    # アイドル(非リセット)とアサート(リセット)を定義
    if RST_ACTIVE_LOW:
        assert_level = GPIO.LOW
        idle_level = GPIO.HIGH
    else:
        assert_level = GPIO.HIGH
        idle_level = GPIO.LOW

    # パルス長は少し長めに
    GPIO.output(rst_gpio, assert_level)
    time.sleep(0.30)
    GPIO.output(rst_gpio, idle_level)

    # 起動待ち：短いと壊れやすいので長め
    time.sleep(2.0)


# ---------- I2C bus recovery ----------
def i2c_recover(GPIO=None):
    """
    I2Cがゴミ状態のときにSCLを9回トグルして復帰を狙う。
    Raspberry PiのSCL/SDA(BCM3/2)をGPIOとして直接触るので最後に戻す。
    """
    try:
        import RPi.GPIO as RGPIO
    except Exception:
        return

    SCL = 3  # BCM
    SDA = 2  # BCM

    RGPIO.setwarnings(False)
    RGPIO.setmode(RGPIO.BCM)

    # open-drainっぽく動かす：入力(プルアップ)が基本、Lowにしたい時だけ出力Low
    def drive_low(pin):
        RGPIO.setup(pin, RGPIO.OUT)
        RGPIO.output(pin, RGPIO.LOW)

    def release(pin):
        RGPIO.setup(pin, RGPIO.IN, pull_up_down=RGPIO.PUD_UP)

    release(SCL)
    release(SDA)
    time.sleep(0.01)

    # SDAがLowならSCLパルスで解放を狙う
    if RGPIO.input(SDA) == 0:
        for _ in range(9):
            drive_low(SCL)
            time.sleep(0.002)
            release(SCL)
            time.sleep(0.002)

    # STOPコンディション風（SCL Highの状態でSDAをHighへ）
    release(SCL)
    time.sleep(0.002)
    release(SDA)
    time.sleep(0.01)

    # cleanupはしない（他GPIO使ってるので）。この関数内で設定は戻してる。


def imu_init_try(i2c, GPIO, rst_gpio):
    """
    1回だけIMU初期化を試す。
    失敗したら例外を投げる（呼び出し側でバックオフ）。
    """
    # まずバス復旧を一回
    i2c_recover()

    # ハードリセット
    hw_reset(GPIO, rst_gpio)

    # 生成。debugは明示的にFalse
    bno = BNO08X_I2C(i2c, address=IMU_ADDR, debug=False)

    # 起動直後はまだ不安定なことがあるので少し待つ
    time.sleep(0.2)

    # report_interval は整数(μs)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=50000)  # 50ms=20Hz

    # 1回読めるか確認（読めなければ初期化失敗扱いにする）
    qx, qy, qz, qw = bno.quaternion
    _ = quat_to_yaw_deg(qx, qy, qz, qw)

    return bno


def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    GPIO = setup_gpio_reset(RST_GPIO)

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
    last_imu_ok = 0.0

    # 初期化バックオフ
    next_imu_try = 0.0
    backoff = 1.0  # 秒。失敗するほど増える（最大30秒）

    log_period = 1.0 / max(LOG_HZ, 1)
    next_log = time.monotonic()

    with open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow([
                "time_utc",
                "gps_fix", "lat", "lon", "alt_m", "sats", "hdop", "cog_deg", "sog_kt",
                "imu_status", "yaw_deg",
                "imu_err",
                "imu_addr", "rst_gpio", "rst_active_low"
            ])

        print(f"Logging to: {CSV_PATH}")
        print("Running... Ctrl+C to stop")

        try:
            while True:
                now = time.monotonic()

                # ---- GPS ----
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
                            if cog_v is not None:
                                cog = cog_v
                            if sog_v is not None:
                                sog = sog_v
                except pynmea2.nmea.ChecksumError:
                    pass
                except Exception:
                    pass

                # ---- IMU init / read ----
                if bno is None and now >= next_imu_try:
                    try:
                        # I2Cロックを軽く確認（取れなくても次周回でOK）
                        if i2c.try_lock():
                            i2c.unlock()

                        bno = imu_init_try(i2c, GPIO, RST_GPIO)
                        imu_status = "ok"
                        imu_err = ""
                        backoff = 1.0  # 成功したら戻す
                        last_imu_ok = now
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"init:{e}"
                        next_imu_try = now + backoff
                        backoff = min(backoff * 2.0, 30.0)  # 最大30秒

                if bno is not None:
                    try:
                        qx, qy, qz, qw = bno.quaternion
                        yaw_deg = quat_to_yaw_deg(qx, qy, qz, qw)
                        imu_status = "ok"
                        imu_err = ""
                        last_imu_ok = now
                    except Exception as e:
                        bno = None
                        imu_status = "down"
                        imu_err = f"read:{e}"
                        next_imu_try = now + backoff
                        backoff = min(backoff * 2.0, 30.0)

                # ---- periodic log ----
                if now >= next_log:
                    next_log += log_period
                    t = utc_now_iso_ms()

                    if (now - last_gps_rx) > 2.0:
                        gps_fix = "NOFIX"

                    writer.writerow([
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
                        imu_err,
                        hex(IMU_ADDR),
                        RST_GPIO if RST_GPIO is not None else "",
                        RST_ACTIVE_LOW
                    ])
                    f.flush()

                    # 表示は「状態が変わった時」＋「エラー時はたまに」くらいにしたいが、
                    # まずは今の問題切り分け優先で1行出す
                    extra = ""
                    if imu_status == "down" and imu_err:
                        extra = f" err={imu_err} next_try={max(0.0, next_imu_try-now):.1f}s backoff={backoff:.1f}s"
                    print(f"{t} GPS={gps_fix} IMU={imu_status}{extra}")

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
