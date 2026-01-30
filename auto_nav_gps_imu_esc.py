#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import serial
import pigpio
import pynmea2

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


# =========================
# GPS config（あなたの現状に合わせ）
# =========================
GPS_PORT = "/dev/ttyS0"      # or "/dev/serial0"
GPS_BAUDRATE = 115200

# =========================
# 目的地（ここだけ変える）
# =========================
TARGET_LAT = 35.681236
TARGET_LON = 139.767125

# =========================
# ナビ制御パラメータ
# =========================
STOP_RADIUS_M = 1.5
SPEED_FOR_COURSE_MS = 1.5     # これ以上の速度ならGPSコースを信頼してIMUオフセット合わせ

KP_YAW = 2.0                  # 旋回ゲイン
KP_V = 0.35                   # 前進ゲイン
MAX_W = 1.2                   # 最大旋回(内部)
MAX_V = 1.0                   # 最大前進(内部)

SLOW_ANGLE_DEG = 35.0         # これ以上ズレたら減速
SPIN_IN_PLACE_DEG = 60.0      # これ以上ズレたら前進0でその場旋回

# ESCに入れる power のスケール（強すぎる場合ここを下げる）
POWER_SCALE_FWD = 0.7
POWER_SCALE_TURN = 0.8
POWER_DEADBAND = 0.03         # 小さい出力を0に潰してガタつき防止

PRINT_INTERVAL = 0.2

EARTH_R = 6371000.0


# =========================
# pigpio / ESC config（あなたの現状に合わせ）
# =========================
ESC_RIGHT_GPIO = 19
ESC_LEFT_GPIO  = 18

PULSE_STOP     = 1480
PULSE_MIN_FWD  = 1520
PULSE_MAX_FWD  = 2000
PULSE_MIN_REV  = 1440
PULSE_MAX_REV  = 1000

ARM_PULSE      = 1000


# =========================
# ユーティリティ
# =========================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap360(deg: float) -> float:
    deg %= 360.0
    return deg + 360.0 if deg < 0 else deg

def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0

def haversine_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dl/2)**2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dl)
    return wrap360(math.degrees(math.atan2(y, x)))  # 北=0, 東=+

def quat_to_yaw_deg(q):
    # bno.quaternion は (x, y, z, w) のことが多い
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return wrap360(math.degrees(yaw))


# =========================
# BNO086 init
# =========================
def init_bno08x():
    i2c = busio.I2C(board.SCL, board.SDA)
    for addr in (0x4B, 0x4A):
        try:
            bno = BNO08X_I2C(i2c, address=addr)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            return bno, addr
        except Exception:
            pass
    raise RuntimeError("BNO086 not found on I2C (0x4B/0x4A).")


# =========================
# ESC制御（あなたの関数を流用）
# =========================
def motor_power_to_pulse(power: float) -> int:
    power = max(-1.0, min(1.0, power))
    if power > 0.0:
        return int(PULSE_MIN_FWD + (PULSE_MAX_FWD - PULSE_MIN_FWD) * power)
    elif power < 0.0:
        rev_power = -power
        return int(PULSE_MIN_REV - (PULSE_MIN_REV - PULSE_MAX_REV) * rev_power)
    else:
        return PULSE_STOP

def set_motor(pi: pigpio.pi, gpio: int, power: float) -> None:
    if abs(power) < POWER_DEADBAND:
        power = 0.0
    pulse = motor_power_to_pulse(power)
    pi.set_servo_pulsewidth(gpio, pulse)

def mix_forward_turn(forward: float, turn: float):
    left  = forward + turn
    right = forward - turn
    left  = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    return left, right

def stop_all(pi: pigpio.pi) -> None:
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, 0)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, 0)

def stop_hold(pi: pigpio.pi) -> None:
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, PULSE_STOP)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, PULSE_STOP)

def arm_escs(pi: pigpio.pi):
    print("\nArming ESCs with ARM_PULSE...")
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, ARM_PULSE)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, ARM_PULSE)
    time.sleep(5)
    print("Set STOP.")
    stop_hold(pi)
    time.sleep(2)


# =========================
# メイン
# =========================
def main():
    # pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpio. Run `sudo pigpiod` first.")
        return

    # GPS
    ser = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)

    # IMU
    bno, addr = init_bno08x()

    # 安全停止
    stop_all(pi)
    time.sleep(1)

    print("=== AUTO NAV (GT-505/GNSS + BNO086 + Dual ESC diff-drive) ===")
    print(f"GPS: {GPS_PORT} @ {GPS_BAUDRATE}")
    print(f"BNO086: I2C addr 0x{addr:02X}")
    print(f"TARGET: lat={TARGET_LAT}, lon={TARGET_LON}")
    print("* 安全のため、最初は浮かせる/タイヤ空転できる状態で！")
    input("ReadyならEnter...")

    arm_escs(pi)

    # 最新値（GPS）
    lat = lon = float("nan")
    alt = float("nan")
    sats = 0
    hdop = float("nan")

    speed_ms = float("nan")
    course_deg = float("nan")
    gps_valid = False

    # IMU
    imu_heading = float("nan")
    imu_reverse = False  # 旋回が逆なら True にする
    offset_locked = False
    heading_offset = 0.0

    last_print = 0.0
    slow_angle_rad = math.radians(SLOW_ANGLE_DEG)

    print("time_ms,lat,lon,dist_m,bearing_deg,imu_corr_deg,err_deg,fwd,turn,left,right,speed_ms,course_deg,sats,hdop,offset_locked")

    try:
        while True:
            # -----------------
            # GPS parse（あなたのgps_parse.pyの流れ＋RMC追加）
            # -----------------
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                if not (line.startswith("$GN") or line.startswith("$GP")):
                    continue

                try:
                    msg = pynmea2.parse(line)
                except pynmea2.nmea.ChecksumError:
                    continue

                # GGA：位置＋衛星数＋HDOP
                if msg.sentence_type == "GGA":
                    if msg.latitude and msg.longitude:
                        lat = msg.latitude
                        lon = msg.longitude
                        gps_valid = True
                    alt = float(getattr(msg, "altitude", float("nan")) or float("nan"))
                    sats = int(getattr(msg, "num_sats", 0) or 0)
                    hdop_val = getattr(msg, "horizontal_dil", None)
                    try:
                        hdop = float(hdop_val) if hdop_val is not None else float("nan")
                    except Exception:
                        hdop = float("nan")

                # RMC：速度＋進行コース（IMUオフセット合わせに使う）
                elif msg.sentence_type == "RMC":
                    # status A=valid
                    if getattr(msg, "status", "") == "A":
                        gps_valid = True
                        # speed knots -> m/s
                        if msg.spd_over_grnd:
                            speed_ms = float(msg.spd_over_grnd) * 0.514444
                        if msg.true_course:
                            course_deg = float(msg.true_course)

            except Exception:
                # パース系は落とさず続行
                pass

            # -----------------
            # IMU read（BNO086）
            # -----------------
            try:
                q = bno.quaternion
                if q is not None:
                    imu_heading = quat_to_yaw_deg(q)
                    if imu_reverse:
                        imu_heading = wrap360(360.0 - imu_heading)
            except Exception:
                pass

            # -----------------
            # GPSコースでIMUのオフセットを1回合わせ（走行中のみ）
            # -----------------
            if (not offset_locked
                and gps_valid
                and math.isfinite(imu_heading)
                and math.isfinite(course_deg)
                and math.isfinite(speed_ms)
                and speed_ms > SPEED_FOR_COURSE_MS):
                heading_offset = wrap180(course_deg - imu_heading)
                offset_locked = True

            imu_corr = float("nan")
            if math.isfinite(imu_heading):
                imu_corr = wrap360(imu_heading + heading_offset)

            # -----------------
            # ナビ：fwd/turn -> 左右ESC
            # -----------------
            fwd = 0.0
            turn = 0.0
            left = right = 0.0

            dist_m = float("nan")
            brng = float("nan")
            err_deg = float("nan")

            if gps_valid and math.isfinite(lat) and math.isfinite(lon) and math.isfinite(imu_corr):
                dist_m = haversine_m(lat, lon, TARGET_LAT, TARGET_LON)

                if dist_m <= STOP_RADIUS_M:
                    fwd, turn = 0.0, 0.0
                else:
                    brng = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
                    err_deg = wrap180(brng - imu_corr)  # +なら右へ向けたい

                    # 旋回量（内部）
                    err_rad = math.radians(err_deg)
                    w = clamp(KP_YAW * err_rad, -MAX_W, MAX_W)

                    # 前進量（内部）
                    v_base = clamp(KP_V * dist_m, 0.0, MAX_V)

                    # 大きくズレてるならその場旋回
                    if abs(err_deg) >= SPIN_IN_PLACE_DEG:
                        v = 0.0
                    else:
                        ang = abs(err_rad)
                        if ang >= slow_angle_rad:
                            v = 0.15 * v_base
                        else:
                            scale = 1.0 - 0.85 * (ang / slow_angle_rad)
                            v = v_base * clamp(scale, 0.15, 1.0)

                    # ESC用に [-1..1] 正規化
                    fwd  = clamp((v / MAX_V) * POWER_SCALE_FWD,  -1.0, 1.0)
                    turn = clamp((w / MAX_W) * POWER_SCALE_TURN, -1.0, 1.0)

                left, right = mix_forward_turn(fwd, turn)
                set_motor(pi, ESC_LEFT_GPIO, left)
                set_motor(pi, ESC_RIGHT_GPIO, right)

            else:
                # センサが揃わない間はニュートラル保持
                stop_hold(pi)

            # -----------------
            # ログ
            # -----------------
            now = time.time()
            if now - last_print >= PRINT_INTERVAL:
                last_print = now
                t_ms = int(now * 1000)

                def f(x, nd=7):
                    return "nan" if not math.isfinite(x) else f"{x:.{nd}f}"

                print(
                    f"{t_ms},"
                    f"{f(lat, 7)},"
                    f"{f(lon, 7)},"
                    f"{f(dist_m, 2)},"
                    f"{f(brng, 2)},"
                    f"{f(imu_corr, 2)},"
                    f"{f(err_deg, 2)},"
                    f"{f(fwd, 2)},"
                    f"{f(turn, 2)},"
                    f"{f(left, 2)},"
                    f"{f(right, 2)},"
                    f"{f(speed_ms, 2)},"
                    f"{f(course_deg, 2)},"
                    f"{sats},"
                    f"{f(hdop, 2)},"
                    f"{int(offset_locked)}"
                )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nCtrl+C -> stop")

    finally:
        stop_hold(pi)
        time.sleep(1)
        stop_all(pi)
        pi.stop()


if __name__ == "__main__":
    main()
