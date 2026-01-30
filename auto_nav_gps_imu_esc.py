#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AUTO NAV (GPS + BNO086) -> Dual ESC diff-drive (pigpio)

- GPS: NMEA over serial (GGA for position, RMC for speed/course)
- IMU: BNO086 over I2C (0x4B), "norst3" style robust init:
    * first boot: wait 4s
    * after BNO08X_I2C: wait 1s
    * enable_feature retry up to 30 times
    * on glitch: drop bno=None and re-init with backoff
- Navigation:
    * compute bearing to target
    * use IMU yaw (optionally offset aligned once by GPS course when moving)
    * generate forward/turn -> left/right power -> ESC pulses

Safety:
- If GPS/IMU not ready -> hold STOP (neutral)
- Ctrl+C -> STOP then signals off
"""

import time
import math
import serial
import pigpio
import pynmea2

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR


# =========================================================
# ====================== USER CONFIG =======================
# =========================================================

# ----- GPS -----
GPS_PORT = "/dev/ttyS0"      # your current working config (or "/dev/serial0")
GPS_BAUDRATE = 115200
GPS_TIMEOUT = 1.0

# Accept GN/GP talker IDs
GPS_TALKERS = ("$GN", "$GP")

# ----- IMU (BNO086) -----
IMU_ADDR = 0x4B
IMU_REPORT_US = 200_000  # 200ms

# "norst3 style" init tuning
IMU_BOOT_SLEEP_FIRST = 4.0      # first init only
IMU_BOOT_SLEEP_REINIT = 0.5     # after glitches
IMU_AFTER_INIT_SLEEP = 1.0
IMU_ENABLE_RETRY = 30
IMU_ENABLE_RETRY_SLEEP = 0.4

# glitch backoff
GLITCH_BACKOFF_MIN = 2.0
GLITCH_BACKOFF_MAX = 30.0
GLITCH_BACKOFF_GROW = 2.0

# ----- Target -----
TARGET_LAT = 35.681236
TARGET_LON = 139.767125

# ----- Navigation -----
STOP_RADIUS_M = 1.5

SPEED_FOR_COURSE_MS = 1.5   # GPS speed threshold to trust course for yaw offset lock
LOCK_OFFSET_ONCE = True     # lock IMU offset once using GPS course when moving

KP_YAW = 2.0
KP_V = 0.35
MAX_W = 1.2
MAX_V = 1.0

SLOW_ANGLE_DEG = 35.0
SPIN_IN_PLACE_DEG = 60.0

# ----- ESC / pigpio -----
ESC_RIGHT_GPIO = 19
ESC_LEFT_GPIO  = 18

PULSE_STOP     = 1480
PULSE_MIN_FWD  = 1520
PULSE_MAX_FWD  = 2000
PULSE_MIN_REV  = 1440
PULSE_MAX_REV  = 1000
ARM_PULSE      = 1000

POWER_SCALE_FWD  = 0.7
POWER_SCALE_TURN = 0.8
POWER_DEADBAND   = 0.03

# ----- Logging -----
PRINT_INTERVAL = 0.2

# ----- IMU yaw direction quick fix -----
# If turning is reversed, set True (or flip sign of err_deg below)
IMU_REVERSE = False


# =========================================================
# ====================== UTILITIES =========================
# =========================================================

EARTH_R = 6371000.0

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
    return wrap360(math.degrees(math.atan2(y, x)))  # North=0, East=+

def quat_to_yaw_deg(q):
    # bno.quaternion usually (x,y,z,w)
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return wrap360(math.degrees(yaw))


# =========================================================
# ====================== IMU ROBUST INIT ===================
# =========================================================

def init_bno08x_norst3(i2c, is_first_init: bool):
    """
    Robust init:
    - sleep (4s first init, 0.5s reinit)
    - create BNO08X_I2C
    - sleep 1s
    - enable_feature retry
    """
    time.sleep(IMU_BOOT_SLEEP_FIRST if is_first_init else IMU_BOOT_SLEEP_REINIT)

    # NOTE: debug=False requested, but some builds may still print packets.
    bno = BNO08X_I2C(i2c, address=IMU_ADDR, debug=False)

    time.sleep(IMU_AFTER_INIT_SLEEP)

    last = None
    for j in range(IMU_ENABLE_RETRY):
        try:
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=IMU_REPORT_US)
            print(f"IMU init OK at 0x{IMU_ADDR:02X}")
            return bno
        except Exception as e:
            last = e
            print(f"IMU enable retry {j+1}/{IMU_ENABLE_RETRY}: {repr(e)}")
            time.sleep(IMU_ENABLE_RETRY_SLEEP)

    raise RuntimeError(f"enable_feature failed: {repr(last)}")


# =========================================================
# ====================== ESC CONTROL ========================
# =========================================================

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


# =========================================================
# =========================== MAIN ==========================
# =========================================================

def main():
    # pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpio. Run `sudo pigpiod` first.")
        return

    # GPS serial
    ser = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=GPS_TIMEOUT)

    # IMU bus
    i2c = busio.I2C(board.SCL, board.SDA)

    # Safety stop
    stop_all(pi)
    time.sleep(1)

    print("=== AUTO NAV (GPS + BNO086 + Dual ESC diff-drive) ===")
    print(f"GPS: {GPS_PORT} @ {GPS_BAUDRATE}")
    print(f"IMU: I2C addr 0x{IMU_ADDR:02X} (scan should show it)")
    print(f"TARGET: lat={TARGET_LAT}, lon={TARGET_LON}")
    print("* Safety: run with wheels lifted first!")
    input("ReadyならEnter...")

    arm_escs(pi)

    # GPS state
    lat = lon = float("nan")
    gps_valid = False
    sats = 0
    hdop = float("nan")
    alt = float("nan")

    speed_ms = float("nan")
    course_deg = float("nan")

    # IMU state (robust)
    bno = None
    is_first_imu_init = True
    next_imu_try = 0.0
    backoff = GLITCH_BACKOFF_MIN

    imu_heading = float("nan")
    heading_offset = 0.0
    offset_locked = False

    slow_angle_rad = math.radians(SLOW_ANGLE_DEG)

    last_print = 0.0

    print("time_ms,lat,lon,dist_m,bearing_deg,imu_corr_deg,err_deg,fwd,turn,left,right,spd_ms,course_deg,sats,hdop,imu_ok,offset_locked")

    try:
        while True:
            now_t = time.time()

            # -----------------
            # GPS parse (GGA + RMC)
            # -----------------
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line and line.startswith(GPS_TALKERS):
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.nmea.ChecksumError:
                        msg = None

                    if msg is not None:
                        # GGA: position + sats + hdop + altitude
                        if msg.sentence_type == "GGA":
                            if msg.latitude and msg.longitude:
                                lat, lon = msg.latitude, msg.longitude
                                gps_valid = True
                            try:
                                sats = int(getattr(msg, "num_sats", 0) or 0)
                            except Exception:
                                sats = sats
                            try:
                                hdop = float(getattr(msg, "horizontal_dil", float("nan")) or float("nan"))
                            except Exception:
                                hdop = hdop
                            try:
                                alt = float(getattr(msg, "altitude", float("nan")) or float("nan"))
                            except Exception:
                                alt = alt

                        # RMC: speed + course + status
                        elif msg.sentence_type == "RMC":
                            if getattr(msg, "status", "") == "A":
                                gps_valid = True
                                try:
                                    if msg.spd_over_grnd:
                                        speed_ms = float(msg.spd_over_grnd) * 0.514444
                                except Exception:
                                    pass
                                try:
                                    if msg.true_course:
                                        course_deg = float(msg.true_course)
                                except Exception:
                                    pass
                            else:
                                gps_valid = False
            except Exception:
                pass

            # -----------------
            # IMU init (with backoff)
            # -----------------
            if bno is None and now_t >= next_imu_try:
                try:
                    print("init IMU...")
                    bno = init_bno08x_norst3(i2c, is_first_imu_init)
                    is_first_imu_init = False
                    backoff = GLITCH_BACKOFF_MIN
                except Exception as e:
                    print("IMU init failed:", repr(e))
                    next_imu_try = now_t + backoff
                    backoff = min(GLITCH_BACKOFF_MAX, backoff * GLITCH_BACKOFF_GROW)

            # -----------------
            # IMU read (glitch -> drop + reinit later)
            # -----------------
            imu_ok = False
            imu_heading = float("nan")

            if bno is not None:
                try:
                    q = bno.quaternion
                    if q is not None:
                        imu_heading = quat_to_yaw_deg(q)
                        if IMU_REVERSE:
                            imu_heading = wrap360(360.0 - imu_heading)
                        imu_ok = True
                except Exception as e:
                    print("IMU glitch:", type(e).__name__, repr(e))
                    bno = None
                    imu_ok = False
                    next_imu_try = time.time() + backoff
                    backoff = min(GLITCH_BACKOFF_MAX, max(GLITCH_BACKOFF_MIN, backoff * GLITCH_BACKOFF_GROW))

            # -----------------
            # Optional: lock yaw offset once using GPS course (when moving)
            # -----------------
            if (LOCK_OFFSET_ONCE
                and (not offset_locked)
                and imu_ok
                and gps_valid
                and math.isfinite(course_deg)
                and math.isfinite(speed_ms)
                and speed_ms > SPEED_FOR_COURSE_MS):
                # offset in degrees: course - imu_heading
                heading_offset = wrap180(course_deg - imu_heading)
                offset_locked = True

            imu_corr = float("nan")
            if imu_ok:
                imu_corr = wrap360(imu_heading + heading_offset)

            # -----------------
            # NAV -> fwd/turn -> motors
            # -----------------
            fwd = 0.0
            turn = 0.0
            left = right = 0.0

            dist_m = float("nan")
            brng = float("nan")
            err_deg = float("nan")

            ready = gps_valid and imu_ok and math.isfinite(lat) and math.isfinite(lon) and math.isfinite(imu_corr)

            if ready:
                dist_m = haversine_m(lat, lon, TARGET_LAT, TARGET_LON)

                if dist_m <= STOP_RADIUS_M:
                    fwd, turn = 0.0, 0.0
                else:
                    brng = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)

                    # +err_deg means "need to turn right"
                    err_deg = wrap180(brng - imu_corr)

                    err_rad = math.radians(err_deg)

                    # yaw control (w)
                    w = clamp(KP_YAW * err_rad, -MAX_W, MAX_W)

                    # forward control (v), slow down if heading error large
                    v_base = clamp(KP_V * dist_m, 0.0, MAX_V)

                    if abs(err_deg) >= SPIN_IN_PLACE_DEG:
                        v = 0.0
                    else:
                        ang = abs(err_rad)
                        if ang >= slow_angle_rad:
                            v = 0.15 * v_base
                        else:
                            scale = 1.0 - 0.85 * (ang / slow_angle_rad)
                            v = v_base * clamp(scale, 0.15, 1.0)

                    # normalize to [-1..1] for ESC mixing
                    fwd  = clamp((v / MAX_V) * POWER_SCALE_FWD,  -1.0, 1.0)
                    turn = clamp((w / MAX_W) * POWER_SCALE_TURN, -1.0, 1.0)

                left, right = mix_forward_turn(fwd, turn)
                set_motor(pi, ESC_LEFT_GPIO, left)
                set_motor(pi, ESC_RIGHT_GPIO, right)

            else:
                # Not ready -> neutral hold
                stop_hold(pi)

            # -----------------
            # LOG
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
                    f"{1 if imu_ok else 0},"
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
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
