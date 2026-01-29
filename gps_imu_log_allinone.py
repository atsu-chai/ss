#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_imu_log_allinone.py

- BNO086(BNO08X) I2C(0x4B) Rotation Vector(quaternion) を安定初期化（待つ＋粘る）
- 必要ならRSTピンでハードリセットも実行（任意）
- adafruit_bno08x が吐く DBG:: Packet... を確実に黙らせる（ライブラリ呼び出し時だけ stdout/stderr を捨てる）
- GPSは gpsd があれば gpsd、なければ /dev/serial0 等の NMEA をざっくり解析（どちらも無ければGPS=NOFIX固定）
- CSVへログ：timestamp, gps_fix, lat, lon, speed, track, alt, sats, imu_ok, qx,qy,qz,qw, yaw,pitch,roll, err

使い方例:
  python gps_imu_log_allinone.py
  python gps_imu_log_allinone.py --csv /home/atsu/data/gps_imu_log.csv
  python gps_imu_log_allinone.py --gps-mode serial --gps-dev /dev/ttyAMA0 --gps-baud 9600
  python gps_imu_log_allinone.py --use-rst --rst-pin 17 --rst-active-low
  python gps_imu_log_allinone.py --no-quiet-lib   # DBGパケットを見たいとき

注意:
- RSTピン配線してないなら --use-rst を付けない（デフォルトはRST無効）
- I2C: Raspberry Pi の I2C有効化＆配線確認
"""

import os
import sys
import csv
import time
import math
import argparse
import contextlib
from dataclasses import dataclass
from datetime import datetime, timezone

# ---- IMU / I2C ----
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# ---------------------------
# Utils
# ---------------------------

def iso_now():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")

@contextlib.contextmanager
def suppress_output(enabled: bool = True):
    """ライブラリ内部print(DBG:: Packetなど)を黙らせる"""
    if not enabled:
        yield
        return
    with open(os.devnull, "w") as devnull:
        with contextlib.redirect_stdout(devnull), contextlib.redirect_stderr(devnull):
            yield

def quat_to_euler_deg(qx, qy, qz, qw):
    """
    quaternion (x,y,z,w) -> yaw/pitch/roll [deg]
    yaw: Z, pitch: Y, roll: X (一般的な航空系Tait-Bryan)
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

# ---------------------------
# RST (optional)
# ---------------------------

class RstController:
    """
    Raspberry Pi BCM GPIOでRST制御（任意）
    - RPi.GPIO があればそれを使用
    - 無ければ gpiozero があれば使用
    - どちらも無ければ「RST無効」にフォールバック
    """
    def __init__(self, enabled: bool, pin_bcm: int, active_low: bool):
        self.enabled = enabled
        self.pin = pin_bcm
        self.active_low = active_low
        self.backend = None
        self._gpio = None
        self._dev = None

        if not self.enabled:
            return

        # Try RPi.GPIO
        try:
            import RPi.GPIO as GPIO
            self.backend = "RPi.GPIO"
            self._gpio = GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)
            # deassert
            self._write(False)
            return
        except Exception:
            self._gpio = None

        # Try gpiozero
        try:
            from gpiozero import DigitalOutputDevice
            self.backend = "gpiozero"
            self._dev = DigitalOutputDevice(self.pin, active_high=(not self.active_low), initial_value=False)
            return
        except Exception:
            self._dev = None

        # fallback disable
        self.enabled = False

    def _write(self, asserted: bool):
        """
        asserted=True でリセット状態にする
        """
        if not self.enabled:
            return
        if self.backend == "RPi.GPIO":
            level = 0
            # active_lowなら asserted=True -> LOW
            if self.active_low:
                level = 0 if asserted else 1
            else:
                level = 1 if asserted else 0
            self._gpio.output(self.pin, level)
        elif self.backend == "gpiozero":
            # gpiozeroは active_high 設定済みなので on/offでOK
            if asserted:
                self._dev.on()
            else:
                self._dev.off()

    def pulse(self, assert_s: float = 0.05, settle_s: float = 0.50):
        """
        RSTを短く叩く：assert -> deassert -> settle
        """
        if not self.enabled:
            return
        self._write(True)
        time.sleep(assert_s)
        self._write(False)
        time.sleep(settle_s)

    def close(self):
        if self.backend == "RPi.GPIO" and self._gpio:
            try:
                self._gpio.cleanup(self.pin)
            except Exception:
                pass
        if self.backend == "gpiozero" and self._dev:
            try:
                self._dev.close()
            except Exception:
                pass

# ---------------------------
# GPS (gpsd or NMEA serial)
# ---------------------------

@dataclass
class GpsData:
    fix: bool = False
    lat: float | None = None
    lon: float | None = None
    speed_mps: float | None = None
    track_deg: float | None = None
    alt_m: float | None = None
    sats: int | None = None

class GpsReader:
    def __init__(self, mode: str, dev: str, baud: int):
        self.mode = mode
        self.dev = dev
        self.baud = baud

        self._gpsd = None
        self._serial = None
        self._buf = b""

        if self.mode == "gpsd":
            self._init_gpsd()
        elif self.mode == "serial":
            self._init_serial()

    def _init_gpsd(self):
        try:
            import gpsd
            gpsd.connect()  # localhost:2947
            self._gpsd = gpsd
        except Exception:
            self._gpsd = None

    def _init_serial(self):
        try:
            import serial
            self._serial = serial.Serial(self.dev, self.baud, timeout=0.1)
        except Exception:
            self._serial = None

    def read(self) -> GpsData:
        if self.mode == "gpsd":
            return self._read_gpsd()
        if self.mode == "serial":
            return self._read_serial_nmea()
        return GpsData(fix=False)

    def _read_gpsd(self) -> GpsData:
        if not self._gpsd:
            return GpsData(fix=False)
        try:
            packet = self._gpsd.get_current()
            # gpsd-py3: packet.mode >= 2 で2D fix, >=3 で3D fix
            fix_ok = getattr(packet, "mode", 0) >= 2
            lat = getattr(packet, "lat", None)
            lon = getattr(packet, "lon", None)
            alt = getattr(packet, "alt", None)
            # speed: m/s
            speed = getattr(packet, "hspeed", None) or getattr(packet, "speed", None)
            track = getattr(packet, "track", None)
            sats = getattr(packet, "sats", None)
            return GpsData(
                fix=bool(fix_ok),
                lat=lat, lon=lon, alt_m=alt,
                speed_mps=speed, track_deg=track,
                sats=sats if isinstance(sats, int) else None
            )
        except Exception:
            return GpsData(fix=False)

    # --- very small NMEA parser (GGA + RMC) ---
    @staticmethod
    def _nmea_degmin_to_deg(dm: str, hemi: str) -> float | None:
        # dm: ddmm.mmmm or dddmm.mmmm
        if not dm or "." not in dm:
            return None
        try:
            if len(dm) < 4:
                return None
            dot = dm.index(".")
            head = dm[:dot]
            if len(head) <= 2:
                return None
            deg_len = 2 if len(head) in (4, 5) else 3  # heuristic
            # Better: lat uses 2 deg digits, lon uses 3. We'll infer from length.
            # If head length is 4 => ddmm ; if 5 => dddmm
            deg_len = 2 if len(head) == 4 else 3
            deg = float(dm[:deg_len])
            minutes = float(dm[deg_len:])
            val = deg + minutes / 60.0
            if hemi in ("S", "W"):
                val = -val
            return val
        except Exception:
            return None

    def _read_serial_nmea(self) -> GpsData:
        if not self._serial:
            return GpsData(fix=False)

        # accumulate some bytes
        try:
            chunk = self._serial.read(512)
            if chunk:
                self._buf += chunk
        except Exception:
            return GpsData(fix=False)

        # parse lines
        lines = self._buf.split(b"\n")
        if len(lines) <= 1:
            return GpsData(fix=False)
        self._buf = lines[-1]  # keep remainder

        gga = None
        rmc = None

        for raw in lines[:-1]:
            try:
                line = raw.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not line.startswith("$"):
                continue
            if "GGA" in line:
                gga = line
            elif "RMC" in line:
                rmc = line

        data = GpsData(fix=False)

        # GGA: fix + lat/lon + alt + sats
        if gga:
            parts = gga.split(",")
            # 0:$xxGGA  2:lat 3:N/S 4:lon 5:E/W 6:fixq 7:sats 9:alt
            if len(parts) > 9:
                fixq = parts[6]
                sats = parts[7]
                lat = self._nmea_degmin_to_deg(parts[2], parts[3])
                lon = self._nmea_degmin_to_deg(parts[4], parts[5])
                alt = None
                try:
                    alt = float(parts[9]) if parts[9] else None
                except Exception:
                    alt = None
                try:
                    sats_i = int(sats) if sats else None
                except Exception:
                    sats_i = None
                data.lat = lat
                data.lon = lon
                data.alt_m = alt
                data.sats = sats_i
                data.fix = (fixq and fixq != "0")

        # RMC: speed(knots) + course
        if rmc:
            parts = rmc.split(",")
            # 0:$xxRMC 2:status(A/V) 7:speed(knots) 8:course
            if len(parts) > 8:
                status = parts[2]
                if status == "A":
                    data.fix = True
                try:
                    sp_kn = float(parts[7]) if parts[7] else None
                    data.speed_mps = sp_kn * 0.514444 if sp_kn is not None else None
                except Exception:
                    pass
                try:
                    data.track_deg = float(parts[8]) if parts[8] else None
                except Exception:
                    pass

        return data

    def close(self):
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass

# ---------------------------
# IMU Manager
# ---------------------------

class ImuManager:
    def __init__(
        self,
        i2c,
        imu_addr: int,
        report_us: int,
        quiet_lib: bool,
        use_rst: bool,
        rst: RstController | None,
        powerup_wait_s: float = 4.0,
        post_ctor_wait_s: float = 1.0,
        enable_retries: int = 30,
        init_retries: int = 40,
    ):
        self.i2c = i2c
        self.addr = imu_addr
        self.report_us = report_us
        self.quiet_lib = quiet_lib
        self.use_rst = use_rst
        self.rst = rst

        self.powerup_wait_s = powerup_wait_s
        self.post_ctor_wait_s = post_ctor_wait_s
        self.enable_retries = enable_retries
        self.init_retries = init_retries

        self.bno = None

        # backoff
        self.backoff_s = 2.0
        self.backoff_max_s = 30.0
        self.next_try_ts = 0.0
        self.last_err = ""

    def _now(self) -> float:
        return time.time()

    def mark_failed(self, err: str):
        self.last_err = err
        self.bno = None
        self.next_try_ts = self._now() + self.backoff_s
        self.backoff_s = min(self.backoff_s * 2.0, self.backoff_max_s)

    def mark_ok(self):
        self.last_err = ""
        self.backoff_s = 2.0
        self.next_try_ts = 0.0

    def maybe_init(self):
        if self.bno is not None:
            return True

        now = self._now()
        if now < self.next_try_ts:
            # not yet
            return False

        last = None

        # optional RST pulse
        if self.use_rst and self.rst and self.rst.enabled:
            # RST叩いてから少し待つ
            try:
                self.rst.pulse(assert_s=0.05, settle_s=0.60)
            except Exception:
                pass

        # powerup wait (重要)
        time.sleep(self.powerup_wait_s)

        for k in range(self.init_retries):
            try:
                with suppress_output(self.quiet_lib):
                    bno = BNO08X_I2C(self.i2c, address=self.addr, debug=False)

                time.sleep(self.post_ctor_wait_s)

                # enable retry
                for j in range(self.enable_retries):
                    try:
                        with suppress_output(self.quiet_lib):
                            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval=self.report_us)

                        # すぐ読めるか確認（ここで落ちることがある）
                        with suppress_output(self.quiet_lib):
                            qx, qy, qz, qw = bno.quaternion
                        _ = quat_to_euler_deg(qx, qy, qz, qw)

                        self.bno = bno
                        self.mark_ok()
                        print(f"{iso_now()} IMU init OK at 0x{self.addr:02X}")
                        return True

                    except Exception as e:
                        last = e
                        # ここで Unprocessable Batch bytes が出ても、とにかく粘る
                        print(f"{iso_now()} IMU enable retry {j+1}/{self.enable_retries}: {e}")
                        time.sleep(0.4)

                raise RuntimeError(f"enable_feature failed: {last}")

            except Exception as e:
                last = e
                print(f"{iso_now()} IMU init retry {k+1}/{self.init_retries}: {e}")
                time.sleep(0.6)

        self.mark_failed(f"init:{last}")
        return False

    def read_quat(self):
        """
        return: (ok, qx,qy,qz,qw, yaw,pitch,roll, err)
        """
        if self.bno is None:
            return (False, None, None, None, None, None, None, None, self.last_err or "IMU not ready")

        try:
            with suppress_output(self.quiet_lib):
                qx, qy, qz, qw = self.bno.quaternion
            yaw, pitch, roll = quat_to_euler_deg(qx, qy, qz, qw)
            return (True, qx, qy, qz, qw, yaw, pitch, roll, "")
        except Exception as e:
            # glitch -> re-init later
            self.mark_failed(f"glitch:{e}")
            return (False, None, None, None, None, None, None, None, self.last_err)

# ---------------------------
# Main
# ---------------------------

def ensure_csv_header(path: str):
    header = [
        "timestamp",
        "gps_fix", "lat", "lon", "speed_mps", "track_deg", "alt_m", "sats",
        "imu_ok", "qx", "qy", "qz", "qw", "yaw_deg", "pitch_deg", "roll_deg",
        "err",
    ]
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return header

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
    return header

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="/home/atsu/data/gps_imu_log.csv")
    parser.add_argument("--imu-addr", default="0x4B")
    parser.add_argument("--report-ms", type=int, default=200)  # 200ms
    parser.add_argument("--loop-ms", type=int, default=200)    # print/log interval
    parser.add_argument("--quiet-lib", action="store_true", default=True)
    parser.add_argument("--no-quiet-lib", action="store_false", dest="quiet_lib")

    # RST
    parser.add_argument("--use-rst", action="store_true", default=False)
    parser.add_argument("--rst-pin", type=int, default=17)  # BCM
    parser.add_argument("--rst-active-low", action="store_true", default=True)
    parser.add_argument("--rst-active-high", action="store_false", dest="rst_active_low")

    # GPS
    parser.add_argument("--gps-mode", choices=["auto", "gpsd", "serial", "none"], default="auto")
    parser.add_argument("--gps-dev", default="/dev/serial0")
    parser.add_argument("--gps-baud", type=int, default=9600)

    args = parser.parse_args()

    # --- encoding: 絵文字を使わないので基本不要だが、念のためUTF-8推奨 ---
    # （環境側で LANG=C.UTF-8 などにしておくと安心）
    # ここでは「文字化けしやすい絵文字」を使わない方針で回避。

    imu_addr = int(args.imu_addr, 16) if isinstance(args.imu_addr, str) else int(args.imu_addr)
    report_us = int(args.report_ms * 1000)
    loop_s = max(0.05, args.loop_ms / 1000.0)

    print(f"Logging to: {args.csv}")
    print("Running... Ctrl+C to stop")

    ensure_csv_header(args.csv)

    # GPS init
    gps_mode = args.gps_mode
    gps = None
    if gps_mode == "auto":
        # try gpsd, else serial
        g1 = GpsReader("gpsd", args.gps_dev, args.gps_baud)
        if g1._gpsd:
            gps = g1
            gps_mode = "gpsd"
        else:
            g1.close()
            g2 = GpsReader("serial", args.gps_dev, args.gps_baud)
            if g2._serial:
                gps = g2
                gps_mode = "serial"
            else:
                g2.close()
                gps = GpsReader("none", args.gps_dev, args.gps_baud)
                gps_mode = "none"
    else:
        gps = GpsReader(gps_mode, args.gps_dev, args.gps_baud)

    print(f"GPS mode: {gps_mode}")

    # RST init
    rst = RstController(enabled=args.use_rst, pin_bcm=args.rst_pin, active_low=args.rst_active_low)
    if args.use_rst:
        if rst.enabled:
            print(f"RST enabled: pin=BCM{args.rst_pin} active_low={args.rst_active_low} backend={rst.backend}")
        else:
            print("RST requested but GPIO backend not available -> RST disabled")

    # I2C init
    i2c = busio.I2C(board.SCL, board.SDA)

    imu = ImuManager(
        i2c=i2c,
        imu_addr=imu_addr,
        report_us=report_us,
        quiet_lib=args.quiet_lib,
        use_rst=args.use_rst,
        rst=rst if rst.enabled else None,
        powerup_wait_s=4.0,     # ここが超重要（あなたのnorst3.pyと同じ思想）
        post_ctor_wait_s=1.0,
        enable_retries=30,
        init_retries=40,
    )

    # main loop
    try:
        while True:
            ts = iso_now()

            # GPS
            gd = gps.read()
            gps_fix = "FIX" if gd.fix else "NOFIX"

            # IMU
            if imu.bno is None:
                print("init IMU...")
                imu.maybe_init()

            imu_ok, qx, qy, qz, qw, yaw, pitch, roll, err = imu.read_quat()
            imu_state = "ok" if imu_ok else "down"

            # status line
            line = f"{ts} GPS={gps_fix} IMU={imu_state}"
            if not imu_ok and imu.last_err:
                # backoff info
                remain = max(0.0, imu.next_try_ts - time.time())
                line += f" err={imu.last_err} next_try={remain:.1f}s backoff={imu.backoff_s:.1f}s"
            print(line)

            # write csv
            with open(args.csv, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    ts,
                    1 if gd.fix else 0,
                    gd.lat, gd.lon, gd.speed_mps, gd.track_deg, gd.alt_m, gd.sats,
                    1 if imu_ok else 0,
                    qx, qy, qz, qw,
                    yaw, pitch, roll,
                    err or imu.last_err,
                ])

            time.sleep(loop_s)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            gps.close()
        except Exception:
            pass
        try:
            rst.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
