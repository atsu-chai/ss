#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import pigpio

# ===== GPIO config =====
ESC_RIGHT_GPIO = 19  # Right motor ESC signal (BCM)
ESC_LEFT_GPIO  = 18  # Left  motor ESC signal (BCM)

# ===== Pulse config (us) =====
PULSE_STOP     = 1480  # Neutral / stop
PULSE_MIN_FWD  = 1520  # Min forward (just above stop, starts to spin)
PULSE_MAX_FWD  = 2000  # Max forward

# Reverse: values BELOW stop.
# You can tune these later if needed.
PULSE_MIN_REV  = 1440  # Min reverse (just below stop, starts to spin backward)
PULSE_MAX_REV  = 1000  # Max reverse

ARM_PULSE      = 1000  # Used when arming ESCs at power-on


def motor_power_to_pulse(power: float) -> int:
    """
    Convert motor power in range [-1.0, 1.0] to pulse width (us).

    power > 0: forward (0.0..1.0 -> 1520..2000)
    power < 0: reverse (0.0..-1.0 -> 1440..1000)
    power = 0: stop (1480)
    """
    # Clamp to [-1.0, 1.0]
    power = max(-1.0, min(1.0, power))

    if power > 0.0:
        # Forward
        # Map 0.0 → PULSE_MIN_FWD, 1.0 → PULSE_MAX_FWD
        return int(PULSE_MIN_FWD + (PULSE_MAX_FWD - PULSE_MIN_FWD) * power)
    elif power < 0.0:
        # Reverse
        # Map 0.0 → PULSE_MIN_REV, -1.0 → PULSE_MAX_REV
        rev_power = -power  # 0.0..1.0
        return int(PULSE_MIN_REV - (PULSE_MIN_REV - PULSE_MAX_REV) * rev_power)
    else:
        # Stop
        return PULSE_STOP


def set_motor(pi: pigpio.pi, gpio: int, power: float) -> None:
    """
    Set one motor power in [-1.0, 1.0] on given GPIO.
    """
    pulse = motor_power_to_pulse(power)
    pi.set_servo_pulsewidth(gpio, pulse)


def mix_forward_turn(forward: float, turn: float) -> tuple[float, float]:
    """
    Simple differential drive mix.

    forward: -1.0 (full reverse) .. 0.0 .. 1.0 (full forward)
    turn   : -1.0 (full left)   .. 0.0 .. 1.0 (full right)

    Returns (left_power, right_power) in [-1.0, 1.0].
    """
    # Basic mixing
    left  = forward + turn
    right = forward - turn

    # Clamp both to [-1.0, 1.0]
    left  = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))

    return left, right


def stop_all(pi: pigpio.pi) -> None:
    """
    Stop both motors and turn off signals.
    """
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, 0)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, 0)


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpio. Run `sudo pigpiod` first.")
        return

    # Make sure everything is stopped
    stop_all(pi)
    time.sleep(1)

    print("=== Dual ESC diff-drive test ===")
    print("* Connect BOTH ESCs to LiPo (via splitter).")
    print("* Connect ESC signal wires:")
    print("  - Right ESC: GPIO19 (BCM)")
    print("  - Left  ESC: GPIO18 (BCM)")
    print("* Make sure ALL props / screws are REMOVED!")
    input("If everything is ready, press Enter...")

    # --- Arm ESCs: send low pulse, then wait with LiPo already connected ---
    print("\nArming ESCs with ARM_PULSE (1000us)...")
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, ARM_PULSE)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, ARM_PULSE)
    time.sleep(5)  # wait for beeps etc.

    # Set to STOP
    print("Set both motors to STOP.")
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, PULSE_STOP)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, PULSE_STOP)
    time.sleep(2)

    # --- Pattern test ---

    # 1) Forward slowly
    print("\n[TEST] Forward slow")
    fwd, turn = 0.3, 0.0
    left_power, right_power = mix_forward_turn(fwd, turn)
    print(f" left={left_power:.2f}, right={right_power:.2f}")
    set_motor(pi, ESC_LEFT_GPIO, left_power)
    set_motor(pi, ESC_RIGHT_GPIO, right_power)
    time.sleep(3)

    # 2) Forward faster
    print("\n[TEST] Forward fast")
    fwd, turn = 0.7, 0.0
    left_power, right_power = mix_forward_turn(fwd, turn)
    print(f" left={left_power:.2f}, right={right_power:.2f}")
    set_motor(pi, ESC_LEFT_GPIO, left_power)
    set_motor(pi, ESC_RIGHT_GPIO, right_power)
    time.sleep(3)

    # 3) Turn right in place (left forward, right reverse)
    print("\n[TEST] Turn right on spot")
    fwd, turn = 0.0, 0.6  # positive turn = right
    left_power, right_power = mix_forward_turn(fwd, turn)
    print(f" left={left_power:.2f}, right={right_power:.2f}")
    set_motor(pi, ESC_LEFT_GPIO, left_power)
    set_motor(pi, ESC_RIGHT_GPIO, right_power)
    time.sleep(3)

    # 4) Turn left in place
    print("\n[TEST] Turn left on spot")
    fwd, turn = 0.0, -0.6
    left_power, right_power = mix_forward_turn(fwd, turn)
    print(f" left={left_power:.2f}, right={right_power:.2f}")
    set_motor(pi, ESC_LEFT_GPIO, left_power)
    set_motor(pi, ESC_RIGHT_GPIO, right_power)
    time.sleep(3)

    # 5) Reverse straight
    print("\n[TEST] Reverse straight")
    fwd, turn = -0.4, 0.0
    left_power, right_power = mix_forward_turn(fwd, turn)
    print(f" left={left_power:.2f}, right={right_power:.2f}")
    set_motor(pi, ESC_LEFT_GPIO, left_power)
    set_motor(pi, ESC_RIGHT_GPIO, right_power)
    time.sleep(3)

    # Back to stop
    print("\nBack to STOP.")
    pi.set_servo_pulsewidth(ESC_LEFT_GPIO, PULSE_STOP)
    pi.set_servo_pulsewidth(ESC_RIGHT_GPIO, PULSE_STOP)
    time.sleep(2)

    print("Turn off signals and exit.")
    stop_all(pi)
    pi.stop()


if __name__ == "__main__":
    main()
