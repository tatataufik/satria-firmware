#!/usr/bin/env python3
"""
test_joystick.py — Joystick → MAVLink RC_CHANNELS_OVERRIDE with live per-channel
bar visualization.

All joystick axes/buttons are mapped to raw PWM channels (1000-2000 µs) and
sent as RC_CHANNELS_OVERRIDE at the specified rate.  No special CH6 handling —
every channel is passed through as-is.

Channel mapping (standard RC layout):
  Axis 0            → CH1  Roll / aileron      (centred)
  Axis 1            → CH2  Pitch / elevator    (centred, inverted)
  Axis 2            → CH3  Throttle            (full-range)
  Axis 3            → CH4  Yaw / rudder        (centred)
  Button 6 / 7      → CH5  3-pos: btn7=1000 (MANUAL), btn6=1500 (STAB), neither=2000 (MISSION)
  Button 4 / 5      → CH6  3-pos: btn5=1000 (low),    btn4=1500 (mid),  neither=2000 (high)
  Button 0          → CH7  (2000 pressed, 1000 released)
  Button 1          → CH8  (2000 pressed, 1000 released)

Usage:
    python3 test_joystick.py
    python3 test_joystick.py --connect udpout:192.168.1.1:14550
    python3 test_joystick.py --connect /dev/ttyUSB0 --baud 115200
    python3 test_joystick.py --connect udp:127.0.0.1:14550 --joy 1 --rate 100
    python3 test_joystick.py --no-mavlink   # visualization only, no MAVLink
"""

import argparse
import signal
import sys
import time

try:
    import pygame
except ImportError:
    print("pygame not installed — run: pip3 install pygame")
    sys.exit(1)

# MAVLink is optional (--no-mavlink skips it)
try:
    from pymavlink import mavutil
    _HAVE_MAV = True
except ImportError:
    _HAVE_MAV = False

# ── Constants ────────────────────────────────────────────────────────────────

BAR_WIDTH  = 28             # characters in the bar graph

# Gesture intent thresholds — tuned to actual joystick output (PX4 uses stricter values internally).
# When ALL conditions are met the script snaps RC values to exact gesture positions so PX4
# receives clean input (1500/1500/1000/2000/1000) and its own gesture detection succeeds.
GESTURE_THR   = -0.74   # throttle must be below this  (PX4 uses -0.80)
GESTURE_YAW   =  0.74   # yaw must be above this (ARM) (PX4 uses +0.90)
GESTURE_ROLL  =  0.85   # |roll|  must be below this   (PX4 uses 0.10)
GESTURE_PITCH =  0.85   # |pitch| must be below this   (PX4 uses 0.10)

CH_LABELS = [
    "CH1  Roll     ",
    "CH2  Pitch    ",
    "CH3  Throttle ",
    "CH4  Yaw      ",
    "CH5  Aux1     ",
    "CH6  Aux2     ",
    "CH7  Aux3     ",
    "CH8  Aux4     ",
]

_SV_LABELS = {
    "plane": [
        "SV1  Aileron  ",
        "SV2  Elevator ",
        "SV3  Throttle ",
        "SV4  Rudder   ",
        "SV5  Aux1     ",
        "SV6  Aux2     ",
        "SV7  Aux3     ",
        "SV8  Aux4     ",
    ],
    "vtail": [
        "SV1  Aileron  ",
        "SV2  R-Rudvtr ",
        "SV3  Throttle ",
        "SV4  L-Rudvtr ",
        "SV5  Aux1     ",
        "SV6  Aux2     ",
        "SV7  Aux3     ",
        "SV8  Aux4     ",
    ],
    "elevon": [
        "SV1  L-Elevon ",
        "SV2  R-Elevon ",
        "SV3  Throttle ",
        "SV4  Rudder   ",
        "SV5  Aux1     ",
        "SV6  Aux2     ",
        "SV7  Aux3     ",
        "SV8  Aux4     ",
    ],
}

# ── PWM helpers ──────────────────────────────────────────────────────────────

def _axis_pwm(v: float, invert: bool = False) -> int:
    """Map a centred axis [-1, +1] to PWM [1000, 2000]."""
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))


def _thr_pwm(v: float) -> int:
    """Map a throttle axis [-1, +1] (full-range) to PWM [1000, 2000]."""
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))


def _btn_pwm(joy: "pygame.joystick.Joystick", *indices) -> int:
    """2000 if any of the given buttons is pressed, else 1000."""
    n = joy.get_numbuttons()
    return 2000 if any(i < n and joy.get_button(i) for i in indices) else 1000


def _normalize(pwm: int, min_us: int = 1000, trim_us: int = 1500, max_us: int = 2000) -> float:
    """Convert PWM µs → normalized [-1.0, +1.0] exactly as PX4 rc_update does."""
    if pwm <= trim_us:
        return max(-1.0, (pwm - trim_us) / float(trim_us - min_us))
    return min(+1.0, (pwm - trim_us) / float(max_us - trim_us))


# ── Channel reader ───────────────────────────────────────────────────────────

def read_channels(joy: "pygame.joystick.Joystick") -> list[int]:
    """Return a list of 8 PWM values for CH1-CH8."""
    n_ax  = joy.get_numaxes()
    n_btn = joy.get_numbuttons()

    def ax(i, inv=False):
        return _axis_pwm(joy.get_axis(i), inv) if i < n_ax else 1500

    ch1 = ax(0)                           # roll
    ch2 = ax(1, inv=True)                 # pitch (inverted: push-forward = nose-down)
    ch3 = _thr_pwm(joy.get_axis(2)) if n_ax > 2 else 1000   # throttle
    ch4 = ax(3)                           # yaw

    # CH5: 3-position switch — btn7=1000 (MANUAL), btn6=1500 (STAB), neither=2000 (MISSION).
    # Hold btn7 to enter MANUAL mode for gesture arming.
    btn6 = 6 < n_btn and joy.get_button(6)
    btn7 = 7 < n_btn and joy.get_button(7)
    ch5 = 1000 if btn7 else (1500 if btn6 else 2000)

    # CH6: 3-position switch — btn5=1000 (low), btn4=1500 (mid), neither=2000 (high).
    btn4 = 4 < n_btn and joy.get_button(4)
    btn5 = 5 < n_btn and joy.get_button(5)
    ch6  = 1000 if btn5 else (1500 if btn4 else 2000)

    ch7 = _btn_pwm(joy, 0)                # button A / cross
    ch8 = _btn_pwm(joy, 1)                # button B / circle

    return [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]


# ── MAVLink send ─────────────────────────────────────────────────────────────

def send_override(master, channels: list[int]):
    """Send RC_CHANNELS_OVERRIDE for CH1-CH8; remaining channels → 0 (no override, MAVLink standard)."""
    # 0 = "do not override this channel" per MAVLink spec (UINT16_MAX is ArduPilot-only)
    padded = channels[:8] + [0] * 10
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *padded,
    )


def release_override(master):
    """Set all channels to 0 (release override)."""
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *([0] * 18),
    )


# ── Visualization ────────────────────────────────────────────────────────────

def _bar(pwm: int) -> str:
    """Render a PWM value as a filled bar centred at 1500."""
    frac   = (pwm - 1000) / 1000.0          # 0.0 … 1.0
    filled = max(0, min(BAR_WIDTH, int(frac * BAR_WIDTH)))
    centre = BAR_WIDTH // 2
    bar    = [" "] * BAR_WIDTH
    for i in range(filled):
        bar[i] = "█"
    bar[centre] = "│"                        # centre marker
    return "".join(bar)


def render(channels: list[int], joy_name: str, connect: str,
           n_ax: int, n_btn: int, n_sent: int, servo_raw: list[int],
           sv_labels: list[str], airframe: str):
    lines = ["\033[H\033[J"]                 # cursor home + clear screen
    lines.append(f" Joystick : {joy_name}  (axes={n_ax}  buttons={n_btn})")
    lines.append(f" MAVLink  : {connect}   packets sent={n_sent}")
    lines.append(f" Airframe : {airframe.upper()}")
    lines.append(f" {'─'*56}")
    lines.append(f"  {'Channel':<18}  1000{'':>{BAR_WIDTH//2 - 2}}1500{'':>{BAR_WIDTH//2 - 2}}2000   µs")
    lines.append(f"  {'─'*56}")
    for i, pwm in enumerate(channels):
        label = CH_LABELS[i] if i < len(CH_LABELS) else f"CH{i+1:<5}       "
        bar   = _bar(pwm)
        lines.append(f"  {label}  [{bar}]  {pwm:4d}")
    lines.append(f"  {'─'*56}")
    lines.append(f"  {'Servo out':<18}  1000{'':>{BAR_WIDTH//2 - 2}}1500{'':>{BAR_WIDTH//2 - 2}}2000   µs")
    lines.append(f"  {'─'*56}")
    for i, pwm in enumerate(servo_raw):
        label = sv_labels[i] if i < len(sv_labels) else f"SV{i+1:<5}       "
        bar   = _bar(pwm) if pwm else " " * BAR_WIDTH
        lines.append(f"  {label}  [{bar}]  {pwm:4d}")
    lines.append(f"  {'─'*56}")
    lines.append("  Ctrl-C to quit")
    print("\n".join(lines), end="", flush=True)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Joystick → MAVLink RC_CHANNELS_OVERRIDE with live visualization"
    )
    parser.add_argument(
        "--connect", default="udpout:127.0.0.1:14550",
        help="MAVLink connection string (default: udpout:127.0.0.1:14550)"
    )
    parser.add_argument("--baud",       type=int,   default=57600,
                        help="Baud rate for serial connections (default: 57600)")
    parser.add_argument("--joy",        type=int,   default=0,
                        help="Joystick device index (default: 0)")
    parser.add_argument("--rate",       type=float, default=10.0,
                        help="RC send rate in Hz (default: 10)")
    parser.add_argument("--render",      action="store_true",
                        help="Enable live terminal bar visualization (default: off)")
    parser.add_argument("--no-mavlink", action="store_true",
                        help="Visualize only — do not open a MAVLink connection")
    parser.add_argument("--airframe", default="plane",
                        choices=["plane", "vtail", "elevon"],
                        help="Airframe type for servo label display (default: plane)")
    args = parser.parse_args()

    # ── MAVLink ──────────────────────────────────────────────────────────────
    master = None
    if not args.no_mavlink:
        if not _HAVE_MAV:
            print("pymavlink not installed — run: pip3 install pymavlink")
            sys.exit(1)
        print(f"[MAV] Connecting to {args.connect} ...")
        master = mavutil.mavlink_connection(args.connect, baud=args.baud)
        master.wait_heartbeat()
        master.mav.srcSystem = 255   # GCS sysid — required for RC override
        print(f"[MAV] Heartbeat from sysid={master.target_system} "
              f"compid={master.target_component}")

    # ── Joystick ─────────────────────────────────────────────────────────────
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("[JOY] No joystick detected")
        sys.exit(1)
    if args.joy >= count:
        print(f"[JOY] Index {args.joy} out of range — found {count} device(s):")
        for i in range(count):
            j = pygame.joystick.Joystick(i)
            j.init()
            print(f"  [{i}] {j.get_name()}")
        sys.exit(1)

    joy = pygame.joystick.Joystick(args.joy)
    joy.init()
    joy_name = joy.get_name()
    n_ax     = joy.get_numaxes()
    n_btn    = joy.get_numbuttons()
    print(f"[JOY] [{args.joy}] {joy_name}  axes={n_ax}  buttons={n_btn}")
    print(f"[JOY] CH5 buttons: btn6={'OK' if n_btn > 6 else 'MISSING (CH5 stuck at 1000)'} "
          f"btn7={'OK' if n_btn > 7 else 'MISSING (CH5 stuck at 1000 or 1500)'}")
    print(f"[JOY] CH6 buttons: btn4={'OK' if n_btn > 4 else 'MISSING'} "
          f"btn5={'OK' if n_btn > 5 else 'MISSING (CH6 stuck at 2000)'}")

    interval      = 1.0 / args.rate
    n_sent        = 0
    running       = True
    last_render   = 0.0
    last_print    = 0.0
    prev_arm_ok   = False
    prev_disarm_ok= False

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    connect_label = args.connect if master else "(no MAVLink)"
    servo_raw     = [0] * 8
    sv_labels     = _SV_LABELS[args.airframe]

    try:
        while running:
            t0 = time.monotonic()

            pygame.event.pump()
            channels = read_channels(joy)

            # Evaluate gesture intent from raw joystick values using loose thresholds.
            # ARM gesture:    thr<GESTURE_THR  yaw>+GESTURE_YAW  |roll|<GESTURE_ROLL  |pitch|<GESTURE_PITCH
            # DISARM gesture: thr<GESTURE_THR  yaw<-GESTURE_YAW  (same roll/pitch/mode)
            ch1, ch2, ch3, ch4, ch5 = channels[:5]
            n1 = _normalize(ch1)
            n2 = _normalize(ch2)
            n3 = _normalize(ch3)
            n4 = _normalize(ch4)

            thr_ok    = n3 < GESTURE_THR
            roll_ok   = abs(n1) < GESTURE_ROLL
            pitch_ok  = abs(n2) < GESTURE_PITCH
            mode_ok   = ch5 <= 1100
            arm_ok    = thr_ok and n4 >  GESTURE_YAW and roll_ok and pitch_ok and mode_ok
            disarm_ok = thr_ok and n4 < -GESTURE_YAW and roll_ok and pitch_ok and mode_ok

            # Snap to exact gesture values so PX4's stricter internal thresholds always pass
            if arm_ok:
                channels = [1500, 1500, 1000, 2000, 1000] + channels[5:]
            elif disarm_ok:
                channels = [1500, 1500, 1000, 1000, 1000] + channels[5:]

            if master is not None:
                send_override(master, channels)
                n_sent += 1

            state_changed = (arm_ok != prev_arm_ok) or (disarm_ok != prev_disarm_ok)
            prev_arm_ok   = arm_ok
            prev_disarm_ok= disarm_ok

            now = time.monotonic()
            # Print immediately on state change, every 0.2s when attempting gesture, else 1s
            print_interval = 0.0 if state_changed else (0.2 if thr_ok else 1.0)
            if now - last_print >= print_interval:
                last_print = now
                def _miss(conds):
                    return ",".join(c for c, ok in conds if not ok) or "—"
                arm_miss    = _miss([("thr",thr_ok),(f"yaw>{GESTURE_YAW}",n4>GESTURE_YAW),
                                     ("roll",roll_ok),("pitch",pitch_ok),("MANUAL",mode_ok)])
                disarm_miss = _miss([("thr",thr_ok),(f"yaw<-{GESTURE_YAW}",n4<-GESTURE_YAW),
                                     ("roll",roll_ok),("pitch",pitch_ok),("MANUAL",mode_ok)])
                print(
                    f"[NORM] roll={n1:+.2f} pitch={n2:+.2f} thr={n3:+.2f} yaw={n4:+.2f} "
                    f"CH5={ch5} "
                    f"| ARM={'*** READY - hold 3s ***' if arm_ok else f'no({arm_miss})'} "
                    f"| DISARM={'*** READY - hold 3s ***' if disarm_ok else f'no({disarm_miss})'}"
                )

            if args.render:
                now = time.monotonic()
                if now - last_render >= 0.1:   # render at 10 Hz max
                    last_render = now
                    if master is not None:
                        while True:
                            msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
                            if msg is None:
                                break
                            servo_raw = [
                                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                                msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
                            ]
                    render(channels, joy_name, connect_label, n_ax, n_btn, n_sent, servo_raw,
                           sv_labels, args.airframe)

            elapsed = time.monotonic() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    finally:
        if master is not None:
            release_override(master)
            print("\n[MAV] RC override released")
        joy.quit()
        pygame.quit()
        print("[JOY] Closed")


if __name__ == "__main__":
    main()
