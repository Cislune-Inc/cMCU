#!/usr/bin/env python3
"""Small keyboard commissioning client for the cMCU velocity protocol."""

import argparse
import select
import sys
import termios
import time
import tty

import serial


HELP = "w/s: forward/reverse  a/d: turn  space: stop  h: help  q: quit"


def send(ser: serial.Serial, line: str) -> None:
    ser.write((line + "\n").encode())
    ser.flush()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    sequence = 0
    linear = 0.0
    angular = 0.0
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    send(ser, "HELLO,1")
    print(HELP)

    try:
        tty.setcbreak(sys.stdin.fileno())
        last_send = 0.0
        while True:
            ready, _, _ = select.select([sys.stdin, ser], [], [], 0.02)
            if ser in ready:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    print("\r" + line)
            if sys.stdin in ready:
                key = sys.stdin.read(1)
                if key == "q":
                    break
                if key == "h":
                    print("\r" + HELP)
                elif key == "w":
                    linear, angular = 0.2, 0.0
                elif key == "s":
                    linear, angular = -0.2, 0.0
                elif key == "a":
                    linear, angular = 0.0, 0.5
                elif key == "d":
                    linear, angular = 0.0, -0.5
                elif key == " ":
                    linear, angular = 0.0, 0.0

            now = time.monotonic()
            if now - last_send >= 0.1:
                sequence += 1
                send(ser, f"CMD_VEL,{sequence},{linear:.3f},{angular:.3f}")
                last_send = now
    finally:
        sequence += 1
        send(ser, f"CMD_VEL,{sequence},0,0")
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
