#!/usr/bin/env python3
"""
Interactive serial helper for the cMCU host link.

Controls
--------
  p -> send PONG
  k -> select KEYBOARD mode
  a -> select AUTONOMOUS mode
  u -> select DUTY drive mode
  v -> select VELOCITY drive mode
  w/s -> throttle +0.5 / -0.5
  d/f -> turn +0.5 / -0.5
  x -> estop on
  z -> estop off
  SPACE -> zero throttle/turn
  : -> explicit command mode
  h -> help
  q -> quit
"""

import argparse
import select
import shutil
import sys
import termios
import time
import tty
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("pyserial is required. Install with: pip3 install pyserial", file=sys.stderr)
    raise


@dataclass
class ControlState:
    throttle: float = 0.0
    turn: float = 0.0
    estop: int = 0

    def clamp(self) -> None:
        self.throttle = max(-1.0, min(1.0, self.throttle))
        self.turn = max(-1.0, min(1.0, self.turn))
        self.estop = 1 if self.estop else 0

    def zero_motion(self) -> None:
        self.throttle = 0.0
        self.turn = 0.0

    def packet(self) -> str:
        self.clamp()
        return f"C,{self.throttle:.1f},{self.turn:.1f},{self.estop}"


@dataclass
class UiState:
    telemetry: str = ""
    last_tx: str = ""
    last_motion_key_at: float = 0.0


HELP = """
  p -> send PONG handshake
  j -> select joystick mode (M,J)
  k -> select keyboard mode (M,K)
  o -> select autonomous mode (M,A)
  u -> select duty drive mode (D,U)
  v -> select velocity drive mode (D,V)
  w/s -> drive forward / reverse at 0.75
  a/d -> turn left / right at 0.75
  x -> estop on
  z -> estop off
  SPACE -> zero throttle/turn immediately
  : -> explicit command mode
  h -> show help
  q -> quit
""".strip()


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()


def redraw(ui: UiState) -> None:
    width = shutil.get_terminal_size((120, 20)).columns
    sys.stdout.write("\r" + " " * max(0, width - 1) + "\r")
    if ui.telemetry:
        line = f"[TEENSY] {ui.telemetry}"
    else:
        line = "[TEENSY] waiting for telemetry"
    if ui.last_tx:
        line += f"  [TX] {ui.last_tx}"
    sys.stdout.write(line[: max(1, width - 1)])
    sys.stdout.flush()


def compact_telemetry(line: str) -> str:
    if line.startswith("ACK,") or line.startswith("ERR,") or line.startswith("DBG,"):
        return ""
    if line.startswith("DBG,"):
        return ""

    parts = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parts[key] = value

    if not parts:
        return line

    required = ("mode", "drive", "host", "batt", "fault")
    if any(key not in parts for key in required):
        return ""

    fields = [
        f"m:{parts['mode']}",
        f"d:{parts['drive']}",
        f"h:{parts['host']}",
        f"b:{parts['batt']}",
        f"f:{parts['fault']}",
    ]
    return " ".join(fields)


def read_key(timeout: float = 0.05):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None
    return sys.stdin.read(1)


def maybe_read_teensy(ser: serial.Serial, ui: UiState) -> None:
    try:
        waiting = ser.in_waiting
    except OSError:
        return
    if not waiting:
        return
    try:
        data = ser.read(waiting).decode("utf-8", errors="replace")
    except OSError:
        return
    for line in data.splitlines():
        if line.strip():
            stripped = line.strip()
            compact = compact_telemetry(stripped)
            if compact:
                ui.telemetry = compact
                redraw(ui)


def explicit_mode(state: ControlState, ser: serial.Serial, ui: UiState) -> None:
    print("\nExplicit command mode.")
    print("Commands: pong, mode <J|K|A>, drive <U|V>, set <thr|turn> <value>, estop <0|1>, zero, send, exit")
    while True:
        try:
            line = input("cmd> ").strip()
        except EOFError:
            print()
            return

        if not line:
            continue
        if line in {"exit", "quit", "q"}:
            redraw(ui)
            return
        if line == "pong":
            send_line(ser, "PONG")
            ui.last_tx = "PONG"
            redraw(ui)
            continue
        if line == "zero":
            state.zero_motion()
            print("zeroed motion")
            continue
        if line == "send":
            pkt = state.packet()
            send_line(ser, pkt)
            ui.last_tx = pkt
            redraw(ui)
            continue

        parts = line.split()
        if len(parts) == 2 and parts[0] == "mode" and parts[1] in {"J", "K", "A"}:
            pkt = f"M,{parts[1]}"
            send_line(ser, pkt)
            ui.last_tx = pkt
            redraw(ui)
            continue
        if len(parts) == 2 and parts[0] == "drive" and parts[1] in {"U", "V"}:
            pkt = f"D,{parts[1]}"
            send_line(ser, pkt)
            ui.last_tx = pkt
            redraw(ui)
            continue
        if len(parts) == 2 and parts[0] == "estop" and parts[1] in {"0", "1"}:
            state.estop = int(parts[1])
            print(f"estop = {state.estop}")
            continue
        if len(parts) == 3 and parts[0] == "set" and parts[1] in {"thr", "turn"}:
            try:
                value = float(parts[2])
            except ValueError:
                print("value must be numeric")
                continue
            if parts[1] == "thr":
                state.throttle = value
            else:
                state.turn = value
            state.clamp()
            print(state.packet())
            continue

        print("unknown command")


def handle_key(key, state: ControlState, ser: serial.Serial, ui: UiState):
    if key is None:
        return True
    if key == "p":
        send_line(ser, "PONG")
        ui.last_tx = "PONG"
        redraw(ui)
        return True
    if key == "k":
        send_line(ser, "M,K")
        ui.last_tx = "M,K"
        redraw(ui)
        return True
    if key == "j":
        send_line(ser, "M,J")
        ui.last_tx = "M,J"
        redraw(ui)
        return True
    if key == "o":
        send_line(ser, "M,A")
        ui.last_tx = "M,A"
        redraw(ui)
        return True
    if key == "u":
        send_line(ser, "D,U")
        ui.last_tx = "D,U"
        redraw(ui)
        return True
    if key == "v":
        send_line(ser, "D,V")
        ui.last_tx = "D,V"
        redraw(ui)
        return True
    if key == "w":
        state.throttle = 0.75
        state.turn = 0.0
        ui.last_motion_key_at = time.monotonic()
    elif key == "s":
        state.throttle = -0.75
        state.turn = 0.0
        ui.last_motion_key_at = time.monotonic()
    elif key == "a":
        state.throttle = 0.0
        state.turn = 0.75
        ui.last_motion_key_at = time.monotonic()
    elif key == "d":
        state.throttle = 0.0
        state.turn = -0.75
        ui.last_motion_key_at = time.monotonic()
    elif key == "x":
        state.estop = 1
    elif key == "z":
        state.estop = 0
    elif key == " ":
        state.zero_motion()
        ui.last_motion_key_at = 0.0
    elif key == ":":
        return "explicit"
    elif key == "h":
        print("\n" + HELP)
        return True
    elif key == "q":
        return False
    else:
        return True

    pkt = state.packet()
    send_line(ser, pkt)
    ui.last_tx = pkt
    redraw(ui)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive host serial controller for cMCU Teensy.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=0.02, help="Serial timeout in seconds")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    state = ControlState()
    ui = UiState()
    last_motion_send = 0.0
    print(f"Opened {args.port} @ {args.baud}")
    print(HELP)
    redraw(ui)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            maybe_read_teensy(ser, ui)
            result = handle_key(read_key(0.05), state, ser, ui)
            if result is False:
                break
            now = time.monotonic()
            if ui.last_motion_key_at and (now - ui.last_motion_key_at) >= 0.2:
                state.zero_motion()
                ui.last_motion_key_at = 0.0
            if now - last_motion_send >= 0.1:
                pkt = state.packet()
                send_line(ser, pkt)
                ui.last_tx = pkt
                redraw(ui)
                last_motion_send = now
            if result == "explicit":
                print()
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                try:
                    explicit_mode(state, ser, ui)
                finally:
                    old_settings = termios.tcgetattr(fd)
                    tty.setcbreak(fd)
                    redraw(ui)
    finally:
        try:
            state.zero_motion()
            send_line(ser, state.packet())
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ser.close()
        print("\nClosed serial port.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
