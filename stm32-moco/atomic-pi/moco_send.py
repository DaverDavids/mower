#!/usr/bin/env python3
"""
moco_send.py  –  Atomic Pi CLI tool for the stm32-moco motor controller

Usage:
  python3 moco_send.py [--port /dev/ttyACM0] <command>

Examples:
  python3 moco_send.py ping
  python3 moco_send.py set 1 720
  python3 moco_send.py en 1
  python3 moco_send.py status
  python3 moco_send.py hall
  python3 moco_send.py ticks
  python3 moco_send.py map 2 2 0 1
  python3 moco_send.py stop
  python3 moco_send.py hallmonitor 1     # stream every hall transition for motor 1 (Ctrl-C to stop)
  python3 moco_send.py shell             # enter interactive shell
"""

import serial
import sys
import time
import argparse

#DEFAULT_PORT    = "/dev/ttyACM0"
DEFAULT_PORT    = "COM24"
DEFAULT_BAUD    = 115200   # ignored by CDC, but pyserial requires a value
TIMEOUT_S       = 1.0
READ_LINES      = 8        # max response lines to collect


def open_port(port: str) -> serial.Serial:
    return serial.Serial(port, baudrate=DEFAULT_BAUD, timeout=TIMEOUT_S)


def send_cmd(ser: serial.Serial, cmd: str, wait_lines: int = READ_LINES) -> list[str]:
    """Send a command and collect response lines until timeout or OK/ERR."""
    ser.write((cmd.strip().upper() + "\n").encode())
    ser.flush()
    responses = []
    deadline = time.time() + TIMEOUT_S
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            responses.append(line)
            if line.startswith("OK ") or line.startswith("ERR "):
                break
        if len(responses) >= wait_lines:
            break
    return responses


def print_responses(lines: list[str]):
    for l in lines:
        print(l)


def interactive_shell(ser: serial.Serial):
    print("stm32-moco interactive shell. Type 'exit' or Ctrl-C to quit.")
    print("Commands are sent to the STM32 over USB CDC.")
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting shell.")
            break
        if cmd.lower() in ("exit", "quit", "q"):
            break
        if not cmd:
            continue
        lines = send_cmd(ser, cmd, wait_lines=20)
        print_responses(lines)


def hallmonitor(ser: serial.Serial, motor: int):
    """
    Stream every Hall transition for the given motor (1-3) in real time.

    Sends HALLMONITOR <motor> to put the firmware into streaming mode, then
    prints each INFO line as it arrives.  Press Ctrl-C to stop; this sends
    a newline to the firmware which exits streaming mode and replies
    OK HALLMONITOR stopped.

    Output format:
        <transition_number>: <hall_value>   (hall values 1-6 are valid)
    """
    if motor < 1 or motor > 3:
        print(f"ERROR: motor must be 1-3, got {motor}", file=sys.stderr)
        return

    cmd = f"HALLMONITOR {motor}\n"
    ser.write(cmd.encode())
    ser.flush()

    # Shorter read timeout so we can react quickly to Ctrl-C
    ser.timeout = 0.1

    total = 0

    print(f"[hallmonitor] motor {motor} – streaming hall transitions. Ctrl-C to stop.")
    print(f"[hallmonitor] format: transition_number: value  (values 1-6 are valid)")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            if line.startswith("OK "):
                print(f"\n[hallmonitor] {line}")
                break
            elif line.startswith("ERR "):
                print(f"\n[hallmonitor] {line}", file=sys.stderr)
                break
            elif line.startswith("INFO "):
                payload = line[5:].strip()
                if payload == "HALLMONITOR running - send any key to stop":
                    # Firmware confirmation line – already printed our own header
                    continue
                total += 1
                print(f"{total:6d}: {payload}")
            # Ignore any other line silently

    except KeyboardInterrupt:
        # Send a byte to tell the firmware to exit streaming mode
        ser.write(b"\n")
        ser.flush()
        # Drain the stop acknowledgment
        ser.timeout = 1.0
        while True:
            raw = ser.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if line.startswith("OK ") or line.startswith("ERR "):
                print(f"\n[hallmonitor] {line}")
                break
        print(f"\n[hallmonitor] stopped after {total} transitions.")
    finally:
        ser.timeout = TIMEOUT_S


def main():
    parser = argparse.ArgumentParser(description="stm32-moco host-side tool")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("command", nargs="+",
                        help="Command to send, 'hallmonitor <motor>', or 'shell' for interactive mode")
    args = parser.parse_args()

    try:
        ser = open_port(args.port)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    cmd_str = " ".join(args.command)

    if cmd_str.lower() == "shell":
        interactive_shell(ser)
    elif args.command[0].lower() == "hallmonitor":
        if len(args.command) < 2:
            print("ERROR: usage: hallmonitor <motor 1-3>", file=sys.stderr)
            ser.close()
            sys.exit(1)
        try:
            motor_num = int(args.command[1])
        except ValueError:
            print("ERROR: motor must be an integer 1-3", file=sys.stderr)
            ser.close()
            sys.exit(1)
        # Make sure firmware is in CMD mode before starting stream
        send_cmd(ser, "RAW", wait_lines=1)
        hallmonitor(ser, motor_num)
    else:
        responses = send_cmd(ser, cmd_str)
        print_responses(responses)

    ser.close()


if __name__ == "__main__":
    main()
