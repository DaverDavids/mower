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
  python3 moco_send.py shell         # enter interactive shell
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
    """Send a command and collect response lines until timeout or INFO/OK/ERR."""
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


def main():
    parser = argparse.ArgumentParser(description="stm32-moco host-side tool")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("command", nargs="+",
                        help="Command to send, or 'shell' for interactive mode")
    args = parser.parse_args()

    try:
        ser = open_port(args.port)
        #time.sleep(0.5)   # let USB CDC re-enumerate after DTR assert
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    cmd_str = " ".join(args.command)

    if cmd_str.lower() == "shell":
        interactive_shell(ser)
    else:
        responses = send_cmd(ser, cmd_str)
        print_responses(responses)

    ser.close()


if __name__ == "__main__":
    main()
