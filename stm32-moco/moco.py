#!/usr/bin/env python3
"""
moco.py  -  MOCO Motor Controller terminal UI

Runs on any platform with Python 3.7+ and pyserial.
Works over USB CDC serial to the STM32 moco board.

Install:  pip install pyserial
Usage:    python moco.py [PORT]          e.g.  python moco.py COM5
                                               python moco.py /dev/ttyACM0
          If PORT is omitted the script auto-detects the first USB CDC device.

          python moco.py [PORT] --raw     raw line-at-a-time CMD mode

Key bindings (TUI mode):
  1/2/3          select motor
  E / D          enable / disable selected motor
  F / R          forward / reverse
  Up / Down      duty +10 / -10
  PgUp / PgDn    duty +100 / -100
  0              zero duty
  A / Z          next / prev phase map permutation
  T              reset ticks
  C              clear hall sequence (clears ring on firmware too)
  M              enter hallmonitor streaming mode (any key to exit)
  S              stop all
  Q / Ctrl-C     quit
"""

import sys
import time
import threading
import curses
import serial
import serial.tools.list_ports
from collections import deque

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MOTOR_COUNT = 3
DUTY_MAX    = 1440
HALLMON_LEN = 64   # match firmware HALL_RING_LEN for full visibility

PHASE_PERMS = [
    (0,1,2),(0,2,1),(1,0,2),(1,2,0),(2,0,1),(2,1,0)
]

# ---------------------------------------------------------------------------
# Auto-detect USB CDC port
# ---------------------------------------------------------------------------
def find_port():
    candidates = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if any(k in desc or k in mfr for k in
               ["stm32","cdc","serial","stmicro","arm"]):
            candidates.append(p.device)
    if candidates:
        return candidates[0]
    ports = serial.tools.list_ports.comports()
    if ports:
        return ports[0].device
    return None

# ---------------------------------------------------------------------------
# Serial communication layer
# ---------------------------------------------------------------------------
class MocoSerial:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._lock = threading.Lock()
        self._response_queue = deque()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="replace").rstrip("\r")
                        if text:
                            self._response_queue.append(text)
            except Exception:
                time.sleep(0.01)

    def send(self, cmd: str):
        with self._lock:
            self.ser.write((cmd + "\n").encode())

    def send_byte(self, b: bytes):
        """Send a raw byte (used for single-key TUI commands like Q)."""
        with self._lock:
            self.ser.write(b)

    def drain(self):
        lines = []
        while self._response_queue:
            lines.append(self._response_queue.popleft())
        return lines

    def send_and_wait(self, cmd: str, timeout=1.0):
        while self._response_queue:
            self._response_queue.popleft()
        self.send(cmd)
        deadline = time.time() + timeout
        lines = []
        while time.time() < deadline:
            while self._response_queue:
                line = self._response_queue.popleft()
                lines.append(line)
                if line.startswith("OK") or line.startswith("ERR"):
                    return lines
            time.sleep(0.005)
        return lines

    def close(self):
        self.ser.close()


def enter_cmd_mode(moco_ser: MocoSerial):
    """
    Reliably switch the firmware from TUI mode to CMD mode.

    The firmware boots into TUI mode (g_mode = MODE_TUI).  In TUI mode
    the CMD dispatcher never runs - every received byte is passed as a
    single keypress to tui_handle_key().  Sending the string "RAW\n"
    therefore fires R (set-reverse), A (next-phase-map), W (no-op), and
    the newline as four separate TUI keys; the firmware never switches
    mode and all subsequent commands are silently ignored.

    The correct exit key from TUI is 'Q', which sets g_mode = MODE_CMD
    and sends "INFO Entered CMD mode.\r\n".  After that the firmware is
    in CMD mode and will respond to line-oriented commands normally.
    """
    # Drain any pending VT100 redraw traffic first.
    time.sleep(0.05)
    moco_ser.drain()

    # Send 'Q' as a single byte - no newline, TUI key handler reads
    # raw bytes one at a time.
    moco_ser.send_byte(b'Q')

    # Wait up to 500 ms for the INFO confirmation banner.
    deadline = time.time() + 0.5
    while time.time() < deadline:
        for line in moco_ser.drain():
            if "CMD mode" in line or line.startswith("OK") or line.startswith("INFO"):
                # Drain any remaining banner lines.
                time.sleep(0.05)
                moco_ser.drain()
                return
        time.sleep(0.02)

    # Fallback: if already in CMD mode (e.g. re-run without power cycle)
    # PING will respond immediately.
    lines = moco_ser.send_and_wait("PING", timeout=0.3)
    if any("PONG" in l for l in lines):
        return

    # Last resort: firmware may still be in TUI; try Q once more.
    moco_ser.send_byte(b'Q')
    time.sleep(0.2)
    moco_ser.drain()


# ---------------------------------------------------------------------------
# Motor state (local mirror)
# ---------------------------------------------------------------------------
class MotorState:
    def __init__(self, idx):
        self.idx       = idx
        self.enabled   = False
        self.duty      = 0
        self.direction = 0
        self.hall      = 0
        self.ticks     = 0
        self.phase_map = [0, 1, 2]
        self.perm_idx  = 0
        self.hallmon   = deque(maxlen=HALLMON_LEN)

    def next_perm(self):
        self.perm_idx = (self.perm_idx + 1) % 6
        self.phase_map = list(PHASE_PERMS[self.perm_idx])

    def prev_perm(self):
        self.perm_idx = (self.perm_idx + 5) % 6
        self.phase_map = list(PHASE_PERMS[self.perm_idx])

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class MocoApp:
    def __init__(self, moco: MocoSerial):
        self.moco           = moco
        self.motors         = [MotorState(i) for i in range(MOTOR_COUNT)]
        self.selected       = 0
        self.status_msg     = "Connected."
        self.gpioa_odr      = 0; self.gpioa_idr = 0
        self.gpiob_odr      = 0; self.gpiob_idr = 0
        self._poll_lock     = threading.Lock()
        self._running       = True
        self._hallmon_mode  = False   # True = fullscreen streaming mode

        # Switch firmware to CMD mode before sending any line commands.
        enter_cmd_mode(moco)
        self._full_refresh()

    # -----------------------------------------------------------------------
    # Polling
    # -----------------------------------------------------------------------
    def _full_refresh(self):
        lines = self.moco.send_and_wait("STATUS", timeout=1.0)
        for line in lines:
            self._parse_status(line)
        lines = self.moco.send_and_wait("HALL", timeout=0.5)
        for line in lines:
            self._parse_hall(line)
        lines = self.moco.send_and_wait("ODRDUMP", timeout=0.5)
        for line in lines:
            self._parse_gpio(line)
        # Load initial hall ring for selected motor
        self._fetch_hallseq(self.selected)

    def poll_once(self):
        """Quick poll: hall state, ticks, GPIO, and full hall ring for selected motor."""
        with self._poll_lock:
            for line in self.moco.send_and_wait("HALL", timeout=0.3):
                self._parse_hall(line)
            for line in self.moco.send_and_wait("TICKS", timeout=0.3):
                self._parse_ticks(line)
            lines = self.moco.send_and_wait("ODRDUMP", timeout=0.3)
            for line in lines:
                self._parse_gpio(line)
            # Always fetch the full ring for the currently selected motor so
            # the hallmon deque reflects every transition, not just snapshots.
            self._fetch_hallseq(self.selected)

    def _fetch_hallseq(self, motor_idx: int):
        """Send HALLSEQ <motor> and populate hallmon from the full ring response."""
        lines = self.moco.send_and_wait(f"HALLSEQ {motor_idx + 1}", timeout=0.5)
        for line in lines:
            # Response: INFO M1 HALLSEQ (736 transitions): 1 5 1 3 4 2 ...
            if "HALLSEQ" in line and ":" in line:
                try:
                    payload = line.split(":", 1)[1].strip()
                    if payload:
                        vals = [int(x, 16) for x in payload.split()]
                        m = self.motors[motor_idx]
                        m.hallmon.clear()
                        for v in vals:
                            m.hallmon.append(v)
                except Exception:
                    pass

    # -----------------------------------------------------------------------
    # Response parsers
    # -----------------------------------------------------------------------
    def _parse_status(self, line):
        if not line.startswith("INFO M"):
            return
        try:
            parts = line.split()
            idx = int(parts[1][1]) - 1
            if idx < 0 or idx >= MOTOR_COUNT:
                return
            m = self.motors[idx]
            for p in parts[2:]:
                k, _, v = p.partition("=")
                if k == "en":     m.enabled   = v == "1"
                elif k == "dir":  m.direction = 0 if v == "FWD" else 1
                elif k == "duty": m.duty      = int(v)
                elif k == "hall": m.hall      = int(v, 16)
                elif k == "ticks":m.ticks     = int(v)
                elif k == "map":
                    nums = v.strip("[]").split(",")
                    m.phase_map = [int(x) for x in nums]
                    try:
                        m.perm_idx = PHASE_PERMS.index(tuple(m.phase_map))
                    except ValueError:
                        m.perm_idx = 0
        except Exception:
            pass

    def _parse_hall(self, line):
        if "HALL:" not in line:
            return
        try:
            parts = line.split()
            idx = int(parts[1][1]) - 1
            if idx < 0 or idx >= MOTOR_COUNT:
                return
            for p in parts[3:]:
                k, _, v = p.partition("=")
                if k == "raw":
                    self.motors[idx].hall = int(v, 16)
        except Exception:
            pass

    def _parse_ticks(self, line):
        if "TICKS:" not in line:
            return
        try:
            parts = line.split()
            idx = int(parts[1][1]) - 1
            if idx < 0 or idx >= MOTOR_COUNT:
                return
            self.motors[idx].ticks = int(parts[3])
        except Exception:
            pass

    def _parse_gpio(self, line):
        try:
            if "GPIOA" in line:
                parts = line.split()
                for p in parts:
                    k, _, v = p.partition("=")
                    if k == "ODR":   self.gpioa_odr = int(v, 16)
                    elif k == "IDR": self.gpioa_idr = int(v, 16)
            elif "GPIOB" in line:
                parts = line.split()
                for p in parts:
                    k, _, v = p.partition("=")
                    if k == "ODR":   self.gpiob_odr = int(v, 16)
                    elif k == "IDR": self.gpiob_idr = int(v, 16)
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # Actions
    # -----------------------------------------------------------------------
    def _cmd(self, cmd):
        lines = self.moco.send_and_wait(cmd, timeout=0.5)
        for line in lines:
            if line.startswith("ERR"):
                self.status_msg = line
                return False
        return True

    def enable(self):  self._cmd(f"EN {self.selected+1}");  self.motors[self.selected].enabled = True
    def disable(self): self._cmd(f"DIS {self.selected+1}"); self.motors[self.selected].enabled = False
    def stop_all(self):
        self._cmd("STOP")
        for m in self.motors: m.enabled = False; m.duty = 0
        self.status_msg = "ALL STOPPED."

    def set_duty(self, delta):
        m = self.motors[self.selected]
        nd = max(0, min(DUTY_MAX, m.duty + delta))
        if self._cmd(f"SET {self.selected+1} {nd}"):
            m.duty = nd
            self.status_msg = f"Duty {nd}"

    def zero_duty(self):
        m = self.motors[self.selected]
        if self._cmd(f"SET {self.selected+1} 0"): m.duty = 0; self.status_msg = "Duty zeroed."

    def set_dir(self, d):
        m = self.motors[self.selected]
        if self._cmd(f"DIR {self.selected+1} {d}"):
            m.direction = d
            self.status_msg = "Dir: " + ("FWD" if d == 0 else "REV")

    def next_map(self):
        m = self.motors[self.selected]
        m.next_perm()
        pm = m.phase_map
        if self._cmd(f"MAP {self.selected+1} {pm[0]} {pm[1]} {pm[2]}"):
            self.status_msg = f"Phase map -> {pm} (perm {m.perm_idx+1}/6)"

    def prev_map(self):
        m = self.motors[self.selected]
        m.prev_perm()
        pm = m.phase_map
        if self._cmd(f"MAP {self.selected+1} {pm[0]} {pm[1]} {pm[2]}"):
            self.status_msg = f"Phase map -> {pm} (perm {m.perm_idx+1}/6)"

    def reset_ticks(self):
        if self._cmd(f"RESETTICKS {self.selected+1}"):
            self.motors[self.selected].ticks = 0
            self.status_msg = "Ticks reset."

    def clear_hallmon(self):
        """Clear the hall ring on the firmware AND locally."""
        self._cmd(f"CLEARRING {self.selected+1}")
        self.motors[self.selected].hallmon.clear()
        self.status_msg = "Hall ring cleared."

    # -----------------------------------------------------------------------
    # Hall monitor streaming mode
    # -----------------------------------------------------------------------
    def run_hallmonitor(self, scr):
        """
        Fullscreen streaming mode: sends HALLMONITOR <motor> to firmware,
        prints every transition as it arrives.  Any key press exits.
        """
        mid = self.selected + 1
        H, W = scr.getmaxyx()
        scr.erase()
        scr.addstr(0, 0, f" HALLMONITOR  Motor {mid}  (any key to stop)",
                   curses.color_pair(3) | curses.A_BOLD)
        scr.addstr(1, 0, "-" * min(60, W - 1), curses.color_pair(6))
        scr.refresh()

        row    = 2
        total  = 0
        # Drain any stale data
        self.moco.drain()
        self.moco.send(f"HALLMONITOR {mid}")
        scr.nodelay(True)

        while True:
            # Check for keypress to exit
            key = scr.getch()
            if key != -1:
                # Tell firmware to stop streaming
                self.moco.send("")
                # Drain until OK
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    for line in self.moco.drain():
                        if line.startswith("OK") or line.startswith("ERR"):
                            break
                    else:
                        time.sleep(0.02)
                        continue
                    break
                break

            # Drain incoming lines
            for line in self.moco.drain():
                if line.startswith("INFO "):
                    payload = line[5:].strip()
                    if "HALLMONITOR running" in payload:
                        continue
                    total += 1
                    label = f"{total:6d}: {payload}"
                    try:
                        if row < H - 1:
                            scr.addstr(row, 0, label[:W - 1], curses.color_pair(3))
                        else:
                            # Scroll: move everything up
                            scr.scroll()
                            row = H - 2
                            scr.addstr(row, 0, label[:W - 1], curses.color_pair(3))
                        scr.refresh()
                        row += 1
                    except curses.error:
                        pass
                elif line.startswith("OK") or line.startswith("ERR"):
                    break

            time.sleep(0.01)

        self.status_msg = f"Hallmonitor stopped after {total} transitions."
        self._hallmon_mode = False

    # -----------------------------------------------------------------------
    # Drawing
    # -----------------------------------------------------------------------
    def draw(self, scr):
        scr.erase()
        H, W = scr.getmaxyx()
        m = self.motors[self.selected]

        def safe_addstr(row, col, text, attr=0):
            try:
                if row < H - 1:
                    scr.addstr(row, col, text[:W - col - 1], attr)
                elif row == H - 1:
                    scr.insstr(row, col, text[:W - col - 2], attr)
            except curses.error:
                pass

        def bar(val, maxv, width):
            filled = int(val * width / maxv) if maxv else 0
            return "[" + "#" * filled + "-" * (width - filled) + "]"

        # Title
        title = "  MOCO Motor Controller  "
        hint  = "[Q]quit  [H]help"
        safe_addstr(0, 0, title, curses.color_pair(3) | curses.A_BOLD)
        safe_addstr(0, len(title), hint, curses.color_pair(4))

        # Tabs
        col = 0
        for i in range(MOTOR_COUNT):
            label = f" M{i+1} "
            attr = curses.A_REVERSE | curses.A_BOLD if i == self.selected else curses.color_pair(4)
            safe_addstr(2, col, label, attr)
            col += len(label) + 1
        safe_addstr(2, col, "  [1/2/3] select", curses.color_pair(4))

        safe_addstr(3, 0, "-" * min(50, W - 1), curses.color_pair(6))

        # Enable
        en_str = "ENABLED " if m.enabled else "DISABLED"
        en_col = curses.color_pair(2) | curses.A_BOLD if m.enabled else curses.color_pair(1) | curses.A_BOLD
        safe_addstr(4, 0, " Status: ")
        safe_addstr(4, 9, en_str, en_col)
        safe_addstr(4, 18, "  [E]enable  [D]disable  [S]stop all", curses.color_pair(4))

        # Direction
        dir_str = "FWD" if m.direction == 0 else "REV"
        dir_col = curses.color_pair(2) if m.direction == 0 else curses.color_pair(4)
        safe_addstr(5, 0, " Dir:    ")
        safe_addstr(5, 9, dir_str, dir_col | curses.A_BOLD)
        safe_addstr(5, 13, "  [F]forward  [R]reverse", curses.color_pair(4))

        # Duty
        duty_bar = bar(m.duty, DUTY_MAX, 20)
        safe_addstr(6, 0, " Duty:   ")
        safe_addstr(6, 9, duty_bar, curses.color_pair(2))
        safe_addstr(6, 31,
            f"  {m.duty:4d}/{DUTY_MAX}  [Up/Dn]+/-10  [PgU/PgD]+/-100  [0]zero",
            curses.color_pair(4))

        # Hall
        h = m.hall
        fault = h == 0 or h == 7
        hall_col = curses.color_pair(1) | curses.A_BOLD if fault else curses.color_pair(3) | curses.A_BOLD
        safe_addstr(7, 0, " Hall:   ")
        safe_addstr(7, 9, f"0x{h:X}  HA={(h>>2)&1} HB={(h>>1)&1} HC={h&1}", hall_col)
        if fault:
            safe_addstr(7, 35, "  [FAULT - sensor dead?]", curses.color_pair(1) | curses.A_BOLD)

        # Hall sequence - full ring from firmware, most recent HALLMON_LEN entries
        seq = list(m.hallmon)
        seq_str = " ".join(f"{v:X}" for v in seq)
        total_label = f"  ({m.ticks} ticks)"
        safe_addstr(8, 0, " HallSeq: ", 0)
        safe_addstr(8, 10, seq_str, curses.color_pair(3))
        safe_addstr(8, 10 + len(seq_str) + 1,
            total_label + "  [C]clear  [M]monitor", curses.color_pair(4))

        # Ticks
        safe_addstr(9, 0, " Ticks:  ")
        safe_addstr(9, 9, str(m.ticks), curses.color_pair(3) | curses.A_BOLD)
        safe_addstr(9, 9 + len(str(m.ticks)) + 1, "  [T]reset", curses.color_pair(4))

        # Phase map
        pm = m.phase_map
        safe_addstr(10, 0,
            f" PhaseMap: [{pm[0]},{pm[1]},{pm[2]}]  perm {m.perm_idx+1}/6  "
            "[A]next  [Z]prev", curses.color_pair(4))

        safe_addstr(11, 0, "-" * min(50, W - 1), curses.color_pair(6))

        # All motors summary
        safe_addstr(12, 0, " All: ")
        col = 6
        for i, mi in enumerate(self.motors):
            en_c = curses.color_pair(2) if mi.enabled else curses.color_pair(1)
            tag  = "EN" if mi.enabled else "DIS"
            dir_c = "F" if mi.direction == 0 else "R"
            safe_addstr(12, col, f"M{i+1}:", 0)
            col += 3
            safe_addstr(12, col, tag, en_c | curses.A_BOLD)
            col += len(tag)
            safe_addstr(12, col, f"/{dir_c}/{mi.duty}  ")
            col += len(f"/{dir_c}/{mi.duty}  ")

        # GPIO
        safe_addstr(13, 0,
            f" GPIO: PA ODR={self.gpioa_odr:04X} IDR={self.gpioa_idr:04X}  "
            f"PB ODR={self.gpiob_odr:04X} IDR={self.gpiob_idr:04X}",
            curses.color_pair(3))

        safe_addstr(14, 0, "-" * min(50, W - 1), curses.color_pair(6))

        safe_addstr(15, 0,
            " [1/2/3]motor  [E/D]en/dis  [F/R]fwd/rev  [Up/Dn]duty  [PgU/D]duty*10",
            curses.color_pair(6))
        safe_addstr(16, 0,
            " [A/Z]phase map  [T]ticks  [C]clear hall  [M]hallmonitor  [S]stop  [Q]quit",
            curses.color_pair(6))

        safe_addstr(17, 0, f" {self.status_msg}", curses.color_pair(4))

        scr.refresh()

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------
    def run(self, scr):
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_RED,     -1)
        curses.init_pair(2, curses.COLOR_GREEN,   -1)
        curses.init_pair(3, curses.COLOR_CYAN,    -1)
        curses.init_pair(4, curses.COLOR_YELLOW,  -1)
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)
        curses.init_pair(6, curses.COLOR_WHITE,   -1)
        scr.nodelay(True)
        scr.keypad(True)
        # Enable scrolling for hallmonitor mode
        scr.scrollok(True)
        scr.idlok(True)

        last_poll     = 0
        POLL_INTERVAL = 0.15

        while self._running:
            if self._hallmon_mode:
                self.run_hallmonitor(scr)
                scr.nodelay(True)
                scr.keypad(True)
                scr.scrollok(True)
                scr.idlok(True)
                continue

            self.draw(scr)

            now = time.time()
            if now - last_poll > POLL_INTERVAL:
                last_poll = now
                t = threading.Thread(target=self.poll_once, daemon=True)
                t.start()

            try:
                key = scr.getch()
            except Exception:
                key = -1

            if key == -1:
                time.sleep(0.02)
                continue

            if key in (ord('q'), ord('Q')):
                break
            elif key in (ord('1'), ord('2'), ord('3')):
                self.selected = key - ord('1')
                self.status_msg = f"Motor {self.selected+1} selected."
                # Fetch ring for newly selected motor immediately
                threading.Thread(
                    target=self._fetch_hallseq, args=(self.selected,), daemon=True
                ).start()
            elif key in (ord('e'), ord('E')): self.enable()
            elif key in (ord('d'), ord('D')): self.disable()
            elif key in (ord('s'), ord('S')): self.stop_all()
            elif key in (ord('f'), ord('F')): self.set_dir(0)
            elif key in (ord('r'), ord('R')): self.set_dir(1)
            elif key == curses.KEY_UP:    self.set_duty(+10)
            elif key == curses.KEY_DOWN:  self.set_duty(-10)
            elif key == curses.KEY_PPAGE: self.set_duty(+100)
            elif key == curses.KEY_NPAGE: self.set_duty(-100)
            elif key == ord('+'): self.set_duty(+10)
            elif key == ord('-'): self.set_duty(-10)
            elif key == ord('0'): self.zero_duty()
            elif key in (ord('a'), ord('A')): self.next_map()
            elif key in (ord('z'), ord('Z')): self.prev_map()
            elif key in (ord('t'), ord('T')): self.reset_ticks()
            elif key in (ord('c'), ord('C')): self.clear_hallmon()
            elif key in (ord('m'), ord('M')):
                self._hallmon_mode = True
            elif key in (ord('h'), ord('H')):
                self.status_msg = (
                    "1/2/3 motor | E/D en/dis | F/R dir | Up/Dn duty | "
                    "A/Z map | T ticks | C hall | M monitor | S stop | Q quit"
                )

        self._running = False
        self.moco.send("STOP")


# ---------------------------------------------------------------------------
# Raw interactive session (--raw flag)
# Does NOT instantiate MocoApp - avoids the STATUS/HALL/ODRDUMP refresh
# storm before the user gets a prompt.
# ---------------------------------------------------------------------------
def run_raw(moco_ser: MocoSerial):
    print("Switching firmware to CMD mode...", flush=True)
    enter_cmd_mode(moco_ser)
    print("Entering RAW command mode. Type commands, Ctrl-C to exit.")
    try:
        while True:
            cmd = input("> ").strip()
            if not cmd:
                continue
            lines = moco_ser.send_and_wait(cmd)
            for line in lines:
                print(line)
    except KeyboardInterrupt:
        print("\nExiting RAW mode.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    port = None
    raw_mode = "--raw" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if args:
        port = args[0]
    if port is None:
        port = find_port()
    if port is None:
        print("ERROR: No serial port found.  Usage: python moco.py [PORT]")
        sys.exit(1)

    print(f"Connecting to {port} ...", flush=True)
    try:
        moco_ser = MocoSerial(port)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    if raw_mode:
        run_raw(moco_ser)
    else:
        app = MocoApp(moco_ser)
        try:
            curses.wrapper(app.run)
        except KeyboardInterrupt:
            pass

    moco_ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
