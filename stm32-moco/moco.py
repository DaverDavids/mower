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
  O / I          next / prev commutation offset (1-6)
  T              reset ticks
  C              clear hall sequence (clears ring on firmware too)
  M              enter hallmonitor streaming mode (any key to exit)
  P              enter pin test screen
  S              stop all
  Q / Ctrl-C     quit

Pin test screen (P from main):
  Up / Down      select pin
  Space          toggle pin enable/disable (PINTEST cmd - stops timer, drives GPIO)
  E / D          force selected pin enable / disable
  R              read back IDR of selected pin
  A              read ALL pins IDR
  Q / Esc        return to main screen
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
HALLMON_LEN = 64

PHASE_PERMS = [
    (0,1,2),(0,2,1),(1,0,2),(1,2,0),(2,0,1),(2,1,0)
]

# ---------------------------------------------------------------------------
# Pin catalogue  (name, mcu_pin, type, motor_group)
# type: 'HS-PWM', 'LS-GPIO', 'HALL'
# ---------------------------------------------------------------------------
PIN_CATALOGUE = [
    # Motor 1
    ("M1HSA",  "A8",  "HS-PWM", "M1"),
    ("M1HSB",  "A9",  "HS-PWM", "M1"),
    ("M1HSC",  "A10", "HS-PWM", "M1"),
    ("M1LSA",  "B13", "LS-PWM", "M1"),
    ("M1LSB",  "B14", "LS-PWM", "M1"),
    ("M1LSC",  "B15", "LS-PWM", "M1"),
    ("M1HA",   "A0",  "HALL",   "M1"),
    ("M1HB",   "A1",  "HALL",   "M1"),
    ("M1HC",   "A2",  "HALL",   "M1"),
    # Motor 2
    ("M2HSA",  "B6",  "HS-PWM", "M2"),
    ("M2HSB",  "B7",  "HS-PWM", "M2"),
    ("M2HSC",  "B8",  "HS-PWM", "M2"),
    ("M2LSA",  "A15", "LS-GPIO","M2"),
    ("M2LSB",  "B3",  "LS-GPIO","M2"),
    ("M2LSC",  "B5",  "LS-GPIO","M2"),
    ("M2HA",   "A3",  "HALL",   "M2"),
    ("M2HB",   "A4",  "HALL",   "M2"),
    ("M2HC",   "A5",  "HALL",   "M2"),
    # Motor 3
    ("M3HSA",  "B4",  "HS-PWM", "M3"),
    ("M3HSB",  "B0",  "HS-PWM", "M3"),
    ("M3HSC",  "B1",  "HS-PWM", "M3"),
    ("M3LSA",  "B9",  "LS-GPIO","M3"),
    ("M3LSB",  "B11", "LS-GPIO","M3"),
    ("M3LSC",  "B12", "LS-GPIO","M3"),
    ("M3HA",   "A6",  "HALL",   "M3"),
    ("M3HB",   "A7",  "HALL",   "M3"),
    ("M3HC",   "B10", "HALL",   "M3"),
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
                        # Strip any ANSI escape sequences the firmware may emit
                        import re
                        text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)
                        if text:
                            self._response_queue.append(text)
            except Exception:
                time.sleep(0.01)

    def send(self, cmd: str):
        with self._lock:
            self.ser.write((cmd + "\n").encode())

    def send_byte(self, b: bytes):
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
    time.sleep(0.05)
    moco_ser.drain()
    moco_ser.send_byte(b'Q')
    deadline = time.time() + 0.5
    while time.time() < deadline:
        for line in moco_ser.drain():
            if "CMD mode" in line or line.startswith("OK") or line.startswith("INFO"):
                time.sleep(0.05)
                moco_ser.drain()
                return
        time.sleep(0.02)
    lines = moco_ser.send_and_wait("PING", timeout=0.3)
    if any("PONG" in l for l in lines):
        return
    moco_ser.send_byte(b'Q')
    time.sleep(0.2)
    moco_ser.drain()


# ---------------------------------------------------------------------------
# Motor state (local mirror)
# ---------------------------------------------------------------------------
class MotorState:
    def __init__(self, idx):
        self.idx           = idx
        self.enabled       = False
        self.duty          = 0
        self.direction     = 0
        self.hall          = 0
        self.ticks         = 0
        self.phase_map     = [0, 1, 2]
        self.perm_idx      = 0
        self.commut_offset = 0
        self.hallmon       = deque(maxlen=HALLMON_LEN)

    def next_perm(self):
        self.perm_idx = (self.perm_idx + 1) % 6
        self.phase_map = list(PHASE_PERMS[self.perm_idx])

    def prev_perm(self):
        self.perm_idx = (self.perm_idx + 5) % 6
        self.phase_map = list(PHASE_PERMS[self.perm_idx])


# ---------------------------------------------------------------------------
# Pin Test Screen
# ---------------------------------------------------------------------------
class PinTestApp:
    """
    Standalone fullscreen pin tester.  Navigates PIN_CATALOGUE up/down.
    For HS-PWM pins: sends PINTEST <name> <val> which tells firmware to stop
    the timer channel, reconfigure the pin as GPIO push-pull, then drive it.
    For LS-GPIO / HALL pins: falls back to SETPINRAW / READPIN commands.
    """

    def __init__(self, moco: MocoSerial):
        self.moco    = moco
        self.pins    = list(PIN_CATALOGUE)   # (name, mcu, type, group)
        self.sel     = 0
        # per-pin state: written_val (-1=unknown), idr (-1=unknown), mismatch
        self.state   = [{
            "written": -1, "idr": -1, "mismatch": False, "last_resp": ""
        } for _ in self.pins]
        self.status  = "Pin Test Mode.  STOP ALL before testing output pins!"

    # -----------------------------------------------------------------------
    def _send(self, cmd, timeout=1.0):
        return self.moco.send_and_wait(cmd, timeout)

    def _pintest(self, idx, val):
        name = self.pins[idx][0]
        ptype = self.pins[idx][2]
        # Clear previous per-pin status
        self.state[idx]["last_resp"] = ""
        if ptype == "HALL":
            # Input-only: just read IDR
            lines = self._send(f"READPIN {name}")
            resp = " | ".join(lines)
            self.state[idx]["last_resp"] = resp
            for l in lines:
                if "=" in l:
                    try:
                        self.state[idx]["idr"] = int(l.split("=")[1].split()[0])
                    except Exception:
                        pass
            self.status = f"{name}: {resp} (input-only pin)"
            return
        # Output pin: use PINTEST command (firmware reconfigures as GPIO)
        lines = self._send(f"PINTEST {name} {val}")
        self.state[idx]["last_resp"] = ""
        self.state[idx]["written"] = val
        # Parse ODR/IDR from response
        for l in lines:
            if l.startswith("OK ") and name in l and "state=" in l:
                # "OK M3HSA bit=8 state=ENABLED ODR=1 IDR=1"
                self.state[idx]["last_resp"] = l
                try:
                    part = l.split("state=")[1].split()[0]
                    odr_part = l.split("ODR=")[1].split()[0]
                    idr_part = l.split("IDR=")[1].split()[0]
                    self.state[idx]["idr"] = int(idr_part)
                    self.state[idx]["mismatch"] = (int(odr_part) != int(idr_part))
                except Exception:
                    pass
                label = "enabled" if val == 1 else "disabled"
                self.status = f"{name}: {label}  {l.strip()}"
                return
            elif l.startswith("ERR"):
                self.state[idx]["last_resp"] = l
                self.status = f"{name}: {l.strip()}"
                return
        self.status = f"{name} -> {val}  {' | '.join(lines)}"

    def _read_pin(self, idx):
        name = self.pins[idx][0]
        self.state[idx]["last_resp"] = ""
        lines = self._send(f"READPIN {name}")
        for l in lines:
            if l.startswith("OK ") and name in l:
                self.state[idx]["last_resp"] = l
                try:
                    part = l.split("=")[1].split()[0]
                    self.state[idx]["idr"] = int(part)
                except Exception:
                    pass
                self.status = f"{name}: {l.strip()}"
                return
            elif l.startswith("ERR"):
                self.state[idx]["last_resp"] = l
                self.status = f"{name}: {l.strip()}"
                return

    def _read_all(self):
        """Read IDR of every pin via individual READPIN calls."""
        for i, (name, mcu, ptype, grp) in enumerate(self.pins):
            lines = self._send(f"READPIN {name}", timeout=0.5)
            for l in lines:
                if "=" in l:
                    try:
                        part = l.split("=")[1].split()[0]
                        self.state[i]["idr"] = int(part)
                    except Exception:
                        pass
        self.status = "All pins read."

    # -----------------------------------------------------------------------
    def draw(self, scr):
        scr.erase()
        H, W = scr.getmaxyx()

        def safe(row, col, txt, attr=0):
            try:
                if row < H - 1:
                    scr.addstr(row, col, txt[:W - col - 1], attr)
                elif row == H - 1:
                    scr.insstr(row, col, txt[:W - col - 2], attr)
            except curses.error:
                pass

        safe(0, 0, "  MOCO PIN TESTER  ", curses.color_pair(3) | curses.A_BOLD)
        safe(0, 20, "[Up/Down]select  [Space]toggle  [E]nable  [D]isable  [R]read  [A]all  [Q]back",
             curses.color_pair(4))
        safe(1, 0, "-" * min(W - 1, 90), curses.color_pair(6))

        # Header
        safe(2, 0,  f" {'#':>2}  {'Group':<5}  {'Name':<8}  {'Pin':<5}  {'Type':<8}  {'Written':>7}  {'IDR':>4}  {'Status'}",
             curses.color_pair(6) | curses.A_BOLD)
        safe(3, 0, "-" * min(W - 1, 90), curses.color_pair(6))

        row = 4
        last_grp = None
        for i, (name, mcu, ptype, grp) in enumerate(self.pins):
            if row >= H - 3:
                break
            st = self.state[i]
            written_s = str(st["written"]) if st["written"] >= 0 else " -"
            idr_s     = str(st["idr"])     if st["idr"]     >= 0 else " -"

            # group separator
            if grp != last_grp:
                if row < H - 3:
                    safe(row, 0, f" --- {grp} ---", curses.color_pair(4) | curses.A_BOLD)
                    row += 1
                last_grp = grp

            # colour
            if i == self.sel:
                attr = curses.A_REVERSE | curses.A_BOLD
            elif ptype == "HALL":
                attr = curses.color_pair(3)
            elif ptype == "LS-GPIO":
                attr = curses.color_pair(2)
            elif st["mismatch"]:
                attr = curses.color_pair(1) | curses.A_BOLD
            else:
                attr = 0

            mismatch_flag = "  [MISMATCH]" if st["mismatch"] else ""
            resp_short    = st["last_resp"][:30] if st["last_resp"] else ""

            line = (f" {i:>2}  {grp:<5}  {name:<8}  {mcu:<5}  {ptype:<8}"
                    f"  {written_s:>7}  {idr_s:>4}  {resp_short}{mismatch_flag}")
            safe(row, 0, line, attr)
            row += 1

        safe(H - 2, 0, "-" * min(W - 1, 90), curses.color_pair(6))
        safe(H - 1, 0, f" {self.status}", curses.color_pair(4))
        scr.refresh()

    # -----------------------------------------------------------------------
    def run(self, scr):
        scr.nodelay(True)
        scr.keypad(True)
        self.status = "Pin Test Mode.  STOP ALL before testing output pins!"

        while True:
            self.draw(scr)
            try:
                key = scr.getch()
            except Exception:
                key = -1

            if key == -1:
                time.sleep(0.02)
                continue

            if key in (ord('q'), ord('Q'), 27):   # Q or Esc
                break
            elif key == curses.KEY_UP:
                self.sel = (self.sel - 1) % len(self.pins)
                self.status = f"Selected: {self.pins[self.sel][0]} ({self.pins[self.sel][1]})"
            elif key == curses.KEY_DOWN:
                self.sel = (self.sel + 1) % len(self.pins)
                self.status = f"Selected: {self.pins[self.sel][0]} ({self.pins[self.sel][1]})"
            elif key == ord(' '):
                # toggle
                cur = self.state[self.sel]["written"]
                nval = 0 if cur == 1 else 1
                self._pintest(self.sel, nval)
            elif key in (ord('e'), ord('E')):
                self._pintest(self.sel, 1)
            elif key in (ord('d'), ord('D')):
                self._pintest(self.sel, 0)
            elif key in (ord('r'), ord('R')):
                self._read_pin(self.sel)
            elif key in (ord('a'), ord('A')):
                self.status = "Reading all pins..."
                self.draw(scr)
                self._read_all()


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
class MocoApp:
    def __init__(self, moco: MocoSerial):
        self.moco           = moco
        self.motors         = [MotorState(i) for i in range(MOTOR_COUNT)]
        self.selected       = 0
        self.status_msg     = "Connected."
        self.gpioa_odr      = 0; self.gpioa_idr = 0
        self.gpiob_odr      = 0; self.gpiob_idr = 0
        # TIM1 live register snapshot (polled via TIM1REGS cmd)
        self.tim1_moe   = None   # True/False or None if unknown
        self.tim1_ccer  = None   # int or None
        self.tim1_ccr   = [None, None, None]   # CCR1/2/3
        self._poll_lock     = threading.Lock()
        self._running       = True
        self._hallmon_mode  = False
        self._pintest_mode  = False
        self._pin_app       = PinTestApp(moco)

        enter_cmd_mode(moco)
        self._full_refresh()

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
        self._fetch_hallseq(self.selected)
        self._fetch_tim1regs()

    def poll_once(self):
        with self._poll_lock:
            for line in self.moco.send_and_wait("HALL", timeout=0.3):
                self._parse_hall(line)
            for line in self.moco.send_and_wait("TICKS", timeout=0.3):
                self._parse_ticks(line)
            for line in self.moco.send_and_wait("ODRDUMP", timeout=0.3):
                self._parse_gpio(line)
            self._fetch_hallseq(self.selected)
            self._fetch_tim1regs()

    def _fetch_hallseq(self, motor_idx: int):
        lines = self.moco.send_and_wait(f"HALLSEQ {motor_idx + 1}", timeout=0.5)
        for line in lines:
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

    def _fetch_tim1regs(self):
        """Poll TIM1REGS and parse MOE, CCER, CCR1/2/3."""
        lines = self.moco.send_and_wait("TIM1REGS", timeout=0.3)
        for line in lines:
            self._parse_tim1regs(line)

    def _parse_tim1regs(self, line):
        """Parse lines like:
             INFO TIM1 MOE=1 CCER=0x0011 CCR1=720 CCR2=0 CCR3=0
        """
        if "TIM1" not in line:
            return
        try:
            for token in line.split():
                k, _, v = token.partition("=")
                if k == "MOE":
                    self.tim1_moe = (v == "1")
                elif k == "CCER":
                    self.tim1_ccer = int(v, 16)
                elif k == "CCR1":
                    self.tim1_ccr[0] = int(v)
                elif k == "CCR2":
                    self.tim1_ccr[1] = int(v)
                elif k == "CCR3":
                    self.tim1_ccr[2] = int(v)
        except Exception:
            pass

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
                if k == "en":       m.enabled        = v == "1"
                elif k == "dir":    m.direction      = 0 if v == "FWD" else 1
                elif k == "duty":   m.duty           = int(v)
                elif k == "hall":   m.hall           = int(v, 16)
                elif k == "ticks":  m.ticks          = int(v)
                elif k == "offset": m.commut_offset  = int(v)
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

    def next_offset(self):
        m = self.motors[self.selected]
        new_off = (m.commut_offset + 1) % 6
        if self._cmd(f"COMMUTOFFSET {self.selected+1} {new_off}"):
            m.commut_offset = new_off
            self.status_msg = f"CommutOffset -> {new_off}/5"

    def prev_offset(self):
        m = self.motors[self.selected]
        new_off = (m.commut_offset + 5) % 6
        if self._cmd(f"COMMUTOFFSET {self.selected+1} {new_off}"):
            m.commut_offset = new_off
            self.status_msg = f"CommutOffset -> {new_off}/5"

    def reset_ticks(self):
        if self._cmd(f"RESETTICKS {self.selected+1}"):
            self.motors[self.selected].ticks = 0
            self.status_msg = "Ticks reset."

    def _query_timing(self):
        mid = self.selected + 1
        lines = self.moco.send_and_wait(f"TIMING {mid}", timeout=1.0)
        for line in lines:
            if "TIMING" in line:
                self.status_msg = line.replace("INFO ", "")
                return
        self.status_msg = "TIMING: no data yet (need 6+ transitions)"

    def _clear_timing(self):
        mid = self.selected + 1
        self.moco.send_and_wait(f"CLEARTIMING {mid}", timeout=0.5)
        self.status_msg = f"M{mid} timing cleared."

    def clear_hallmon(self):
        self._cmd(f"CLEARRING {self.selected+1}")
        self.motors[self.selected].hallmon.clear()
        self.status_msg = "Hall ring cleared."

    # -----------------------------------------------------------------------
    def run_hallmonitor(self, scr):
        mid = self.selected + 1
        H, W = scr.getmaxyx()
        scr.erase()
        scr.addstr(0, 0, f" HALLMONITOR  Motor {mid}  (any key to stop)",
                   curses.color_pair(3) | curses.A_BOLD)
        scr.addstr(1, 0, "-" * min(60, W - 1), curses.color_pair(6))
        scr.refresh()
        row = 2; total = 0
        self.moco.drain()
        self.moco.send(f"HALLMONITOR {mid}")
        scr.nodelay(True)
        while True:
            key = scr.getch()
            if key != -1:
                self.moco.send("")
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
                            scr.scroll(); row = H - 2
                            scr.addstr(row, 0, label[:W - 1], curses.color_pair(3))
                        scr.refresh(); row += 1
                    except curses.error:
                        pass
                elif line.startswith("OK") or line.startswith("ERR"):
                    break
            time.sleep(0.01)
        self.status_msg = f"Hallmonitor stopped after {total} transitions."
        self._hallmon_mode = False

    # -----------------------------------------------------------------------
    def _draw_tim1_row(self, scr, row, W, safe_addstr):
        """Render one row showing live TIM1 MOE/CCER/CCR state."""
        # Channel enable bits in CCER
        # CC1E=bit0, CC1NE=bit2, CC2E=bit4, CC2NE=bit6, CC3E=bit8, CC3NE=bit10
        CCE_BITS  = [0,  4,  8]
        CCNE_BITS = [2,  6, 10]

        if self.tim1_ccer is None:
            safe_addstr(row, 0, " TIM1: (no data)", curses.color_pair(4))
            return

        moe_s = "MOE=1" if self.tim1_moe else "MOE=0"
        moe_col = curses.color_pair(2) | curses.A_BOLD if self.tim1_moe else curses.color_pair(1) | curses.A_BOLD
        safe_addstr(row, 0, " TIM1: ", 0)
        safe_addstr(row, 7, moe_s, moe_col)
        col = 13

        ch_names = ["CH1", "CH2", "CH3"]
        active_ls = 0
        for i in range(3):
            cce  = (self.tim1_ccer >> CCE_BITS[i])  & 1
            ccne = (self.tim1_ccer >> CCNE_BITS[i]) & 1
            ccr  = self.tim1_ccr[i] if self.tim1_ccr[i] is not None else 0
            if ccne:
                active_ls += 1
            hs_col = curses.color_pair(2) | curses.A_BOLD if cce  else curses.color_pair(6)
            ls_col = curses.color_pair(2) | curses.A_BOLD if ccne else curses.color_pair(6)
            tag = f"{ch_names[i]} HS={'ON ' if cce else 'off'} LS={'ON' if ccne else 'off'} CCR={ccr:<4}  "
            # Colour the whole tag based on HS state
            safe_addstr(row, col, tag, hs_col if cce else (ls_col if ccne else curses.color_pair(6)))
            col += len(tag)

        if active_ls > 1:
            warn = f" !! {active_ls} LS ON !!"
            safe_addstr(row, col, warn, curses.color_pair(1) | curses.A_BOLD)

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

        title = "  MOCO Motor Controller  "
        hint  = "[Q]quit  [H]help  [P]pin test"
        safe_addstr(0, 0, title, curses.color_pair(3) | curses.A_BOLD)
        safe_addstr(0, len(title), hint, curses.color_pair(4))

        col = 0
        for i in range(MOTOR_COUNT):
            label = f" M{i+1} "
            attr = curses.A_REVERSE | curses.A_BOLD if i == self.selected else curses.color_pair(4)
            safe_addstr(2, col, label, attr)
            col += len(label) + 1
        safe_addstr(2, col, "  [1/2/3] select", curses.color_pair(4))

        safe_addstr(3, 0, "-" * min(50, W - 1), curses.color_pair(6))

        en_str = "ENABLED " if m.enabled else "DISABLED"
        en_col = curses.color_pair(2) | curses.A_BOLD if m.enabled else curses.color_pair(1) | curses.A_BOLD
        safe_addstr(4, 0, " Status: ")
        safe_addstr(4, 9, en_str, en_col)
        safe_addstr(4, 18, "  [E]enable  [D]disable  [S]stop all", curses.color_pair(4))

        dir_str = "FWD" if m.direction == 0 else "REV"
        dir_col = curses.color_pair(2) if m.direction == 0 else curses.color_pair(4)
        safe_addstr(5, 0, " Dir:    ")
        safe_addstr(5, 9, dir_str, dir_col | curses.A_BOLD)
        safe_addstr(5, 13, "  [F]forward  [R]reverse", curses.color_pair(4))

        duty_bar = bar(m.duty, DUTY_MAX, 20)
        safe_addstr(6, 0, " Duty:   ")
        safe_addstr(6, 9, duty_bar, curses.color_pair(2))
        safe_addstr(6, 31,
            f"  {m.duty:4d}/{DUTY_MAX}  [Up/Dn]+/-10  [PgU/PgD]+/-100  [0]zero",
            curses.color_pair(4))

        h = m.hall
        fault = h == 0 or h == 7
        hall_col = curses.color_pair(1) | curses.A_BOLD if fault else curses.color_pair(3) | curses.A_BOLD
        safe_addstr(7, 0, " Hall:   ")
        safe_addstr(7, 9, f"0x{h:X}  HA={(h>>2)&1} HB={(h>>1)&1} HC={h&1}", hall_col)
        if fault:
            safe_addstr(7, 35, "  [FAULT - sensor dead?]", curses.color_pair(1) | curses.A_BOLD)

        seq = list(m.hallmon)
        seq_str = " ".join(f"{v:X}" for v in seq)
        total_label = f"  ({m.ticks} ticks)"
        safe_addstr(8, 0, " HallSeq: ", 0)
        safe_addstr(8, 10, seq_str, curses.color_pair(3))
        safe_addstr(8, 10 + len(seq_str) + 1,
            total_label + "  [C]clear  [M]monitor", curses.color_pair(4))

        safe_addstr(9, 0, " Ticks:  ")
        safe_addstr(9, 9, str(m.ticks), curses.color_pair(3) | curses.A_BOLD)
        safe_addstr(9, 9 + len(str(m.ticks)) + 1, "  [T]reset", curses.color_pair(4))

        pm = m.phase_map
        safe_addstr(10, 0,
            f" PhaseMap: [{pm[0]},{pm[1]},{pm[2]}]  perm {m.perm_idx+1}/6  "
            "[A]next  [Z]prev", curses.color_pair(4))

        safe_addstr(11, 0,
            f" CommutOff: {m.commut_offset + 1}/6  [O]next  [I]prev",
            curses.color_pair(4))

        safe_addstr(12, 0, "-" * min(50, W - 1), curses.color_pair(6))

        safe_addstr(13, 0, " All: ")
        col = 6
        for i, mi in enumerate(self.motors):
            en_c = curses.color_pair(2) if mi.enabled else curses.color_pair(1)
            tag  = "EN" if mi.enabled else "DIS"
            dir_c = "F" if mi.direction == 0 else "R"
            safe_addstr(13, col, f"M{i+1}:", 0)
            col += 3
            safe_addstr(13, col, tag, en_c | curses.A_BOLD)
            col += len(tag)
            safe_addstr(13, col, f"/{dir_c}/{mi.duty}/off{mi.commut_offset + 1}  ")
            col += len(f"/{dir_c}/{mi.duty}/off{mi.commut_offset}  ")

        safe_addstr(14, 0,
            f" GPIO: PA ODR={self.gpioa_odr:04X} IDR={self.gpioa_idr:04X}  "
            f"PB ODR={self.gpiob_odr:04X} IDR={self.gpiob_idr:04X}",
            curses.color_pair(3))

        # TIM1 live register row (polled from firmware TIM1REGS cmd)
        self._draw_tim1_row(scr, 15, W, safe_addstr)

        safe_addstr(16, 0, "-" * min(50, W - 1), curses.color_pair(6))
        safe_addstr(17, 0,
            " [1/2/3]motor  [E/D]en/dis  [F/R]fwd/rev  [Up/Dn]duty  [PgU/D]duty*10",
            curses.color_pair(6))
        safe_addstr(18, 0,
            " [A/Z]phase map  [O/I]commut offset  [T]ticks  [`]timing  [~]clr timing  "
            "[C]hall  [M]monitor  [P]pin test  [S]stop  [Q]quit",
            curses.color_pair(6))
        safe_addstr(19, 0, f" {self.status_msg}", curses.color_pair(4))
        scr.refresh()

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
        scr.scrollok(True)
        scr.idlok(True)

        last_poll     = 0
        POLL_INTERVAL = 0.15

        while self._running:
            if self._hallmon_mode:
                self.run_hallmonitor(scr)
                scr.nodelay(True); scr.keypad(True)
                scr.scrollok(True); scr.idlok(True)
                continue

            if self._pintest_mode:
                self._pin_app.run(scr)
                self._pintest_mode = False
                # re-arm curses state
                scr.nodelay(True); scr.keypad(True)
                scr.scrollok(True); scr.idlok(True)
                self.status_msg = "Returned from pin test."
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
            elif key in (ord('o'), ord('O')): self.next_offset()
            elif key in (ord('i'), ord('I')): self.prev_offset()
            elif key in (ord('t'), ord('T')): self.reset_ticks()
            elif key == 96:      # backtick ` — timing query
                self._query_timing()
            elif key == 126:     # tilde ~ — clear timing
                self._clear_timing()
            elif key in (ord('c'), ord('C')): self.clear_hallmon()
            elif key in (ord('m'), ord('M')):
                self._hallmon_mode = True
            elif key in (ord('p'), ord('P')):
                self._pintest_mode = True
            elif key in (ord('h'), ord('H')):
                self.status_msg = (
                    "1/2/3 motor | E/D en/dis | F/R dir | Up/Dn duty | "
                    "A/Z map | O/I offset | T ticks | ` timing | ~ clr timing | "
                    "C hall | M monitor | P pins | S stop | Q quit"
                )

        self._running = False
        self.moco.send("STOP")


# ---------------------------------------------------------------------------
# Raw interactive session (--raw flag)
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
