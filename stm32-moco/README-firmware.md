# MOCO Firmware — Bring-Up Guide

## Hardware: STM32F103C8T6 Blue Pill

### Quick Pin Reference

| Motor | High-Side PWM | Low-Side / Enable | Timer |
|-------|--------------|-------------------|-------|
| M1    | A8/A9/A10    | B13/B14/B15 (CHN) | TIM1 (dead-time HW) |
| M2    | B6/B7/B8     | A15, B3, B5 (GPIO) | TIM4 |
| M3    | B4/B0/B1     | B9, B11, B12 (GPIO) | TIM3 |

| Motor | Hall A | Hall B | Hall C |
|-------|--------|--------|--------|
| M1    | A0     | A1     | A2     |
| M2    | A3     | A4     | A5     |
| M3    | A6     | A7     | B10    |

**System pins**: USB D-/D+ = A11/A12 | SWD = A13/A14

---

## Flashing

1. Set BOOT0 jumper HIGH.
2. Flash via USB DFU using STM32CubeProgrammer.
3. Set BOOT0 LOW, power cycle.

---

## USB CDC Middleware (Required for comms)

The comms layer compiles without CDC middleware but is a no-op until it's added.

1. Open CubeMX, enable **Middleware → USB_DEVICE → Communication Device Class (CDC)**.
2. Regenerate code.
3. In `USB_DEVICE/App/usbd_cdc_if.c`, add to `CDC_Receive_FS()`:
   ```c
   #include "usb_cdc_comms.h"
   COMMS_RxCallback(Buf, *Len);
   ```
4. Rebuild and flash.

The Atomic Pi will see the STM32 as `/dev/ttyACM0`.

---

## Atomic Pi Command Protocol

Open the port at any baud rate (USB CDC ignores baud):
```bash
screen /dev/ttyACM0
# or
cat /dev/ttyACM0 &
echo 'P' > /dev/ttyACM0
```

| Command        | Action                                |
|----------------|---------------------------------------|
| `P\n`          | Ping → `PONG`                         |
| `?\n`          | Status of all 3 motors                |
| `E<m>\n`       | Enable motor m (0/1/2)                |
| `D<m>\n`       | Disable motor m                       |
| `DA\n`         | **Emergency stop** — disable all      |
| `S<m>,<duty>\n`| Set duty 0–3599 (0=off, 3599=100%)    |
| `R<m>,<dir>\n` | Direction: 0=forward, 1=reverse       |
| `C<m>\n`       | Reset commutation counter for motor m |

### Status Response Format
```
STATUS:<motor>,<enabled>,<duty>,<direction>,<commut_count>,<hall_errors>
```
`commut_count` increments on every Hall state transition — use as distance feedback.

---

## Safe Default Behavior

- All motors start **disabled** at boot (0% duty, all enables LOW).
- Invalid Hall state (0b000 or 0b111) immediately kills that motor's outputs.
- Any HAL fault triggers `Error_Handler()` which calls `MC_DisableAll()` before locking up.
- LED blinks rapidly on fault.

---

## Commutation Table Calibration

The 6-step tables in `motor_controller.c` are a standard starting point.
If a motor spins the wrong direction, either:
- Send `R<m>,1\n` to reverse direction, or
- Swap any two phase wires on that motor.

If a motor vibrates but doesn't spin, two Hall sensor wires are likely swapped.
Use `?\n` to watch `commut_count` increment — it should step through all 6 states smoothly.

---

## Simple Python Script (Atomic Pi)

```python
import serial, time

ser = serial.Serial('/dev/ttyACM0', timeout=1)

def cmd(c):
    ser.write((c + '\n').encode())
    return ser.readline().decode().strip()

print(cmd('P'))          # PONG
print(cmd('?'))          # status
cmd('E0')                # enable motor 0
cmd('S0,360')            # ~10% duty
time.sleep(2)
print(cmd('?'))          # check commut_count
cmd('DA')                # stop all
ser.close()
```
