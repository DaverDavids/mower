# stm32-moco — STM32 Motor Controller

Low-level motor controller for the autonomous mower. Runs on an **STM32F103C8T6** (Blue Pill).
Receives drive commands from the Atomic Pi (ROS 2) over UART/USB-serial and returns wheel encoder feedback.

---

## Files

| File | Purpose |
|---|---|
| `moco_main.ino` | Main sketch (setup + loop, state machine) |
| `moco_config.h` | All tunable constants (baud, pins, wheel dims) |
| `motor.h/.cpp` | PWM motor drive + soft ramp |
| `encoder.h/.cpp` | Quadrature encoder via hardware timers TIM3/TIM4 |
| `comms.h/.cpp` | Serial command parser + feedback output |
| `watchdog.h/.cpp` | Software comms watchdog + IWDG helper |
| `project-and-pins.txt` | Full pin reference and calibration notes |

---

## Quick Start

### 1. Flash
- Open `moco_main.ino` in **Arduino IDE** with the [STM32duino board package](https://github.com/stm32duino/Arduino_Core_STM32) installed.
- Board: **Generic STM32F1 series** → STM32F103C8Tx (Blue Pill)
- Upload method: STM32CubeProgrammer (DFU) or STLINK

### 2. First Hook-Up (Debug Workflow)
1. Connect USB-serial adapter to PA9 (TX) / PA10 (RX) / GND.
2. Open a terminal at **115200 baud**. You should see:
   ```
   MOCO:READY
   DBG:State=SAFE. Send PING to confirm, then L:0,R:0 to arm.
   ```
3. Send `PING\n` → expect `PONG`
4. Send `DEBUG:ON\n` → enables verbose `DBG:` output
5. Send `L:0,R:0\n` → transitions to RUN state
6. Send `L:100,R:100\n` → both motors forward at ~39% speed
7. Send `ESTOP\n` → immediate stop, returns to SAFE

### 3. Calibration
Edit `moco_config.h`:
- `WHEEL_DIAMETER_MM` — measure your actual drive wheel OD in mm
- `ENCODER_PPR` — pulses per revolution (single channel, check your encoder datasheet)
- `PWM_DEADBAND` — increase if motors creep near zero command

### 4. ROS 2 Integration (Atomic Pi)
The `base_controller` ROS 2 node sends:
```
L:<left_pwm>,R:<right_pwm>\n
```
and reads:
```
ENC:L<ticks>,R<ticks>,DL<mm>,DR<mm>\n
```
Default serial device: `/dev/ttyUSB0` at 115200.

---

## Serial Protocol Reference

### Commands (Atomic Pi → STM32)
| Command | Description |
|---|---|
| `L:<n>,R:<n>\n` | Set motor speeds. n = -255 to 255. 0 = brake. |
| `ESTOP\n` | Emergency stop. Returns to SAFE state. |
| `PING\n` | Connectivity check. Returns `PONG`. |
| `DEBUG:ON\n` | Enable verbose debug output. |
| `DEBUG:OFF\n` | Disable debug output. |
| `RESET_ENC\n` | Zero both encoder tick counters. |
| `STATUS\n` | Request immediate encoder feedback. |

### Responses (STM32 → Atomic Pi)
| Response | Description |
|---|---|
| `MOCO:READY\n` | Sent once on boot. |
| `PONG\n` | Reply to PING. |
| `ENC:L<t>,R<t>,DL<mm>,DR<mm>\n` | Periodic feedback at 10 Hz. |
| `DBG:<msg>\n` | Debug message (only when DEBUG:ON). |
| `ERR:<msg>\n` | Error or fault message. |

---

## Safety Features

- **SAFE state on boot** — motors are off until the first drive command.
- **Comms watchdog (500 ms)** — if no command is received for 500 ms while in RUN state, motors stop and return to SAFE. Prevents runaway if the Atomic Pi crashes or serial drops.
- **Hardware IWDG (~2.5 s)** — resets the MCU entirely if the main loop locks up.
- **ESTOP command** — forces immediate stop from any state.
- **Fault LED (PB12)** — illuminates whenever in SAFE or FAULT state due to watchdog/estop.
- **Heartbeat LED (PC13)** — fast blink (200 ms) = RUN, slow blink (1000 ms) = SAFE.

---

## Wiring Summary

```
STM32 Blue Pill          H-Bridge (L298N / TB6612)
─────────────────────────────────────────────────
PA0  (TIM2_CH1 PWM)  --> ENA (Left motor speed)
PB0  (GPIO OUT)      --> IN1 (Left direction A)
PB1  (GPIO OUT)      --> IN2 (Left direction B)
PA1  (TIM2_CH2 PWM)  --> ENB (Right motor speed)
PB10 (GPIO OUT)      --> IN3 (Right direction A)
PB11 (GPIO OUT)      --> IN4 (Right direction B)

PA6  (TIM3_CH1)      <-- Left Encoder A
PA7  (TIM3_CH2)      <-- Left Encoder B
PB6  (TIM4_CH1)      <-- Right Encoder A
PB7  (TIM4_CH2)      <-- Right Encoder B

PA9  (USART1 TX)     --> Atomic Pi RX
PA10 (USART1 RX)     <-- Atomic Pi TX
GND                  --- Common GND (STM32 + driver + Pi)

PC13 (LED, active LOW)  Heartbeat blink
PB12 (LED, active HIGH) Fault/SAFE indicator
```

---

## TODO / Future
- [ ] Tune `WHEEL_DIAMETER_MM` and `ENCODER_PPR` to actual hardware
- [ ] Verify encoder direction (swap A/B if ticks go wrong direction)
- [ ] Tune `PWM_DEADBAND` to prevent motor creep
- [ ] Add battery voltage monitoring (ADC on PA4) for under-voltage cutoff
- [ ] Implement PID closed-loop speed control (replace open-loop PWM)
- [ ] Add CRC or checksum to serial protocol for noise immunity
