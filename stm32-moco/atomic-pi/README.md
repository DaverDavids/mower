# Atomic Pi → stm32-moco Communication

## Hardware Connection

Connect the Blue Pill USB port (A11/A12) to any USB port on the Atomic Pi.  
The STM32 enumerates as a USB CDC Virtual COM Port (`/dev/ttyACMx`).

## Finding the Device

```bash
ls /dev/ttyACM*
# or
dmesg | tail -20 | grep tty
```

## Python Tool

```bash
pip3 install pyserial

# Single commands
python3 moco_send.py ping
python3 moco_send.py status
python3 moco_send.py hall
python3 moco_send.py ticks

# Control a motor
python3 moco_send.py set 1 720        # Motor 1, 50% duty
python3 moco_send.py dir 1 0          # Motor 1, forward
python3 moco_send.py en 1             # Enable motor 1
python3 moco_send.py dis 1            # Disable motor 1
python3 moco_send.py stop             # Emergency stop all

# Interactive shell
python3 moco_send.py shell
```

## ASCII Protocol

All commands are newline-terminated ASCII (`\n`).  
Commands are case-insensitive.  
Responses are prefixed `OK`, `ERR`, or `INFO`.

| Prefix | Meaning |
|--------|---------|
| `OK`   | Command succeeded |
| `ERR`  | Bad argument or unknown command |
| `INFO` | Informational/multi-line data |

## Full Command Reference

| Command | Arguments | Description |
|---------|-----------|-------------|
| `PING` | — | Returns `OK PONG` |
| `STOP` | — | Disable all motors immediately |
| `SET` | `<motor 1-3> <duty 0-1440>` | Set PWM duty cycle |
| `DIR` | `<motor 1-3> <0\|1>` | Direction (0=fwd, 1=rev) |
| `EN` | `<motor 1-3>` | Enable motor |
| `DIS` | `<motor 1-3>` | Disable motor |
| `STATUS` | — | Full status dump |
| `HALL` | — | Raw Hall sensor readings |
| `TICKS` | — | Hall tick counters (position) |
| `RESETTICKS` | `[<motor 1-3>]` | Reset tick counter(s) |
| `MAP` | `<motor 1-3> <p0> <p1> <p2>` | Remap phase outputs (debug) |
| `GETMAP` | `<motor 1-3>` | Read current phase map |
| `HELP` | — | Print command list |

## Phase Remapping for Pin Debug

If a motor runs backwards, spins wrong, or doesn't commutate, use `MAP` to
swap phase channels **in firmware** without rewiring:

```
MAP 1 0 1 2     # identity – no swap (default)
MAP 1 1 0 2     # swap CH1 and CH2 on motor 1
MAP 1 2 1 0     # rotate channels
```

Try all 6 permutations of `{0,1,2}` until the motor spins correctly, then
hardcode the working map in `motor_ctrl.c`'s `Motor_Init()` if desired.

## Distance / Position Feedback

`TICKS` returns the cumulative Hall transition count per motor since last reset.  
Convert to distance using:

```
distance = ticks * (2 * π * wheel_radius) / (pole_pairs * 6)
```

- Each electrical cycle = 6 Hall transitions
- `pole_pairs` = (number of motor poles) / 2
- This gives distance in the same units as `wheel_radius`

## Using `screen` or `minicom` for Raw Serial Debug

```bash
# screen
screen /dev/ttyACM0

# minicom (no baud needed for CDC, but must set something)
minicom -D /dev/ttyACM0 -b 115200

# Exit screen: Ctrl-A then K
```

Type `HELP` after connecting to see all available commands.
