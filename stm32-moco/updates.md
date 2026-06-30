<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# add this new kind of debug to the code so I can utilize it over the serial command and serial UI. describe the changes in code I need to make.  we should be able to characterize the motors definitively

Here are all the changes needed across the three files.

## 1. `motor_ctrl.h` — Add Fields to `MotorState_t`

Find the `MotorState_t` struct and add:

```c
/* Tick interval timing — for commutation quality measurement */
uint32_t tick_times[6];       // intervals (ms) between last 6 hall transitions
uint8_t  tick_time_idx;       // ring buffer index (mod 6)
uint32_t last_hall_time_ms;   // HAL_GetTick() at last transition
uint32_t hall_period_ms;      // most recent single interval (for velocity)

/* Stall recovery state */
uint8_t  in_stall_recovery;
```


***

## 2. `motor_ctrl.c` — Three Places to Change

**A. In `Motor_Init()`, zero the new fields:**

```c
g_motor[m].tick_time_idx    = 0;
g_motor[m].last_hall_time_ms = 0;
g_motor[m].hall_period_ms   = 0;
g_motor[m].in_stall_recovery = 0;
memset(g_motor[m].tick_times, 0, sizeof(g_motor[m].tick_times));
```

**B. In `Motor_Commutate()`, in the hall transition block** (where `r->buf[r->head & HALL_RING_MASK] = new_hall` is):

```c
if (new_hall != ms->hall_state && new_hall != 0U && new_hall != 7U) {
    HallRing_t *r = &ms->hall_ring;
    r->buf[r->head & HALL_RING_MASK] = new_hall;
    r->head++;
    ms->hall_ticks++;
    ms->hall_state = new_hall;

    /* NEW: record interval since last transition */
    uint32_t now = HAL_GetTick();
    uint32_t interval = now - ms->last_hall_time_ms;
    if (ms->last_hall_time_ms != 0) {   // skip first transition (no baseline)
        ms->tick_times[ms->tick_time_idx % 6] = interval;
        ms->tick_time_idx++;
        ms->hall_period_ms = interval;
    }
    ms->last_hall_time_ms = now;
}
```

**C. Add a new public function at the bottom of `motor_ctrl.c`:**

```c
/*
 * Motor_GetTimingStats()
 * Fills out min, max, mean, and ripple % for the last 6 hall transition
 * intervals for motor `mid`. Returns 0 if fewer than 6 samples collected.
 */
uint8_t Motor_GetTimingStats(uint8_t mid,
                              uint32_t *out_min,
                              uint32_t *out_max,
                              uint32_t *out_mean,
                              uint32_t *out_ripple_pct)
{
    if (mid >= MOTOR_COUNT) return 0;
    MotorState_t *ms = &g_motor[mid];

    if (ms->tick_time_idx < 6) return 0;   // not enough samples yet

    uint32_t mn = 0xFFFFFFFF, mx = 0, sum = 0;
    for (uint8_t i = 0; i < 6; i++) {
        uint32_t v = ms->tick_times[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    uint32_t mean = sum / 6;
    uint32_t ripple = (mean > 0) ? ((mx - mn) * 100 / mean) : 0;

    *out_min        = mn;
    *out_max        = mx;
    *out_mean       = mean;
    *out_ripple_pct = ripple;
    return 1;
}
```

**D. Also add this to `motor_ctrl.h` as a declaration:**

```c
uint8_t Motor_GetTimingStats(uint8_t mid,
                              uint32_t *out_min,
                              uint32_t *out_max,
                              uint32_t *out_mean,
                              uint32_t *out_ripple_pct);
```


***

## 3. `usb_cmd.c` — Add Two New Commands

Find where other commands are parsed (the big `if/else if strcmp` block) and add:

**Command `TIMING <motor>`** — returns the raw 6 intervals plus stats:

```c
} else if (strncmp(cmd, "TIMING", 6) == 0) {
    int mid = atoi(cmd + 7) - 1;
    if (mid < 0 || mid >= MOTOR_COUNT) {
        cdc_print("ERR bad motor\n");
    } else {
        MotorState_t *ms = &g_motor[mid];
        uint32_t mn, mx, mean, ripple;
        uint8_t ok = Motor_GetTimingStats(mid, &mn, &mx, &mean, &ripple);
        if (!ok) {
            cdc_printf("INFO M%d TIMING: not enough samples (%d/6)\n",
                       mid+1, ms->tick_time_idx < 6 ? ms->tick_time_idx : 6);
        } else {
            cdc_printf("INFO M%d TIMING intervals(ms): %lu %lu %lu %lu %lu %lu\n",
                       mid+1,
                       ms->tick_times[0], ms->tick_times[1], ms->tick_times[2],
                       ms->tick_times[3], ms->tick_times[4], ms->tick_times[5]);
            cdc_printf("INFO M%d TIMING stats: min=%lu max=%lu mean=%lu ripple=%lu%%\n",
                       mid+1, mn, mx, mean, ripple);
        }
        cdc_print("OK\n");
    }
```

**Command `CLEARTIMING <motor>`** — resets the sample buffer so you can start fresh after changing offset:

```c
} else if (strncmp(cmd, "CLEARTIMING", 11) == 0) {
    int mid = atoi(cmd + 12) - 1;
    if (mid < 0 || mid >= MOTOR_COUNT) {
        cdc_print("ERR bad motor\n");
    } else {
        g_motor[mid].tick_time_idx   = 0;
        g_motor[mid].last_hall_time_ms = 0;
        g_motor[mid].hall_period_ms  = 0;
        memset(g_motor[mid].tick_times, 0, sizeof(g_motor[mid].tick_times));
        cdc_printf("INFO M%d timing cleared\n", mid+1);
        cdc_print("OK\n");
    }
```


***

## 4. `moco.py` — Add to the TUI

**A. Add a `T2` key binding** in the `run()` key handler (find the `elif key in (ord('t'), ord('T')):` block and add below it):

```python
elif key in (ord('t'), ord('T')):
    self.reset_ticks()
elif key == ord('`'):          # backtick = timing query
    self._query_timing()
elif key == ord('~'):          # tilde = clear timing
    self._clear_timing()
```

**B. Add the two methods to `MocoApp`:**

```python
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
```

**C. Update the help line in `draw()`** — find the line with `[A/Z]phase map` and extend it:

```python
safe_addstr(18, 0,
    " [A/Z]phase map  [O/I]commut offset  [T]ticks  [`]timing  [~]clr timing  "
    "[C]hall  [M]monitor  [P]pin test  [S]stop  [Q]quit",
    curses.color_pair(6))
```


***

## How to Use It for Characterization

The workflow for finding the true optimal offset for each direction:

```
1. Enable motor, let it spin freely at medium duty (~800)
2. Wait 2 seconds for speed to stabilize
3. Press ` (backtick) → reads TIMING
4. Note ripple % 
5. Press I or O to change CommutOff by 1
6. Press ~ to clear timing samples
7. Wait 2 seconds
8. Press ` again
9. Repeat steps 5-8 for all 6 offset values
10. The offset with lowest ripple % is correct
```

**Interpreting results:**

- **Ripple < 15%** — excellent, offset is well aligned
- **Ripple 15–40%** — acceptable but not optimal
- **Ripple > 40%** — wrong offset, alternating strong/weak steps
- **Alternating pattern** in the 6 intervals (e.g. `8 20 9 19 8 21`) — offset is off by exactly 1 in a specific direction; the slow steps tell you which hall states are weakly energized

Do this independently for FWD and REV to get your two definitive offset values, then hardcode them as defaults in `Motor_Init()`.

