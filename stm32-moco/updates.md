<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# leave the list order as it is and just make each motor have its proper default. describe all changes again

Here are all the changes:

***

## `motor_ctrl.h`

```c
#define FIRMWARE_VERSION  "moco v1.3"
```


***

## `motor_ctrl.c` — `Motor_Init()`

M2 block, replace the three `phase_map` lines:

```c
g_motor[1].phase_map[0] = 2;
g_motor[1].phase_map[1] = 0;
g_motor[1].phase_map[2] = 1;
```

M3 block, replace the three `phase_map` lines:

```c
g_motor[2].phase_map[0] = 0;
g_motor[2].phase_map[1] = 1;
g_motor[2].phase_map[2] = 2;
```


***

## `motor_ctrl.c` — `Motor_Enable()`

Replace:

```c
ms->force_steps    = 60;
ms->force_duty     = 200;
```

With:

```c
ms->force_steps    = 120;
ms->force_duty     = (ms->duty > 0 && ms->duty < 400) ? ms->duty : 400;
```


***

## `usb_cmd.c` — `current_perm_idx`

The current list is `{2,1,0},{2,0,1},{1,2,0},{1,0,2},{0,2,1},{0,1,2}` — indices 0–5.

- M1 default map `[0,1,2]` = index **5**
- M2 default map `[2,0,1]` = index **1**
- M3 default map `[0,1,2]` = index **5**

Replace:

```c
static uint8_t current_perm_idx[MOTOR_COUNT] = {5,5,0};
```

With:

```c
static uint8_t current_perm_idx[MOTOR_COUNT] = {5,1,5};
```

That's it — no list reordering needed.

