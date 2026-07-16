
## Navigation fix

In the pin-test screen, find the handler for cursor keys or `curses.KEY_LEFT` / `curses.KEY_RIGHT`. Right now left/right is apparently incrementing and decrementing the selected **row**, so change that logic to `KEY_UP => selected_index -= 1` and `KEY_DOWN => selected_index += 1`, with bounds clamp or wraparound, and remove left/right from vertical navigation.

Also update the help text on that screen so it explicitly says `Up/Down: select pin`. If you keep horizontal keys at all, reserve them for page changes or no-op them so the visual model matches the list layout.

## Control wording

In the pin-test UI model, replace any user-facing `high` / `low` labels with `enable` / `disable`. That change should affect:

- On-screen help text.
- Button legends or key hints.
- Status/result text.
- Any confirmation prompt text.

Do **not** rename BLDC pin names like `M3HSA` or `M3LSA`; those are hardware names and should stay as-is. Only change the action verbs used for pin forcing.

## Key bindings

Bind `e` to force the selected output pin **enabled** and `d` to force it **disabled**. For hall inputs, `e` and `d` should either no-op with a status message like `input-only pin` or just trigger a read refresh, because firmware `PINTEST` refuses to drive inputs and only reports `IDR` for them.

```
If the host currently sends `PINTEST <pin> 1` and `PINTEST <pin> 0`, you can keep that transport unchanged for now and just relabel the UI as Enable/Disable. If you want the protocol itself cleaned up, that requires a firmware parser change too. 
```


## Status-line cleanup

Your pin-test status line should display only the response associated with the currently selected action. Since firmware replies are line-oriented and prefixed with `OK`, `ERR`, or `INFO`, the host should:

- Clear the previous per-pin status before issuing a new command.
- Wait for the first matching `OK <pin> ...` or `ERR ...` line.
- Ignore unrelated `INFO ...` lines for the per-pin status widget, or route them to a separate log pane.

That directly addresses the “different info the more I read a pin” symptom, because the firmware emits many legitimate `INFO` lines for other commands and diagnostics. `PINTEST` itself is only supposed to yield one final `OK ...` result for the requested pin.

## Recommended parser rule

For the host-side serial parser, treat these differently:

- `OK <pin> ...` after `PINTEST` or `READPIN`: authoritative result for the selected pin.
- `ERR ...`: show in the pin-test status line immediately.
- `INFO ...`: append to a rolling debug log, not the selected-pin result box.

If you already have a function that sends a command and blocks for a response, make it accept a predicate like “line starts with `OK M3HSA`” so unrelated traffic cannot overwrite the displayed result.

## Optional firmware cleanup

If you want the protocol to match the new UI language, update `usb_cmd.c` so `PINTEST` accepts `E/D` or `ENABLE/DISABLE` in addition to `0/1`, then map those to driven GPIO set/reset internally. Also change the reply string from `wrote=%d` to something like `state=ENABLED` or `state=DISABLED`; that is a firmware enhancement, not required for the host navigation bug.
