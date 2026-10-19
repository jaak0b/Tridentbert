# CFlap: CPAP flap mode for Kalico

Remote CPAP blowers (e.g. a WS9290) move a lot of air, but they are slow to change speed: spinning up and especially
spinning down takes a second or more. A slicer's "fan speed-up time" can start the fan earlier, but it does nothing for
slowing down.

**CFlap keeps the blower at a constant speed during a print and controls toolhead cooling with a servo-driven flap.**
The flap diverts the air the toolhead shouldn't get into a bypass. It covers its full travel in about 0.13 s, so cooling
changes are almost instant in both directions.

## How it works

1. **PRINT_START** runs `CPAP_SCAN`. The plugin scans the sliced G-code (in a separate process, so Klipper is never
   blocked) for the highest part-fan `M106` and stores it as `printer.cpap_flap.fan_max`.
2. **At the end of PRINT_START**, `_CPAP_MODE ACTIVE=1` starts the blower at exactly that speed for the whole print.
3. **During the print**, every `M106`/`M107` only moves the flap: `flap share = requested speed / blower speed`.
4. **PRINT_END** runs `_CPAP_MODE ACTIVE=0`: blower off, flap fully open to the toolhead, `M106` works normally again.
5. **Optional:** the plugin acts as Kalico **MPC's cooling fan** and reports the air that really reaches the hotend
   (blower power × flap share). MPC then models the fan correctly even though the blower speed never changes.

Example: the file's highest fan speed is 60 %, so the blower runs at 60 % for the whole print.

| Slicer asks for | Flap (toolhead share) | Air at the toolhead |
|---|---|---|
| 60 % | 100 % | 60 % |
| 30 % | 50 % | 30 % |
| 0 % (`M107`) | 0 % | none |

## Requirements

- **Kalico.** Tested at commit `fb200af0` (`v2026.10.00-90`) on a Raspberry Pi 4, Python 3.13. The plugin uses Kalico's
  `klippy/plugins/` folder and Kalico MPC. Mainline Klipper is untested.
- **A blower on `[fan]`** (the part-cooling fan), PWM controlled.
- **A hobby servo** on a free PWM-capable pin with 5 V, e.g. the BLTouch header of a BTT Octopus Pro (`GND · 5V · PB6`).
  Tested with a Miuzei MF90 (500–2500 µs = 0–180°).
- **A flap/diverter part** that splits the blower air between the toolhead and a bypass, e.g.
  [cflap-rework](https://github.com/beattune-core/cflap-rework) (see [Hardware](#hardware)).

## Hardware

The flap used with this plugin is **[cflap-rework](https://github.com/beattune-core/cflap-rework)** by beattune-core
(GPL-3.0). That repository has everything needed to build it:

- **STL files** (servo version) and **STEP files** (servo and stepper versions)
- **Hose adapters** for Mellow CPAP hoses
- **Parts list, print materials** (ABS/ASA, TPU seals) and **assembly steps**

Notes for use with this plugin:

- **CFlap supports the servo version only.** It reads a Kalico `[servo]`; the stepper version is not supported.
- **Servo:** cflap-rework lists an MG90S. This plugin was tested with a Miuzei MF90 (same 9 g size class,
  500–2500 µs = 0–180°). Whatever servo you use, find its end stops as described in
  [Configuration](#1-servo-and-flap-end-stops).
- **Feedback on fit, sealing and airflow** goes to the cflap-rework repository.

## Installation

Order matters: the plugin file must be on the printer before the config mentions `[cpap_flap]`, or Klipper won't start.

1. **Copy the plugin** to the printer:
   ```bash
   scp cpap_flap.py pi@<printer>:~/klipper/klippy/plugins/cpap_flap.py
   ```
2. **Copy `printer_cfg_flap_block.cfg`** into your config folder (e.g. upload it in Mainsail) and add to `printer.cfg`:
   ```ini
   [include printer_cfg_flap_block.cfg]
   ```
3. **Add the servo section and the `CPAP_FLAP` macro** (see [Configuration](#configuration)).
4. **Add the PRINT_START/PRINT_END lines** (see [PRINT_START and PRINT_END](#3-print_start-and-print_end)).
5. **Restart the Klipper service** (Mainsail power menu → Klipper, or `sudo systemctl restart klipper`). A plain
   `RESTART` does not load a changed plugin file, because Python keeps the old module loaded.

## Configuration

### 1. Servo and flap end stops

The flap must never be driven into its walls (a stalled servo draws its full stall current and heats up). So first find
the two end positions, then limit the servo to them.

1. Start with the servo's full range. Example for an MF90:
   ```ini
   [servo cpap_flap]
   pin: PB6
   minimum_pulse_width: 0.0005
   maximum_pulse_width: 0.0025
   maximum_servo_angle: 180
   initial_angle: 90
   ```
2. With the vane **off** the spline, restart: the servo moves to 90°. Press the vane on so it points at the middle of its
   slot.
3. Step towards each end in small steps (`SET_SERVO SERVO=cpap_flap ANGLE=88`, `86`, …) until the vane is just short of
   the wall. Note both angles.
4. Convert each end angle to a pulse width: `width = 0.0005 + angle × 0.002 / 180` (for a 500–2500 µs servo).
   Example: 35° → `0.000889`, 115° → `0.001778`.
5. Final config:
   ```ini
   [servo cpap_flap]
   pin: PB6
   minimum_pulse_width: 0.000889   # end A: ALL air to the toolhead
   maximum_pulse_width: 0.001778   # end B: NO air to the toolhead
   maximum_servo_angle: 100
   initial_angle: 0

   [gcode_macro CPAP_FLAP]
   description: Toolhead share of the CPAP air. CPAP_FLAP PERCENT=0-100 (100 = all air to toolhead)
   gcode:
       {% set pct = params.PERCENT|default(100)|float %}
       SET_SERVO SERVO=cpap_flap ANGLE={100 - pct}
   ```

**Important:** the plugin assumes the **minimum** pulse width is the "all air to the toolhead" end. If your flap works the
other way round, mount the servo or vane so that it matches. Kalico doesn't allow `maximum_pulse_width` to be smaller
than `minimum_pulse_width`.

### 2. `[cpap_flap]` options

All are optional; the defaults are shown.

```ini
[cpap_flap]
#timeout: 60        # seconds before a scan is killed (result: fail-safe 100 %)
#fan: fan           # printer object of the blower
#servo: cpap_flap   # name of the flap [servo ...]
```

The macros in `printer_cfg_flap_block.cfg` assume the blower is `[fan]` and the servo is `cpap_flap`.

### 3. PRINT_START and PRINT_END

```ini
# PRINT_START, at the very start (before homing):
    _CPAP_MODE ACTIVE=0      # safety: end a flap mode left over from an aborted print/test
    CPAP_SCAN                # highest part-fan speed of this file -> printer.cpap_flap.fan_max

# PRINT_START, at the very end (after the purge line; replaces a final M107 if you have one):
    _CPAP_MODE ACTIVE=1      # blower constant at fan_max, M106/M107 now move the flap

# PRINT_END, as the FIRST line (so it also runs on cancel/error):
    _CPAP_MODE ACTIVE=0      # blower off, flap fully open
```

- `M106`/`M107` used inside PRINT_START **before** `_CPAP_MODE ACTIVE=1` (e.g. a chamber-soak fan) still drive the real
  blower.
- Don't read `printer.cpap_flap.fan_max` inside PRINT_START itself: Klipper renders the whole macro before it runs, so it
  would still see the old value. `_CPAP_MODE` reads it when it is called.

### 4. Slicer (OrcaSlicer)

- **Fan speed-up time:** about `0.2 s`. The flap is fast; a longer value moves every cooling change too early.
- **Fan kick-start time:** `0`. A kick-start writes `M106 S255`, which would make every print's maximum 100 %.
- **Part cooling fan min PWM:** `0`.
- **Aux/exhaust fans** (`M106 P2`/`P3`) are ignored by the scan and in flap mode.

### 5. Blower `min_power`

In flap mode, the slicer's `S/255` is treated as **real blower power** and Kalico's `min_power` rescale is bypassed on
purpose: `S128` means 50 % blower-equivalent air. Outside flap mode, `M106` is the normal Kalico command and rescales
(`PWM = min_power + S/255 × (max_power − min_power)`).

### 6. Optional: CFlap as MPC's cooling fan

```ini
[extruder]
cooling_fan: cpap_flap
```

Then **re-calibrate MPC**, because the old fan table was measured on a different scale:

1. Write down your current MPC values (`SAVE_CONFIG` overwrites them).
2. Not printing, flap mode off, hotend cold: run `CPAP_MPC_CALIBRATE` (default `FAN_BREAKPOINTS=11`; other parameters are
   passed to `MPC_CALIBRATE`). It refuses immediately in flap mode or during a print.
3. When it finishes, run `SAVE_CONFIG`. If it aborts, run `_CPAP_MPC_END` (fan off, flap open).

During calibration the plugin drives the real blower and the flap. Speeds below `min_power` are reached by running the
blower at `min_power` and closing the flap partway.

> **Note:** `CPAP_MPC_CALIBRATE` and `_CPAP_MPC_END` pause and re-arm a `[delayed_gcode FAN_ON_WITH_HEATER]` loop from
> the author's config. If you don't have that loop, remove the `UPDATE_DELAYED_GCODE ID=FAN_ON_WITH_HEATER …` line from
> each of the two macros, otherwise they fail.

## Idle test

With the hotend cold, in the console:

```
CPAP_SCAN FILE="<a sliced file>.gcode"
_CPAP_MODE ACTIVE=1
M106 S128
M107
CPAP_MPC_CALIBRATE
_CPAP_MODE ACTIVE=0
```

What you should see:

1. **`CPAP_SCAN`:** a line like `CPAP_SCAN: fan_max 65.1 % (max S166, 365 part-fan M106, 0.38 s)`.
2. **`_CPAP_MODE ACTIVE=1`:** the blower runs at that speed and the flap closes.
3. **`M106` / `M107`:** the flap moves; the blower speed stays the same.
4. **`CPAP_MPC_CALIBRATE`:** refused immediately ("flap mode is active").
5. **`_CPAP_MODE ACTIVE=0`:** blower off, flap fully open.

File names with spaces must be quoted (`FILE="a b.gcode"`).

## Commands and status

| Command | What it does |
|---|---|
| `CPAP_SCAN [FILE=]` | Scan the printed file (or `FILE=`, relative to the gcodes folder) for the highest part-fan speed |
| `_CPAP_MODE ACTIVE=1\|0` | Flap mode on (blower constant at `fan_max`) / off (blower off, flap open) |
| `CPAP_FLAP PERCENT=0-100` | Move the flap: toolhead share of the air |
| `CPAP_MPC_CALIBRATE [...]` | MPC calibration with blower and flap prepared |
| `_CPAP_MPC_END` | Clean up after an aborted calibration |

Status `printer.cpap_flap.*`: `fan_max`, `max_s`, `m106_count`, `file`, `scan_time`, `valid`, `error`, `hotend_air`.

**Fail-safe:** if a scan fails (no file, read error, timeout), `fan_max` is 1.0 and a warning is printed. The scan never
aborts a print.

## Good to know

- **The blower runs at the print's highest fan speed for the whole print:** a single 100 % bridge means 100 % blower
  for the whole print.
- **The airflow model is linear** (blower power × flap share). Real blowers and flaps are not perfectly linear.
- **Hotend cooling:** with the flap at 0 % the hotend gets no air at all. If your toolhead relies on the part-cooling air
  to cool the heat break, add a minimum flap share.
- **Mainsail** shows the constant blower speed during a print, not the slicer's request. The fan slider moves the flap.
- **Kalico internals:** the plugin relies on some Kalico internals (listed in the header of `cpap_flap.py`). Re-check
  them after a Kalico update.

## Files

| File | Purpose |
|---|---|
| `cpap_flap.py` | The Kalico plugin (`CPAP_SCAN`, MPC fan adapter) |
| `printer_cfg_flap_block.cfg` | Macros: `_CPAP_MODE`, `M106`/`M107` overrides, `CPAP_MPC_CALIBRATE`, `_CPAP_MPC_END` |
| `tests/` | Tests and fixtures (see below) |

## Running the tests

```bash
python -I -B tests/test_cpap_flap.py     # scan + background process
python -I -B tests/test_mpc_adapter.py   # MPC fan adapter
python -I -B tests/sim_macros.py         # macro simulation, needs: python -m pip install jinja2==3.1.6
```

- `tests/run_wsl.sh` runs the suites on Linux (fork, like on the Pi).
- To check a full-size file, set `CPAP_BIG_GCODE=<path>`.
- The fixtures, including unmodified GPLv3 Kalico files used as a test oracle, are described in
  [tests/fixtures/README.md](tests/fixtures/README.md).

## Credits

- **Flap hardware:** [cflap-rework](https://github.com/beattune-core/cflap-rework) by beattune-core (GPL-3.0), itself
  inspired by [Flap-controlled-CPAP](https://github.com/vitals78/Flap-controlled-CPAP) by vitals78.
- **Firmware:** [Kalico](https://github.com/KalicoCrew/kalico).
