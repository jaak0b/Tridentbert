# CFlap test fixtures

- `kalico_fb200af0/klippy/gcode.py`, `mathutil.py`, `queuelogger.py`: unmodified copies from Kalico commit
  [fb200af0](https://github.com/KalicoCrew/kalico/tree/fb200af07259b3dbb28c174b174add7af0bbbf0a/klippy)
  (the version running on the printer), GPLv3 (license headers kept). Used **only as a test oracle** by
  `test_cpap_flap.py`, to check that `cpap_flap.part_fan_s()` parses lines exactly like Kalico. They are never installed.
- `katzenfutter_v6_trimmed.gcode`: 342 lines cut from the real OrcaSlicer 2.4.2 file
  `Katzenfutter Halter v6_ASA_13h32m.gcode` (2,362,160 lines). It keeps the header, the start with `PRINT_START`, a window
  of +/- 4 lines around the first M106 of every value (and a few repeats), the end, and the fan / custom-gcode keys of
  the config block. Omitted parts are marked `; [CFlap test fixture] ...`. The last three lines before
  `; CONFIG_BLOCK_END` are synthetic (marked) and put `M106` text into comments. Expected scan: max S166, 25 part-fan M106.
- `printer_cfg_snapshot.cfg`, `macro_cfg_snapshot.cfg`: verbatim sections of the live config (Moonraker GET,
  2026-10-19) that `sim_macros.py` needs: `[gcode_macro CPAP_FLAP]`, `[delayed_gcode FAN_ON_WITH_HEATER]` and
  `[gcode_macro PC_HOTEND]`.
