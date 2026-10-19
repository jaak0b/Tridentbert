# CPAP flap mode: find the highest part-fan speed of a sliced file, and act
# as MPC's cooling fan (Kalico plugin)
#
# Install: copy to ~/klipper/klippy/plugins/cpap_flap.py and add
#
#   [cpap_flap]
#   #timeout: 60         # seconds before the scan process is killed
#   #fan: fan            # printer object of the blower
#   #servo: cpap_flap    # name of the flap [servo ...]
#
# then restart the Klipper SERVICE (Mainsail power menu -> Klipper, or
# `sudo systemctl restart klipper`). A plain RESTART also picks up a plugin
# that was never imported before, but after UPDATING this file only a
# service restart loads the new code: Kalico re-creates the Printer on
# RESTART/FIRMWARE_RESTART but Python keeps the old module in sys.modules.
#
# What it does:
#   CPAP_SCAN [FILE=<path>]
#     Scans a G-code file for part-fan M106 commands and stores the highest
#     S as printer.cpap_flap.fan_max (0.0-1.0). Lines are parsed exactly like
#     Kalico's gcode.py does (case, spaces, N<n> line numbers, *checksums,
#     ';' comments, P/S in any order); a missing S means 255, S values Kalico
#     rejects (unparsable, negative) are skipped, and aux fans (P present and
#     not 0) are ignored. Like virtual_sdcard, a last line without a trailing
#     newline is never run, so it is not counted either.
#     Default file = the one virtual_sdcard is printing
#     (also for an empty FILE=); FILE= is relative to the gcodes folder (or
#     absolute) for idle tests. Names with spaces must be quoted, because
#     Kalico parses parameters with shlex: CPAP_SCAN FILE="a b.gcode".
#     The scan runs in a forked child process, so klippy is never blocked
#     (same pattern as Kalico's shaper_calibrate.background_process_exec).
#   Fail safe: no file / read error / timeout / dead child -> fan_max = 1.0
#   plus a console warning. The command never raises, so it never aborts
#   a print. No part-fan M106 in the file -> fan_max = 0.0.
#   Loading a new file in virtual_sdcard resets fan_max to 1.0 until the
#   next scan, so a stale result of an older file is never used.
#
#   MPC cooling fan: `[extruder] cooling_fan: cpap_flap`
#     heaters.py accepts any object with .fan.set_speed as cooling fan. This
#     plugin's .fan reports the air that actually reaches the hotend:
#       speed = blower PWM power ([fan] status "power") * flap toolhead share
#     (share from the [servo] pulse width: min width = ANGLE 0 = 100 %).
#     MPC_CALIBRATE drives it through set_speed(): >= min_power -> blower at
#     that PWM, flap 100 %; below min_power -> blower at min_power, flap
#     = value / min_power; 0 -> blower off, flap 100 %. Refused (error) while
#     flap mode (_CPAP_MODE) is active or a print is running/paused.
#     After switching `cooling_fan` to cpap_flap, re-run the MPC calibration
#     with the CPAP_MPC_CALIBRATE macro (printer_cfg_flap_block.cfg): the
#     old fan_ambient_transfer table was measured on a different scale.
#
# Status (printer.cpap_flap.*): fan_max, max_s, m106_count, file,
#   scan_time, valid, error, hotend_air
#
# NOTE: reads Kalico internals
#   virtual_sdcard.VirtualSD.sdcard_dirname (gcodes folder for FILE=)
#   event "virtual_sdcard:load_file"
#   queuelogger.clear_bg_logging (child process)
#   fan.Fan.set_speed / .min_power / .max_power (via [fan] object .fan)
#   servo.PrinterServo.min_width / .max_width / .max_angle (fallback:
#   configfile settings), pwm value = pulse width / 0.020 s
#   (servo.py SERVO_SIGNAL_PERIOD)
#   gcode.py args_r / _process_commands parsing rules (copied for the scan)
# Re-check these names after a Kalico update if the plugin stops working.

import logging
import math
import multiprocessing
import os
import re
import time
import traceback

# Kalico itself calls multiprocessing.set_start_method("fork") in
# printer.main(); asking for the fork context explicitly keeps this plugin
# independent of that global (Python 3.14 changes the Linux default to
# forkserver). The child target is a module-level function, so it would
# also be picklable under spawn/forkserver.
START_METHOD = "fork"

# Kalico gcode.py GCodeDispatch.args_r (fb200af0): splits an upper-cased
# line into letter keys and their values
_ARGS_R = re.compile("([A-Z_]+|[A-Z*])")


# servo.py: pwm value = pulse width / SERVO_SIGNAL_PERIOD (20 ms)
SERVO_SIGNAL_PERIOD = 0.020
FLAP_MODE_MACRO = "gcode_macro _CPAP_MODE"
# fan request used for "blower on at min_power" (any value > 0 gives
# power = min_power + value * (max_power - min_power) in fan.py)
MIN_FAN_REQUEST = 0.0001
LOG_INTERVAL = 60.0


class ScanError(Exception):
    pass


def _clamp01(x):
    x = float(x)
    if math.isnan(x):
        return 0.0
    return max(0.0, min(1.0, x))


def toolhead_share(servo_value, min_width, max_width):
    """Flap toolhead share 0..1 from the servo pwm value.

    min_width (ANGLE 0) = all air to the toolhead. A value of 0 (no pulse,
    e.g. SET_SERVO WIDTH=0 or never commanded) or an invalid geometry gives
    1.0."""
    if not servo_value or not max_width > min_width:
        return 1.0
    width = servo_value * SERVO_SIGNAL_PERIOD
    return _clamp01(1.0 - (width - min_width) / (max_width - min_width))


def blower_request(value, min_power, max_power):
    """For MPC_CALIBRATE: (fan request, flap toolhead share) so that
    blower PWM power * share == value (as far as the blower allows)."""
    v = _clamp01(value)
    if v <= 0.0:
        return 0.0, 1.0
    span = max_power - min_power
    if span > 0.0:
        pwm = min(max(v, min_power), max_power)
        req = max((pwm - min_power) / span, MIN_FAN_REQUEST)
        actual = min_power + req * span
    else:
        req, actual = 1.0, max_power
    share = 1.0 if actual <= 0.0 else min(1.0, v / actual)
    return min(req, 1.0), share


class MpcFanAdapter:
    """The object MPC sees as `cooling_fan` (`[extruder] cooling_fan:
    cpap_flap` -> heaters.py stores this plugin's .fan).

    control_mpc.py calls get_status(read_time)["speed"] on every temperature
    reading - from the MCU serial thread, so this must be cheap, thread-safe
    (only reads status) and must never raise - and set_speed(speed) during
    MPC_CALIBRATE (inside the gcode command)."""

    def __init__(self, owner):
        self.owner = owner
        self._last_log = {}

    def _log_limited(self, key, msg):
        now = time.monotonic()
        if now - self._last_log.get(key, -LOG_INTERVAL) >= LOG_INTERVAL:
            self._last_log[key] = now
            logging.warning("cpap_flap: %s", msg, exc_info=True)

    def air(self, eventtime):
        """Air at the hotend 0..1 = blower PWM power * flap toolhead share."""
        owner = self.owner
        pfan = owner.pfan
        if pfan is None:
            return 0.0
        try:
            power = _clamp01(pfan.get_status(eventtime)["power"])
        except Exception:
            self._log_limited("power", "blower power unavailable -> 0")
            return 0.0
        share = 1.0
        servo, geometry = owner.servo, owner.servo_geometry
        if servo is not None and geometry is not None:
            try:
                share = toolhead_share(
                    servo.get_status(eventtime)["value"],
                    geometry[0],
                    geometry[1],
                )
            except Exception:
                self._log_limited("share", "flap position unavailable -> 100 %")
                share = 1.0
        return _clamp01(power * share)

    def get_status(self, eventtime):
        try:
            return {"speed": self.air(eventtime)}
        except Exception:
            return {"speed": 0.0}

    def set_speed(self, value, print_time=None):
        self.owner.drive_for_mpc(value, print_time)


def parse_gcode_line(text):
    """(cmd, params) exactly as Kalico fb200af0 gcode.py _process_commands
    builds them: strip, cut at ';', upper-case, args_r split, optional
    leading N<line number>. A '*checksum' becomes its own '*' key."""
    line = text.strip()
    cpos = line.find(";")
    if cpos >= 0:
        line = line[:cpos]
    parts = _ARGS_R.split(line.upper())
    if "".join(parts[:2]) == "N":
        cmd = "".join(parts[3:5]).strip()
    else:
        cmd = "".join(parts[:3]).strip()
    params = {parts[i]: parts[i + 1].strip() for i in range(1, len(parts), 2)}
    return cmd, params


def _safe_float(value):
    # Kalico mathutil.safe_float (used by gcmd.get_float)
    f = float(value)
    if math.isnan(f) or math.isinf(f):
        raise ValueError(value)
    return f


def _macro_int(value):
    # Jinja2 `|int` (do_int), as the M106 macro reads P: "0.0" -> 0, junk -> 0
    try:
        return int(value, 10)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError, OverflowError):
            return 0


def part_fan_s(text):
    """S value of a part-fan M106 line as Kalico would apply it, else None.

    None for: any other command (`M106.0`, `M1060`, `M 106` ...), aux fans
    (P present and not 0, in any position - the M106 macro ignores them in
    flap mode) and S values Kalico rejects with an error (unparsable or
    < 0, fan.py get_float("S", 255.0, minval=0.0)). Missing S -> 255."""
    cmd, params = parse_gcode_line(text)
    if cmd != "M106":
        return None
    p = params.get("P")
    if p is not None and _macro_int(p) != 0:
        return None
    s = params.get("S")
    if s is None:
        return 255.0
    try:
        value = _safe_float(s)
    except ValueError:
        return None
    if value < 0.0:
        return None
    return value


# Kalico's command can only be M106 if the line contains "M106" or "m106"
# (cmd = 'M' + '106...' of the args_r split of the upper-cased line)
_NEEDLES = (b"M106", b"m106")


def scan_file(path, chunk_size=1 << 20):
    """Return (max_s, count) of the part-fan M106 commands in a G-code file.

    max_s is None when the file has no part-fan M106. A bare `M106`
    counts as S255 (Kalico fan.py default). Lines are split on "\\n" like
    virtual_sdcard does; its work_handler never runs a last line without
    a trailing "\\n", so that one is ignored here too.
    Fast path: the file is searched in big blocks for the (rare) needles and
    only those lines are parsed - no Python loop over every line."""
    max_s = None
    count = 0
    tail = b""
    with open(path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            data = tail + chunk if tail else chunk
            cut = data.rfind(b"\n") + 1  # complete lines only
            tail = data[cut:]
            if not cut:
                continue
            spans = set()
            for needle in _NEEDLES:
                pos = data.find(needle, 0, cut)
                while pos >= 0:
                    start = data.rfind(b"\n", 0, pos) + 1
                    end = data.find(b"\n", pos)  # found: data[cut - 1] is "\n"
                    spans.add((start, end))
                    pos = data.find(needle, end, cut)
            for start, end in spans:
                value = part_fan_s(data[start:end].decode("utf-8", "replace"))
                if value is None:
                    continue
                count += 1
                if max_s is None or value > max_s:
                    max_s = value
    return max_s, count


def fan_max_from_s(max_s):
    if max_s is None:
        return 0.0
    return max(0.0, min(1.0, max_s / 255.0))


def _clear_bg_logging():
    # The forked child must not log through klippy's queue handler (its
    # writer thread does not exist in the child) - same as shaper_calibrate.
    try:
        from .. import queuelogger  # Kalico package layout
    except ImportError:
        try:
            import queuelogger  # legacy layout
        except ImportError:
            return
    queuelogger.clear_bg_logging()


def _scan_child(conn, path):
    # Runs in the child process. Always tries to send exactly one message:
    # (error_text or None, max_s, count, seconds)
    try:
        _clear_bg_logging()
        t0 = time.monotonic()
        max_s, count = scan_file(path)
        msg = (None, max_s, count, time.monotonic() - t0)
    except Exception:
        msg = (traceback.format_exc(limit=2), None, 0, 0.0)
    try:
        conn.send(msg)
    finally:
        conn.close()


class CpapFlap:
    cmd_CPAP_SCAN_help = (
        "Find the highest part-fan speed of a G-code file -> "
        "printer.cpap_flap.fan_max. [FILE=<path>] (default: file being printed)"
    )

    def __init__(self, config):
        # May be created early by [extruder] (cooling_fan: cpap_flap ->
        # load_object), so no lookups of [fan]/[servo]/macros here.
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.timeout = config.getfloat("timeout", 60.0, minval=1.0)
        self.fan_name = config.get("fan", "fan")
        self.servo_name = config.get("servo", "cpap_flap")
        self.child_target = _scan_child
        self.busy = False
        self.status = {}
        self._set_failsafe("", "not scanned yet")
        # MPC cooling-fan adapter (heaters.py needs <object>.fan.set_speed)
        self.fan = MpcFanAdapter(self)
        self.pfan = None  # [fan] printer object (status "power")
        self.real_fan = None  # its fan.Fan (set_speed, min/max_power)
        self.servo = None
        self.servo_geometry = None  # (min_width, max_width, max_angle)
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "virtual_sdcard:load_file", self._handle_load_file
        )
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "CPAP_SCAN", self.cmd_CPAP_SCAN, desc=self.cmd_CPAP_SCAN_help
        )

    def _handle_connect(self):
        self.pfan = self.printer.lookup_object(self.fan_name, None)
        self.real_fan = getattr(self.pfan, "fan", None)
        if self.pfan is None or not hasattr(self.real_fan, "set_speed"):
            logging.warning("cpap_flap: fan object '%s' not found", self.fan_name)
            self.pfan = self.real_fan = None
        section = "servo " + self.servo_name
        self.servo = self.printer.lookup_object(section, None)
        self.servo_geometry = None
        if self.servo is None:
            logging.warning("cpap_flap: [%s] not found", section)
            return
        self.servo_geometry = self._servo_geometry(section)
        if self.servo_geometry is None:
            logging.warning("cpap_flap: no pulse widths for [%s]", section)

    @staticmethod
    def _valid_geometry(min_w, max_w, max_angle):
        geometry = (float(min_w), float(max_w), float(max_angle))
        if not (geometry[1] > geometry[0] >= 0.0 and geometry[2] > 0.0):
            raise ValueError(geometry)
        return geometry

    def _servo_geometry(self, section):
        # 1) the servo object itself (servo.py PrinterServo.min_width,
        #    .max_width, .max_angle - exactly what it uses for ANGLE=)
        servo = self.servo
        try:
            return self._valid_geometry(
                servo.min_width, servo.max_width, servo.max_angle
            )
        except Exception:
            pass
        # 2) configfile settings (keys are lower-cased by configfile.py)
        try:
            cfg = self.printer.lookup_object("configfile")
            all_settings = cfg.get_status(self.reactor.monotonic())["settings"]
            settings = all_settings.get(section.lower(), {})
            return self._valid_geometry(
                settings["minimum_pulse_width"],
                settings["maximum_pulse_width"],
                settings["maximum_servo_angle"],
            )
        except Exception:
            return None

    def _handle_load_file(self):
        self._set_failsafe("", "new file loaded, not scanned yet")

    # MPC_CALIBRATE support (MpcFanAdapter.set_speed)
    def check_mpc_drive_allowed(self):
        eventtime = self.reactor.monotonic()
        macro = self.printer.lookup_object(FLAP_MODE_MACRO, None)
        if macro is not None and macro.get_status(eventtime).get("active"):
            raise self.printer.command_error(
                "cpap_flap: flap mode is active - run _CPAP_MODE ACTIVE=0"
                " before MPC_CALIBRATE"
            )
        print_stats = self.printer.lookup_object("print_stats", None)
        if print_stats is not None:
            state = print_stats.get_status(eventtime).get("state")
            if state in ("printing", "paused"):
                raise self.printer.command_error(
                    "cpap_flap: a print is %s - MPC may not drive the"
                    " blower/flap now" % (state,)
                )

    def drive_for_mpc(self, value, print_time=None):
        self.check_mpc_drive_allowed()
        fan = self.real_fan
        if fan is None:
            raise self.printer.command_error(
                "cpap_flap: blower '%s' not found" % (self.fan_name,)
            )
        if self.servo is None or self.servo_geometry is None:
            raise self.printer.command_error(
                "cpap_flap: [servo %s] not usable" % (self.servo_name,)
            )
        min_power = float(getattr(fan, "min_power", 0.0) or 0.0)
        max_power = float(getattr(fan, "max_power", 1.0) or 1.0)
        req, share = blower_request(value, min_power, max_power)
        angle = (1.0 - share) * self.servo_geometry[2]
        # public command path; MPC calls set_speed() from inside the
        # MPC_CALIBRATE command, so run_script_from_command is valid here
        self.gcode.run_script_from_command(
            "SET_SERVO SERVO=%s ANGLE=%.4f" % (self.servo_name, angle)
        )
        fan.set_speed(req, print_time)
        pwm = 0.0 if req <= 0.0 else min_power + req * (max_power - min_power)
        self.gcode.respond_info(
            "cpap_flap: MPC fan %.0f %% -> blower PWM %.1f %%, flap %.1f %%"
            " toolhead" % (_clamp01(value) * 100.0, pwm * 100.0, share * 100.0)
        )

    def _set_failsafe(self, path, reason, scan_time=0.0):
        self.status = {
            "fan_max": 1.0,
            "max_s": None,
            "m106_count": 0,
            "file": path,
            "scan_time": scan_time,
            "valid": False,
            "error": reason,
        }

    # Helpers
    def _resolve_path(self, fname):
        vsd = self.printer.lookup_object("virtual_sdcard", None)
        if fname is None:
            path = None
            if vsd is not None:
                path = vsd.get_status(self.reactor.monotonic())["file_path"]
            if not path:
                raise ScanError("no file is being printed (use FILE=)")
            return path
        fname = fname.strip()
        path = os.path.expanduser(fname)
        if not (os.path.isabs(path) and os.path.isfile(path)):
            gcode_dir = getattr(vsd, "sdcard_dirname", None)
            if gcode_dir is None:
                gcode_dir = os.path.expanduser("~/printer_data/gcodes")
            path = os.path.join(gcode_dir, fname.lstrip("/"))
        if not os.path.isfile(path):
            raise ScanError("file not found: %s" % (path,))
        return path

    def _wait_exit(self, proc, seconds):
        # poll via the reactor instead of a blocking join()
        eventtime = self.reactor.monotonic()
        end = eventtime + seconds
        while proc.is_alive() and eventtime < end:
            eventtime = self.reactor.pause(eventtime + 0.05)

    def _reap(self, proc, grace):
        # let the child exit on its own for `grace` seconds, then kill it
        self._wait_exit(proc, grace)
        if proc.is_alive():
            proc.kill()
            self._wait_exit(proc, 2.0)
        if not proc.is_alive():
            proc.join(0)  # removes the finished child from multiprocessing
        else:
            logging.warning("CPAP_SCAN: scan process %s did not exit", proc.pid)

    def _run_scan(self, path):
        ctx = multiprocessing.get_context(START_METHOD)
        reader, writer = ctx.Pipe(duplex=False)
        proc = ctx.Process(
            target=self.child_target, args=(writer, path), name="cpap_scan"
        )
        proc.daemon = True
        started = False
        grace = 0.0
        try:
            proc.start()
            started = True
            # keep only the read end here: EOF once the child has exited
            writer.close()
            eventtime = self.reactor.monotonic()
            deadline = eventtime + self.timeout
            while True:
                # poll() before recv(): a child that dies without sending
                # gives EOF (EOFError; OSError on Windows) instead of
                # hanging klippy
                alive = proc.is_alive()
                try:
                    ready = reader.poll()
                except OSError:
                    ready = True
                if ready:
                    try:
                        result = reader.recv()
                    except (EOFError, OSError):
                        self._wait_exit(proc, 0.5)
                        raise ScanError(
                            "scan process ended without a result"
                            " (exit code %s)" % (proc.exitcode,)
                        )
                    grace = 1.0
                    return result
                if not alive:
                    raise ScanError(
                        "scan process ended without a result (exit code %s)"
                        % (proc.exitcode,)
                    )
                if eventtime > deadline:
                    raise ScanError(
                        "scan timed out after %.0f s" % (self.timeout,)
                    )
                eventtime = self.reactor.pause(eventtime + 0.05)
        finally:
            if started:
                self._reap(proc, grace)
            writer.close()
            reader.close()

    def _fail(self, gcmd, path, reason, scan_time=0.0):
        self._set_failsafe(path, reason, scan_time)
        logging.warning("CPAP_SCAN failed (%s): %s", path, reason)
        gcmd.respond_info(
            "CPAP_SCAN WARNING: %s -> fail safe fan_max = 100 %%" % (reason,)
        )

    # G-Code command
    def cmd_CPAP_SCAN(self, gcmd):
        if self.busy:
            gcmd.respond_info("CPAP_SCAN: a scan is already running")
            return
        # empty FILE= (or FILE="") means "the file being printed"
        fname = gcmd.get("FILE", None) or None
        try:
            path = self._resolve_path(fname)
        except ScanError as e:
            self._fail(gcmd, fname or "", str(e))
            return
        self.busy = True
        t0 = self.reactor.monotonic()
        try:
            err, max_s, count, _child_time = self._run_scan(path)
        except ScanError as e:
            self._fail(gcmd, path, str(e), self.reactor.monotonic() - t0)
            return
        except Exception as e:
            logging.exception("CPAP_SCAN")
            self._fail(gcmd, path, "internal error: %s" % (e,))
            return
        finally:
            self.busy = False
        scan_time = self.reactor.monotonic() - t0
        if err is not None:
            lines = str(err).strip().splitlines()
            reason = lines[-1] if lines else "scan failed (no details)"
            self._fail(gcmd, path, reason, scan_time)
            return
        fan_max = fan_max_from_s(max_s)
        self.status = {
            "fan_max": fan_max,
            "max_s": max_s,
            "m106_count": count,
            "file": path,
            "scan_time": scan_time,
            "valid": True,
            "error": "",
        }
        name = os.path.basename(path)
        if max_s is None:
            gcmd.respond_info(
                "CPAP_SCAN: no part-fan M106 -> fan_max 0 %% (%.2f s) %s"
                % (scan_time, name)
            )
        else:
            gcmd.respond_info(
                "CPAP_SCAN: fan_max %.1f %% (max S%g, %d part-fan M106, %.2f s)"
                " %s" % (fan_max * 100.0, max_s, count, scan_time, name)
            )

    def get_status(self, eventtime):
        status = dict(self.status)
        status["hotend_air"] = self.fan.air(eventtime)
        return status


def load_config(config):
    return CpapFlap(config)
