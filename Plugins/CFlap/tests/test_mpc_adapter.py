# Tests for the MPC cooling-fan adapter in Plugins/CFlap/cpap_flap.py
# Run: python -I test_mpc_adapter.py
import logging
import math
import os
import sys
import threading
import time

ASSETS = os.environ.get("CPAP_ASSETS", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ASSETS)
sys.dont_write_bytecode = True  # no __pycache__ next to the plugin
import cpap_flap  # noqa: E402

MIN_W, MAX_W, MAX_ANGLE = 0.000889, 0.001778, 100.0  # live [servo cpap_flap]
# live SAVE_CONFIG fan_ambient_transfer (11 breakpoints)
TABLE = [0.104181, 0.102975, 0.136172, 0.173852, 0.21571, 0.236996, 0.266251, 0.271052, 0.308746, 0.318673, 0.310365]


class CommandError(Exception):
    pass


# ---- stubs modelled on Kalico fb200af0 -------------------------------------
class FakeFan:  # fan.Fan
    def __init__(self, min_power=0.2, max_power=1.0):
        self.min_power, self.max_power = min_power, max_power
        self.last_req_value = 0.0
        self.last_pwm_value = 0.0
        self.calls = []

    def set_speed(self, value, print_time=None):
        self.calls.append((value, print_time))
        if value > 0:  # fan.py _apply_speed, no kick-start
            value = min(value, 1.0)
            self.last_pwm_value = value * (self.max_power - self.min_power) + self.min_power
        else:
            self.last_pwm_value = 0.0
        self.last_req_value = value


class FakePrinterFan:  # fan.PrinterFan
    def __init__(self, fan):
        self.fan = fan
        self.raise_status = False

    def get_status(self, eventtime):
        if self.raise_status:
            raise RuntimeError("boom")
        f = self.fan
        return {"power": f.last_pwm_value, "value": f.last_req_value,
                "speed": f.last_req_value * f.max_power, "rpm": None}


class FakeServo:  # servo.PrinterServo
    def __init__(self, attrs=True):
        self.last_value = 0.0
        if attrs:  # servo.py L14-23 attribute names
            self.min_width, self.max_width, self.max_angle = MIN_W, MAX_W, MAX_ANGLE
        self.raise_status = False

    def set_angle(self, angle):  # servo.py _get_pwm_from_angle
        angle = max(0.0, min(MAX_ANGLE, angle))
        width = MIN_W + angle * (MAX_W - MIN_W) / MAX_ANGLE
        self.last_value = width / 0.020

    def get_status(self, eventtime):
        if self.raise_status:
            raise RuntimeError("servo boom")
        return {"value": self.last_value}


class FakeGcode:
    def __init__(self, printer):
        self.printer = printer
        self.commands = {}
        self.scripts = []
        self.msgs = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run_script_from_command(self, script):
        self.scripts.append(script)
        parts = dict(p.split("=", 1) for p in script.split()[1:])
        assert script.split()[0] == "SET_SERVO", script
        servo = self.printer.objects["servo " + parts["SERVO"]]
        servo.set_angle(float(parts["ANGLE"]))

    def respond_info(self, msg):
        self.msgs.append(msg)


class FakeConfigfile:
    def __init__(self, settings):
        self.settings = settings

    def get_status(self, eventtime):
        return {"settings": self.settings}


class FakeStatusObj:
    def __init__(self, status):
        self.status = status

    def get_status(self, eventtime):
        return dict(self.status)


class FakeReactor:
    def monotonic(self):
        return time.monotonic()


class FakePrinter:
    command_error = CommandError

    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
        self.events = {}
        self.lookups = []
        self.objects["gcode"] = FakeGcode(self)

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        self.lookups.append(name)
        return self.objects.get(name, default)

    def register_event_handler(self, event, cb):
        self.events.setdefault(event, []).append(cb)

    def connect(self):
        for cb in self.events.get("klippy:connect", []):
            cb()


class FakeConfig:
    def __init__(self, printer, options=None):
        self.printer = printer
        self.options = options or {}

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default, minval=None):
        return float(self.options.get(name, default))

    def get(self, name, default=None):
        return self.options.get(name, default)


def full_printer(min_power=0.2):
    pr = FakePrinter()
    fan = FakeFan(min_power=min_power)
    pr.objects["fan"] = FakePrinterFan(fan)
    pr.objects["servo cpap_flap"] = FakeServo()
    pr.objects["configfile"] = FakeConfigfile({"servo cpap_flap": {
        "minimum_pulse_width": MIN_W, "maximum_pulse_width": MAX_W,
        "maximum_servo_angle": MAX_ANGLE, "initial_angle": 0.0, "pin": "PB6"}})
    pr.objects["gcode_macro _CPAP_MODE"] = FakeStatusObj({"active": False, "blower": 0.0})
    pr.objects["print_stats"] = FakeStatusObj({"state": "standby"})
    return pr


FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("  [" + str(detail) + "]" if detail != "" else ""))
    if not cond:
        FAILS.append(name)


def close(a, b, tol=1e-3):
    return abs(a - b) <= tol


def mpc_ambient_transfer(speed):
    # control_mpc.py temperature_update L200-215
    speed = max(0.0, min(1.0, speed))
    fb = speed * (len(TABLE) - 1)
    below, above = TABLE[math.floor(fb)], TABLE[math.ceil(fb)]
    return below if below == above else below * (1 - fb % 1.0) + (fb % 1.0) * above


def main():
    # ---- load order / lazy lookup ----------------------------------------
    pr = FakePrinter()  # [extruder] creates the plugin before [fan]/[servo] exist
    obj = cpap_flap.load_config(FakeConfig(pr))
    check("heaters.py accepts it: hasattr(obj,'fan') and .fan.set_speed",
          hasattr(obj, "fan") and hasattr(obj.fan, "set_speed"))
    check("__init__ looks up only 'gcode'", pr.lookups == ["gcode"], pr.lookups)
    check("before connect: speed 0.0, no raise", obj.fan.get_status(0) == {"speed": 0.0})
    # objects appear later, then klippy:connect
    full = full_printer()
    for k, v in full.objects.items():
        if k != "gcode":
            pr.objects[k] = v
    pr.connect()
    check("after connect: fan, servo, geometry resolved",
          obj.pfan is pr.objects["fan"] and obj.servo is pr.objects["servo cpap_flap"]
          and obj.servo_geometry == (MIN_W, MAX_W, MAX_ANGLE), obj.servo_geometry)

    # ---- get_status maths --------------------------------------------------
    fan = pr.objects["fan"].fan
    servo = pr.objects["servo cpap_flap"]
    fan.last_pwm_value = 0.651
    servo.set_angle(50.0)  # CPAP_FLAP PERCENT=50
    check("power 0.651, flap 50 % -> 0.3255", close(obj.fan.get_status(0)["speed"], 0.3255, 1e-6), obj.fan.get_status(0))
    servo.set_angle(100.0)  # PERCENT=0
    check("flap 0 % -> 0.0", close(obj.fan.get_status(0)["speed"], 0.0, 1e-9), obj.fan.get_status(0))
    servo.set_angle(0.0)  # PERCENT=100
    check("flap 100 % -> 0.651", close(obj.fan.get_status(0)["speed"], 0.651, 1e-9))
    servo.set_angle(25.0)
    check("flap 75 % -> 0.48825", close(obj.fan.get_status(0)["speed"], 0.651 * 0.75, 1e-6))
    servo.last_value = 0.0  # SET_SERVO WIDTH=0 / never commanded
    check("servo value 0 -> fallback share 1 -> 0.651", close(obj.fan.get_status(0)["speed"], 0.651, 1e-9))
    fan.last_pwm_value = 0.0
    servo.set_angle(0.0)
    check("blower off -> 0.0", obj.fan.get_status(0)["speed"] == 0.0)
    check("plugin status has hotend_air", "hotend_air" in obj.get_status(0) and obj.get_status(0)["hotend_air"] == 0.0)

    # ---- failures never raise ----------------------------------------------
    logged = []

    class H(logging.Handler):
        def emit(self, record):
            logged.append(record.getMessage())

    logging.getLogger().addHandler(H())
    fan.last_pwm_value = 0.651
    servo.raise_status = True
    vals = [obj.fan.get_status(0)["speed"] for _ in range(100)]
    check("servo.get_status raises -> share 1.0, no raise", all(close(v, 0.651, 1e-9) for v in vals))
    check("rate-limited log (1 line for 100 failures)", len([m for m in logged if "flap position" in m]) == 1, logged)
    servo.raise_status = False
    pr.objects["fan"].raise_status = True
    check("fan.get_status raises -> 0.0, no raise", obj.fan.get_status(0) == {"speed": 0.0})
    pr.objects["fan"].raise_status = False
    for bad in (None, "x", float("nan"), -0.5, 1.7):
        fan.last_pwm_value = bad
        try:
            s = obj.fan.get_status(0)["speed"]
            ok = 0.0 <= s <= 1.0
        except Exception as e:  # must never happen
            s, ok = repr(e), False
        check("bad power %r -> in 0..1, no raise" % (bad,), ok, s)
    fan.last_pwm_value = 0.651
    saved = obj.servo_geometry
    obj.servo_geometry = (MAX_W, MIN_W, 100.0)  # nonsense geometry
    check("invalid geometry -> share 1", close(obj.fan.get_status(0)["speed"], 0.651, 1e-9))
    obj.servo_geometry = saved
    obj.servo = None
    check("servo missing -> blower power only", close(obj.fan.get_status(0)["speed"], 0.651, 1e-9))
    obj.servo = servo

    # missing objects at connect
    pr2 = FakePrinter()
    obj2 = cpap_flap.load_config(FakeConfig(pr2))
    pr2.connect()
    check("no fan/servo at all: speed 0.0, no raise", obj2.fan.get_status(0) == {"speed": 0.0})
    try:
        obj2.fan.set_speed(0.5)
        check("no fan: set_speed raises command_error", False)
    except CommandError as e:
        check("no fan: set_speed raises command_error", True, e)

    # called from another thread (MCU serial thread) while values change
    errors = []

    def reader():
        for _ in range(20000):
            try:
                s = obj.fan.get_status(0)["speed"]
                assert 0.0 <= s <= 1.0
            except Exception as e:
                errors.append(e)

    th = threading.Thread(target=reader)
    th.start()
    for i in range(20000):
        servo.set_angle(i % 101)
        fan.last_pwm_value = (i % 11) / 10.0
    th.join()
    check("concurrent reads from a 2nd thread: no errors", not errors, errors[:3])

    # ---- servo geometry: object attributes first, configfile fallback -------
    def geometry_for(servo, settings, name="cpap_flap"):
        prg = FakePrinter()
        prg.objects["fan"] = FakePrinterFan(FakeFan())
        prg.objects["servo " + name] = servo
        if settings is not None:
            prg.objects["configfile"] = FakeConfigfile(settings)
        o = cpap_flap.load_config(FakeConfig(prg, {"servo": name}))
        prg.connect()
        return o

    other = {"servo cpap_flap": {"minimum_pulse_width": 0.001, "maximum_pulse_width": 0.002, "maximum_servo_angle": 180.0}}
    o = geometry_for(FakeServo(), None)
    check("geometry from servo attributes (no configfile)", o.servo_geometry == (MIN_W, MAX_W, MAX_ANGLE), o.servo_geometry)
    o = geometry_for(FakeServo(), other)
    check("servo attributes win over configfile settings", o.servo_geometry == (MIN_W, MAX_W, MAX_ANGLE), o.servo_geometry)
    lower = {"servo cpap_flap": {"minimum_pulse_width": MIN_W, "maximum_pulse_width": MAX_W, "maximum_servo_angle": MAX_ANGLE}}
    o = geometry_for(FakeServo(attrs=False), lower, name="CPAP_Flap")
    check("fallback: configfile settings with lower-cased key ('servo CPAP_Flap')", o.servo_geometry == (MIN_W, MAX_W, MAX_ANGLE), o.servo_geometry)
    bad_servo = FakeServo()
    bad_servo.min_width, bad_servo.max_width = MAX_W, MIN_W  # nonsense
    o = geometry_for(bad_servo, lower)
    check("invalid servo attributes -> configfile fallback", o.servo_geometry == (MIN_W, MAX_W, MAX_ANGLE), o.servo_geometry)
    o = geometry_for(FakeServo(attrs=False), None)
    o.pfan.fan.last_pwm_value = 0.5
    check("no geometry anywhere -> None, share 1.0", o.servo_geometry is None and o.fan.get_status(0) == {"speed": 0.5})
    try:
        o.fan.set_speed(0.3)
        check("no geometry: set_speed refuses", False)
    except CommandError as e:
        check("no geometry: set_speed refuses", "not usable" in str(e), e)

    # ---- set_speed mapping (MPC_CALIBRATE) ----------------------------------
    gcode = pr.objects["gcode"]
    expected = {  # value: (blower pwm, flap share)
        0.0: (0.0, 1.0),
        0.1: (0.2, 0.5),
        0.2: (0.2, 1.0),
        0.5: (0.5, 1.0),
        1.0: (1.0, 1.0),
    }
    for v, (pwm, share) in expected.items():
        obj.fan.set_speed(v)
        air = obj.fan.get_status(0)["speed"]
        flap = cpap_flap.toolhead_share(servo.last_value, MIN_W, MAX_W)
        check("set_speed(%.1f): pwm %.3f flap %.3f air %.4f" % (v, fan.last_pwm_value, flap, air),
              close(fan.last_pwm_value, pwm) and close(flap, share) and close(air, v))
    check("blower driven via Fan.set_speed", len(fan.calls) == len(expected) and fan.calls[1][0] == cpap_flap.MIN_FAN_REQUEST, fan.calls[:2])
    check("flap driven via SET_SERVO", gcode.scripts[-1] == "SET_SERVO SERVO=cpap_flap ANGLE=0.0000", gcode.scripts[-2:])
    check("console line per breakpoint", "blower PWM" in gcode.msgs[-1], gcode.msgs[-1])
    # 11 breakpoints like MPC_CALIBRATE FAN_BREAKPOINTS=11 (control_mpc.py L615-618)
    worst = 0.0
    for idx in range(11):
        speed = idx / 10.0
        obj.fan.set_speed(speed)
        worst = max(worst, abs(obj.fan.get_status(0)["speed"] - speed))
    obj.fan.set_speed(0.0)
    check("11 breakpoints: reported air == requested speed", worst < 0.0005, "max err %.6f" % worst)
    check("after set_speed(0.0): blower off, flap 100 %", fan.last_pwm_value == 0.0 and servo.last_value == MIN_W / 0.020)

    # min_power 0 (plain fan): no flap needed
    pr3 = full_printer(min_power=0.0)
    obj3 = cpap_flap.load_config(FakeConfig(pr3))
    pr3.connect()
    obj3.fan.set_speed(0.1)
    check("min_power 0: set_speed(0.1) -> pwm 0.1, flap 100 %",
          close(pr3.objects["fan"].fan.last_pwm_value, 0.1) and close(obj3.fan.get_status(0)["speed"], 0.1))

    # ---- refusal -------------------------------------------------------------
    n_calls, n_scripts = len(fan.calls), len(gcode.scripts)
    pr.objects["gcode_macro _CPAP_MODE"].status["active"] = True
    try:
        obj.fan.set_speed(0.5)
        check("flap mode active -> refused", False)
    except CommandError as e:
        check("flap mode active -> refused", "flap mode" in str(e), e)
    pr.objects["gcode_macro _CPAP_MODE"].status["active"] = False
    for state in ("printing", "paused"):
        pr.objects["print_stats"].status["state"] = state
        try:
            obj.fan.set_speed(0.5)
            check("print %s -> refused" % state, False)
        except CommandError as e:
            check("print %s -> refused" % state, state in str(e), e)
    check("refusals touched neither blower nor flap", len(fan.calls) == n_calls and len(gcode.scripts) == n_scripts)
    pr.objects["print_stats"].status["state"] = "complete"
    obj.fan.set_speed(0.3)
    check("state complete -> allowed", close(obj.fan.get_status(0)["speed"], 0.3))
    del pr.objects["gcode_macro _CPAP_MODE"]
    obj.fan.set_speed(0.0)
    check("no _CPAP_MODE macro -> allowed", fan.last_pwm_value == 0.0)

    # ---- what MPC makes of it (live table) -----------------------------------
    print("\nMPC ambient_transfer with the live 11-point table (W/K):")
    for label, speed in (("flap mode, blower 65.1 %, flap 0 %", 0.0),
                         ("flap mode, blower 65.1 %, flap 50 %", 0.3255),
                         ("flap mode, blower 65.1 %, flap 100 %", 0.651),
                         ("old: cooling_fan: fan (value 0.564), any flap", 0.5637)):
        print("   %-48s speed %.4f -> %.4f" % (label, speed, mpc_ambient_transfer(speed)))

    print()
    print("FAILED: %s" % FAILS if FAILS else "ALL ADAPTER TESTS PASSED")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
