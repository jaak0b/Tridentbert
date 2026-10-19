# Simulates the flap-mode macros the way Kalico parses/renders them.
# Run: python -I sim_macros.py
import ast
import configparser
import os
import re
import shlex
import sys

sys.dont_write_bytecode = True

try:
    import jinja2
except ImportError:
    print("sim_macros.py needs Jinja2 (Kalico pins jinja2==3.1.6):\n"
          "    python -m pip install jinja2==3.1.6")
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, "fixtures")
# trimmed verbatim snapshots of the live config (fixtures/README.md)
LIVE_PRINTER = os.path.join(FIXTURES, "printer_cfg_snapshot.cfg")
LIVE_MACRO = os.path.join(FIXTURES, "macro_cfg_snapshot.cfg")
BLOCK = os.path.join(os.path.dirname(HERE), "printer_cfg_flap_block.cfg")
print("jinja2 %s, snapshots from tests/fixtures" % (jinja2.__version__,))


def section_text(path, header):
    # copy one section verbatim from a live file
    lines = open(path, encoding="utf-8").read().split("\n")
    out, on = [], False
    for ln in lines:
        if ln.startswith("["):
            on = ln.strip() == header
        if on:
            out.append(ln)
    return "\n".join(out)


text = "\n".join([
    section_text(LIVE_PRINTER, "[gcode_macro CPAP_FLAP]"),
    section_text(LIVE_MACRO, "[gcode_macro PC_HOTEND]"),
    open(BLOCK, encoding="utf-8").read(),
])
# the delayed_gcode ID that CPAP_MPC_CALIBRATE / _CPAP_MPC_END pause and re-arm
assert section_text(LIVE_MACRO, "[delayed_gcode FAN_ON_WITH_HEATER]"), "FAN_ON_WITH_HEATER missing"

# Kalico configfile.py: RawConfigParser(strict=False, inline_comment_prefixes=(";", "#"))
cp = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(";", "#"), interpolation=None)
cp.read_string(text)
# Kalico gcode_macro.py PrinterGCodeMacro env
env = jinja2.Environment("{%", "%}", "{", "}", extensions=["jinja2.ext.do", "jinja2.ext.loopcontrols"])

MACROS = {}
for sect in cp.sections():
    if not sect.startswith("gcode_macro "):
        continue
    name = sect.split()[1].upper()
    variables = {o[len("variable_"):]: ast.literal_eval(cp.get(sect, o))
                 for o in cp.options(sect) if o.startswith("variable_")}
    MACROS[name] = {
        "tpl": env.from_string(cp.get(sect, "gcode").strip()),
        "vars": variables,
        "rename": cp.get(sect, "rename_existing", fallback=None),
        "in_script": False,
    }
assert "cpap_flap" in cp.sections()
assert "${" not in text  # Kalico SectionInterpolation would touch it


class SimCommandError(Exception):
    pass


def action_raise_error(msg):
    # gcode_macro.py _action_raise_error: raises while the template renders
    raise SimCommandError(msg)


class State:
    def __init__(self, fan_max, valid=True, minp=0.2, maxp=1.0, hotend=25.0):
        self.minp, self.maxp = minp, maxp
        self.value = 0.0
        self.power = 0.0
        self.angle = 0.0
        self.scan = {"fan_max": fan_max, "valid": valid}
        self.hotend = hotend
        self.log = []
        self.print_state = "standby"
        self.delayed = {"FAN_ON_WITH_HEATER": 5.0}  # armed
        self.mpc_calls = []
        self.mpc_start = None
        self.mpc_fail_at = None
        for m in MACROS.values():  # fresh macro variables (= Klipper restart)
            m["vars"] = dict(m["vars"])
        MACROS["_CPAP_MODE"]["vars"] = {"active": False, "blower": 0.0}

    @property
    def flap(self):  # toolhead share in %
        return 100.0 - self.angle

    def printer(self):
        settings_fan = {"min_power": self.minp, "max_power": self.maxp}
        return {
            "configfile": {"settings": {"fan": settings_fan}},
            "cpap_flap": dict(self.scan),
            "gcode_macro _CPAP_MODE": dict(MACROS["_CPAP_MODE"]["vars"]),
            "fan": {"speed": self.value * self.maxp, "value": self.value, "power": self.power},
            "toolhead": {"extruder": "extruder"},
            "extruder": {"temperature": self.hotend},
            "print_stats": {"state": self.print_state},
        }

    # Kalico fan.py _apply_speed (without kick-start)
    def fan_set(self, value):
        if value > 0:
            value = min(value, 1.0)
            self.power = value * (self.maxp - self.minp) + self.minp
        else:
            self.power = 0.0
        self.value = value


ARGS_R = re.compile("([A-Z_]+|[A-Z*])")


def is_traditional(cmd):
    try:
        float(cmd[1:])
        return cmd[0].isupper() and cmd[1].isdigit()
    except Exception:
        return False


def raw_params(command, origline):
    # Kalico gcode.py GCodeCommand.get_raw_command_parameters
    param_start = len(command)
    param_end = len(origline)
    if origline[:param_start].upper() != command:
        param_start += origline.upper().find(command)
        end = origline.rfind("*")
        if end >= 0 and origline[end + 1:].isdigit():
            param_end = end
    if origline[param_start:param_start + 1].isspace():
        param_start += 1
    return origline[param_start:param_end]


def run(st, script):
    for origline in script.split("\n"):
        origline = origline.strip()
        line = origline
        cpos = line.find(";")
        if cpos >= 0:
            line = line[:cpos]
        parts = ARGS_R.split(line.upper())
        cmd = "".join(parts[:3]).strip()
        if not cmd:
            continue
        rawparams = raw_params(cmd, origline)
        if is_traditional(cmd):
            params = {parts[i]: parts[i + 1].strip() for i in range(1, len(parts), 2)}
        else:
            s = shlex.shlex(rawparams, posix=True)
            s.whitespace_split = True
            s.commenters = "#;"
            params = {k.upper(): v for k, v in (a.split("=", 1) for a in s)}
        dispatch(st, cmd, params, rawparams)


def dispatch(st, cmd, params, rawparams):
    if cmd in MACROS:
        m = MACROS[cmd]
        assert not m["in_script"], "Macro %s called recursively" % cmd
        ctx = dict(m["vars"])
        ctx.update({"printer": st.printer(), "params": params, "rawparams": rawparams,
                    "action_raise_error": action_raise_error})
        out = m["tpl"].render(ctx)
        st.log.append("%s %s" % (cmd, rawparams))
        m["in_script"] = True
        try:
            run(st, out)
        finally:
            m["in_script"] = False
    elif cmd == "M106.1":
        st.fan_set(float(params.get("S", "255")) / 255.0)
        st.log.append("  -> M106.1 %s : fan value %.4f power %.4f" % (rawparams, st.value, st.power))
    elif cmd == "M107.1":
        st.fan_set(0.0)
        st.log.append("  -> M107.1 : fan off")
    elif cmd == "SET_SERVO":
        st.angle = max(0.0, min(100.0, float(params["ANGLE"])))
        st.log.append("  -> SET_SERVO ANGLE=%s : flap %.1f %% toolhead" % (params["ANGLE"], st.flap))
    elif cmd == "SET_GCODE_VARIABLE":
        mv = MACROS[params["MACRO"].upper()]["vars"]
        assert params["VARIABLE"] in mv, params
        mv[params["VARIABLE"]] = ast.literal_eval(params["VALUE"])
    elif cmd == "RESPOND":
        st.log.append("  // " + params.get("MSG", ""))
    elif cmd == "UPDATE_DELAYED_GCODE":
        st.delayed[params["ID"]] = float(params["DURATION"])
    elif cmd == "MPC_CALIBRATE":
        # stub of control_mpc.py with cooling_fan: fan (drives [fan] directly)
        st.mpc_calls.append(dict(params))
        st.mpc_start = (st.value, st.flap, st.delayed["FAN_ON_WITH_HEATER"])
        n = int(params.get("FAN_BREAKPOINTS", "3"))
        for idx in range(n):
            if st.mpc_fail_at == idx:
                raise SimCommandError("MPC_CALIBRATE failed: simulated")
            st.fan_set(idx / (n - 1))
        st.fan_set(0.0)
    else:
        raise RuntimeError("unknown command %s" % cmd)


FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("  [" + str(detail) + "]" if detail != "" else ""))
    if not cond:
        FAILS.append(name)


def close(a, b, tol=0.002):
    return abs(a - b) <= tol


# ---- A: normal case, fan_max 65.1 % (S166), min_power 0.2 ------------------
st = State(166 / 255.0)
run(st, "M106 S83")
check("A0 normal mode M106 S83 -> M106.1 S83 (rescaled power 0.46)", close(st.value, 83 / 255) and close(st.power, 0.2 + 0.8 * 83 / 255), "%.3f" % st.power)
run(st, "M107")
check("A0 normal mode M107 -> off", st.value == 0.0 and st.power == 0.0)
run(st, "M106")
check("A0 normal bare M106 -> value 1.0", st.value == 1.0)
run(st, "M107")
run(st, "_CPAP_MODE ACTIVE=1")
check("A1 ACTIVE=1: blower power == fan_max (0.651)", close(st.power, 166 / 255), "value %.4f power %.4f" % (st.value, st.power))
check("A1 ACTIVE=1: flap 0 %", st.flap == 0.0)
check("A1 variables", MACROS["_CPAP_MODE"]["vars"]["active"] is True and close(MACROS["_CPAP_MODE"]["vars"]["blower"], 166 / 255))
run(st, "M106 S83")
check("A2 M106 S83 -> flap 50 %, blower unchanged", close(st.flap, 50.0, 0.06) and close(st.power, 166 / 255), "%.1f" % st.flap)
run(st, "M106 S166 ; enable fan")
check("A3 M106 S166 -> flap 100 %", st.flap == 100.0)
run(st, "M106 S255")
check("A4 M106 S255 -> flap capped at 100 %", st.flap == 100.0)
run(st, "M106 S26")
check("A5 M106 S26 -> flap ~15.7 %", close(st.flap, 26 / 166 * 100, 0.06), "%.1f" % st.flap)
run(st, "M106 P2 S255")
check("A6 M106 P2 S255 ignored", close(st.flap, 26 / 166 * 100, 0.06) and close(st.power, 166 / 255))
run(st, "M106 S0")
check("A7 M106 S0 -> flap 0 %", st.flap == 0.0 and close(st.power, 166 / 255))
run(st, "M106 S100")
run(st, "M107")
check("A8 M107 -> flap 0 %, blower keeps running", st.flap == 0.0 and close(st.power, 166 / 255))
run(st, "M106")
check("A9 bare M106 -> flap 100 %", st.flap == 100.0)
st.hotend = 220.0
run(st, "PC_HOTEND")
check("A10 PC_HOTEND in flap mode (hot, fan.speed>0): no change", st.flap == 100.0 and close(st.power, 166 / 255))
run(st, "_CPAP_MODE ACTIVE=0")
check("A11 ACTIVE=0: blower off, flap 100 %, inactive", st.value == 0 and st.flap == 100.0 and MACROS["_CPAP_MODE"]["vars"]["active"] is False)
run(st, "M106 S83")
check("A12 after ACTIVE=0 M106 is normal again", close(st.value, 83 / 255) and st.flap == 100.0)
print("\n".join("      " + l for l in st.log if l.startswith("  //")))

# ---- B: file without part cooling (fan_max 0) -------------------------------
st = State(0.0)
run(st, "_CPAP_MODE ACTIVE=1")
check("B1 fan_max 0 -> blower stays off", st.value == 0.0 and st.power == 0.0)
run(st, "M106 S55")
check("B2 blower 0 -> M106 moves flap to 100 %", st.flap == 100.0 and st.power == 0.0)
st.hotend = 220.0
run(st, "PC_HOTEND")
check("B3 PC_HOTEND fires (fan.speed 0) -> only flap, blower stays OFF", st.flap == 100.0 and st.power == 0.0, st.log[-3:])

# ---- C: fan_max below min_power ---------------------------------------------
st = State(25 / 255.0)
run(st, "_CPAP_MODE ACTIVE=1")
check("C1 fan_max 9.8 % < min_power -> blower at min_power 0.2", close(st.power, 0.2, 0.0005) and st.value > 0, "value %.6f power %.5f" % (st.value, st.power))
run(st, "M106 S25")
check("C2 M106 S25 -> flap ~49 %", close(st.flap, (25 / 255) / 0.2 * 100, 0.1), "%.1f" % st.flap)

# ---- D: fail safe (no valid scan) ------------------------------------------
st = State(1.0, valid=False)
run(st, "_CPAP_MODE ACTIVE=1")
check("D1 fail safe -> blower 100 %", st.value == 1.0 and st.power == 1.0)
check("D1 message says FAIL SAFE", "FAIL SAFE" in st.log[-1], st.log[-1])
run(st, "M106 S83")
check("D2 M106 S83 -> flap 32.5 %", close(st.flap, 83 / 255 * 100, 0.06), "%.1f" % st.flap)

# ---- E: min_power not set (None in settings) --------------------------------
st = State(166 / 255.0, minp=None)
st.minp = 0.0
p = st.printer
st.printer = lambda: dict(p(), configfile={"settings": {"fan": {"min_power": None, "max_power": 1.0}}})
run(st, "_CPAP_MODE ACTIVE=1")
check("E1 min_power None -> value = fan_max", close(st.value, 166 / 255) and close(st.power, 166 / 255))

# ---- F: PRINT_END order: _CPAP_MODE ACTIVE=0 first, the old M107 later -------
st = State(166 / 255.0)
run(st, "_CPAP_MODE ACTIVE=1")
run(st, "M106 S83")
run(st, "_CPAP_MODE ACTIVE=0")
n_log = len(st.log)
run(st, "M107")
check("F1 PRINT_END: M107 after ACTIVE=0 is the plain M107.1 (harmless)",
      st.value == 0.0 and st.flap == 100.0 and any("M107.1" in l for l in st.log[n_log:]), st.log[n_log:])
st = State(166 / 255.0)
run(st, "_CPAP_MODE ACTIVE=0")
check("F2 PRINT_START safety ACTIVE=0 when flap mode is off: fan off, flap 100 %",
      st.value == 0.0 and st.flap == 100.0 and MACROS["_CPAP_MODE"]["vars"]["active"] is False)


def expect_refusal(st, line, needle):
    before = (st.value, st.flap, dict(st.delayed), len(st.mpc_calls))
    try:
        run(st, line)
        return False, "no error"
    except SimCommandError as e:
        after = (st.value, st.flap, dict(st.delayed), len(st.mpc_calls))
        return (needle in str(e) and before == after), str(e)


# ---- G: CPAP_MPC_CALIBRATE ---------------------------------------------------
st = State(166 / 255.0)
run(st, "_CPAP_MODE ACTIVE=1")
run(st, "M106 S83")
ok, msg = expect_refusal(st, "CPAP_MPC_CALIBRATE", "flap mode is active")
check("G1 refused up front in flap mode, nothing touched", ok, msg)
run(st, "_CPAP_MODE ACTIVE=0")
for state in ("printing", "paused"):
    st.print_state = state
    ok, msg = expect_refusal(st, "CPAP_MPC_CALIBRATE", "not while a print is " + state)
    check("G2 refused while %s, nothing touched" % state, ok, msg)
st.print_state = "standby"
ok, msg = expect_refusal(st, "CPAP_MPC_CALIBRATE HEATER=heater_bed", "only for HEATER=extruder")
check("G3 refused for another heater", ok, msg)

st = State(166 / 255.0)
st.hotend = 60.0
run(st, "M106 S55")  # e.g. set by the PC_HOTEND loop on a warm hotend
run(st, "CPAP_FLAP PERCENT=30")
run(st, "CPAP_MPC_CALIBRATE")
check("G4 default call: MPC_CALIBRATE HEATER=extruder FAN_BREAKPOINTS=11",
      st.mpc_calls == [{"HEATER": "extruder", "FAN_BREAKPOINTS": "11"}], st.mpc_calls)
check("G4 at MPC start: fan off, flap 100 %, FAN_ON_WITH_HEATER paused",
      st.mpc_start == (0.0, 100.0, 0.0), st.mpc_start)
check("G4 afterwards: fan off, flap 100 %, loop re-armed (5 s)",
      st.value == 0.0 and st.flap == 100.0 and st.delayed["FAN_ON_WITH_HEATER"] == 5.0)
check("G4 SAVE_CONFIG reminder", "SAVE_CONFIG" in st.log[-1], st.log[-1])

st = State(166 / 255.0)
run(st, "CPAP_MPC_CALIBRATE TARGET=210 fan_breakpoints=7 HEATER=extruder")
check("G5 parameters passed through (keys upper-cased, no duplicate)",
      st.mpc_calls == [{"HEATER": "extruder", "TARGET": "210", "FAN_BREAKPOINTS": "7"}], st.mpc_calls)

st = State(166 / 255.0)
st.mpc_fail_at = 4
try:
    run(st, "CPAP_MPC_CALIBRATE")
    check("G6 MPC failure aborts the macro", False)
except SimCommandError as e:
    check("G6 MPC failure aborts the macro", True, e)
check("G6 after the abort: fan left on, loop still paused (needs _CPAP_MPC_END)",
      st.value > 0.0 and st.delayed["FAN_ON_WITH_HEATER"] == 0.0, "value %.2f" % st.value)
run(st, "_CPAP_MPC_END")
check("G6 _CPAP_MPC_END recovers: fan off, flap 100 %, loop re-armed",
      st.value == 0.0 and st.flap == 100.0 and st.delayed["FAN_ON_WITH_HEATER"] == 5.0)

print()
print("FAILED: %s" % FAILS if FAILS else "ALL MACRO CHECKS PASSED")
sys.exit(1 if FAILS else 0)
