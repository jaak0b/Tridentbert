# Local tests for Plugins/CFlap/cpap_flap.py (run with: python -I test_cpap_flap.py)
import os
import random
import re
import sys
import tempfile
import time

sys.dont_write_bytecode = True  # no __pycache__ next to the plugin

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.environ.get("CPAP_ASSETS", os.path.dirname(HERE))
FIXTURES = os.path.join(HERE, "fixtures")
START = os.environ.get("CPAP_START_METHOD", "spawn")  # Windows has no fork
# trimmed real Orca file (fixtures/README.md) and its expected scan result
FIXTURE_GCODE = os.path.join(FIXTURES, "katzenfutter_v6_trimmed.gcode")
FIXTURE_EXPECT = (166.0, 25)
# optional: the full 59 MB original, e.g. CPAP_BIG_GCODE=...\big.gcode
BIG = os.environ.get("CPAP_BIG_GCODE", "")
BIG_EXPECT = (166.0, 365)
sys.path.insert(0, ASSETS)

import cpap_flap  # noqa: E402

# Kalico fb200af0 gcode.py/mathutil.py/queuelogger.py (unmodified, namespace
# package without __init__.py) as parsing oracle
ORACLE_DIR = os.path.join(FIXTURES, "kalico_fb200af0")


def jinja_do_int():
    """Jinja2's `|int` filter (how the M106 macro reads P): the installed
    jinja2 if available, else the same logic as jinja2 3.1.6 filters.do_int."""
    try:
        from jinja2.filters import do_int
        return do_int, "jinja2 %s" % (__import__("jinja2").__version__,)
    except ImportError:
        def do_int(value, default=0, base=10):
            try:
                if isinstance(value, str):
                    return int(value, base)
                return int(value)
            except (TypeError, ValueError):
                try:
                    return int(float(value))
                except (TypeError, ValueError):
                    return default
        return do_int, "inline copy of jinja2 3.1.6 do_int (jinja2 not installed)"


def child_dies(conn, path):
    # module-level so spawn can pickle it: exits without sending anything
    os._exit(3)


def child_sleeps(conn, path):
    time.sleep(30)


def child_empty_err(conn, path):
    conn.send(("", None, 0, 0.0))
    conn.close()


def child_blank_err(conn, path):
    conn.send(("  \n ", None, 0, 0.0))
    conn.close()


# ---- stubs -------------------------------------------------------------
class FakeReactor:
    def monotonic(self):
        return time.monotonic()

    def pause(self, waketime):
        d = waketime - time.monotonic()
        if d > 0:
            time.sleep(d)
        return time.monotonic()


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeVSD:
    def __init__(self, dirname, current=None):
        self.sdcard_dirname = dirname
        self.current = current

    def get_status(self, eventtime):
        return {"file_path": self.current}


class FakePrinter:
    def __init__(self, vsd):
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.vsd = vsd
        self.events = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return {"gcode": self.gcode, "virtual_sdcard": self.vsd}.get(name, default)

    def register_event_handler(self, event, cb):
        self.events[event] = cb


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default, minval=None):
        return default

    def get(self, name, default=None):
        return default


class FakeGcmd:
    def __init__(self, **params):
        self.params = params
        self.msgs = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def respond_info(self, msg):
        self.msgs.append(msg)
        print("    respond_info:", msg)


# ---- helpers -------------------------------------------------------------
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (" - " + str(detail) if detail else ""))
    if not cond:
        FAILS.append(name)


def write(dirname, name, data):
    path = os.path.join(dirname, name)
    with open(path, "wb") as f:
        f.write(data)
    return path


def make_plugin(gdir, current=None):
    printer = FakePrinter(FakeVSD(gdir, current))
    obj = cpap_flap.load_config(FakeConfig(printer))
    return printer, obj


# ---- previous scanner (regex version), only for the timing comparison ----
_OLD_RE = re.compile(rb"M106(?:[ \t]+P0)?(?:[ \t]+S(\d+(?:\.\d*)?|\.\d+))?[ \t]*(?:;|\r?\n?$)")


def old_scan_file(path):
    max_s, count, match = None, 0, _OLD_RE.match
    with open(path, "rb") as f:
        for line in f:
            if not line.startswith(b"M106"):
                continue
            m = match(line)
            if m is None:
                continue
            count += 1
            v = 255.0 if m.group(1) is None else float(m.group(1))
            if max_s is None or v > max_s:
                max_s = v
    return max_s, count


def load_oracle():
    if not os.path.isfile(os.path.join(ORACLE_DIR, "klippy", "gcode.py")):
        return None
    sys.path.insert(0, ORACLE_DIR)
    from klippy import gcode as kg  # Kalico fb200af0, unmodified
    do_int, source = jinja_do_int()
    print("    oracle: Kalico fb200af0 gcode.py from fixtures, P rule via " + source)

    class _Printer:
        def send_event(self, *args):
            pass

        def invoke_shutdown(self, msg):
            raise AssertionError("shutdown: " + msg)

    def oracle(line):
        """S that Kalico's own parser + fan.py cmd_M106 would apply for this
        line, with the M106 macro's P rule (P|int != 0 -> aux fan), else None."""
        got = {}

        class Dispatch:
            args_r = kg.GCodeDispatch.args_r
            _script_context = 0
            error = kg.CommandError
            printer = _Printer()

            def respond_info(self, *a, **k):
                pass

            def respond_raw(self, *a, **k):
                pass

            def _respond_error(self, msg):
                pass

            def cmd_default(self, gcmd):
                pass

        d = Dispatch()

        def m106(gcmd):  # fan.py PrinterFan.cmd_M106
            got["S"] = gcmd.get_float("S", 255.0, minval=0.0)
            got["P"] = gcmd.get_command_parameters().get("P")

        d.gcode_handlers = {"M106": m106}
        try:
            kg.GCodeDispatch._process_commands(d, [line], need_ack=False)
        except kg.CommandError:
            return None  # Kalico rejects the line (error, print would abort)
        if "S" not in got:
            return None  # not the M106 command
        if got["P"] is not None and do_int(got["P"]) != 0:
            return None
        return got["S"]

    return oracle


def fuzz_lines(n, seed=1):
    rnd = random.Random(seed)
    prefixes = ["", " ", "\t", "  ", "N10 ", "n5", "N7  ", "N3M106"]
    cmds = ["M106", "m106", "M106", "M106 ", "M106.0", "M1060", "M 106", "M0106", "G1", "M107", "m106", "M106.1"]
    keys = ["S", "s", "P", "p", "E", "X", "F", "SP", "N"]
    vals = ["0", "255", "166", "+200", "-5", "2e2", "1.5", "", " 200", ".5", "5.", "1.2.3", "007", "0.0", "2",
            "300", "1_0", "0x10", "(1)", "inf", "nan", "+-1", "2 ", "1e400"]
    seps = ["", " ", "  ", "\t"]
    tails = ["", "", "*45", " *12", " ; enable fan", ";M106 S255", "; x", "\r"]
    lines = []
    for _ in range(n):
        if rnd.random() < 0.08:
            alphabet = "Mm106SsPpN *;.+-eE \t2559"
            lines.append("".join(rnd.choice(alphabet) for _ in range(rnd.randint(0, 18))))
            continue
        line = rnd.choice(prefixes) + rnd.choice(cmds)
        for _ in range(rnd.randint(0, 4)):
            line += rnd.choice(seps) + rnd.choice(keys) + rnd.choice(seps[:2]) + rnd.choice(vals)
        line += rnd.choice(tails)
        lines.append(line)
    return lines


def passes_prefilter(raw):
    return any(n in raw for n in cpap_flap._NEEDLES)


def main():
    cpap_flap.START_METHOD = START
    print("start method:", START, "python", sys.version.split()[0])
    if START == "fork":
        # mimic klippy: a busy background logging thread while we fork
        import logging
        import threading

        logging.getLogger().addHandler(logging.NullHandler())
        stop = threading.Event()

        def spam():
            while not stop.is_set():
                logging.getLogger().info("bg log line")

        th = threading.Thread(target=spam, daemon=True)
        th.start()
    gdir = tempfile.mkdtemp(prefix="cpap_gcodes_")

    # ---- 1. scan_file on the trimmed real file (fixture) ----------------
    max_s, count = cpap_flap.scan_file(FIXTURE_GCODE)
    fm = cpap_flap.fan_max_from_s(max_s)
    check("fixture (max_s, count) == %s" % (FIXTURE_EXPECT,), (max_s, count) == FIXTURE_EXPECT, (max_s, count))
    check("fixture fan_max ~0.651", abs(fm - 0.651) < 0.0005, "%.4f" % fm)
    check("fixture: previous regex scanner agrees", old_scan_file(FIXTURE_GCODE) == FIXTURE_EXPECT,
          old_scan_file(FIXTURE_GCODE))

    # ---- 1b. optional: the full 59 MB file + timing (CPAP_BIG_GCODE) ------
    if BIG and os.path.isfile(BIG):
        def best_of(fn, n=3):
            best = None
            for _ in range(n):
                t = time.perf_counter()
                res = fn(BIG)
                t = time.perf_counter() - t
                best = t if best is None or t < best else best
            return best, res

        t_old, r_old = best_of(old_scan_file)
        t_new, r_new = best_of(cpap_flap.scan_file)
        check("full file: old and new scanner == %s" % (BIG_EXPECT,), r_old == r_new == BIG_EXPECT, (r_old, r_new))
        print("    timing full file (best of 3): old %.3f s, new %.3f s, ratio new/old %.2f"
              % (t_old, t_new, t_new / t_old))
    else:
        print("    SKIP full 59 MB file check (set CPAP_BIG_GCODE to the original .gcode to run it)")

    # ---- 1c. parsing exactly like Kalico ---------------------------------
    cases = [
        ("M106 S100 P0", 100.0), ("M106S200", 200.0), ("   M106 S150", 150.0),
        ("\tM106 S151", 151.0), ("m106 s200", 200.0), ("N10 M106 S200*45", 200.0),
        ("M106 S 200", 200.0), ("M106 S+200", 200.0), ("M106 S2e2", 2.0),
        ("M106 P2 S255", None), ("M106 S255 P2", None), ("M106 P0.0 S90", 90.0),
        ("M106 S-5", None), ("M106 S1.2.3", None), ("M106 S", None),
        ("M106", 255.0), ("M106 ; S100", 255.0), ("; M106 S255", None),
        ("M106.0 S255", None), ("M1060 S255", None), ("M 106 S255", None),
        ("M0106 S255", None), ("G1 X106 Y106", None), ("M106 S300", 300.0),
        ("M106 SP200", 255.0), ("M106 S77\r", 77.0), ("N3M106S9", 9.0),
    ]
    oracle = load_oracle()
    bad = []
    for line, want in cases:
        got = cpap_flap.part_fan_s(line)
        ref = oracle(line) if oracle else want
        if not (got == want == ref):
            bad.append((line, want, got, ref))
    check("%d review cases (plugin == expected == Kalico oracle)" % len(cases), not bad, bad)
    if oracle is None:
        check("oracle files present in tests/fixtures/kalico_fb200af0/klippy", False, ORACLE_DIR)
    else:
        lines = fuzz_lines(30000)
        mism = [(l, cpap_flap.part_fan_s(l), oracle(l)) for l in lines
                if cpap_flap.part_fan_s(l) != oracle(l)]
        accepted = [l for l in lines if oracle(l) is not None]
        check("fuzz: 30000 random lines, plugin == Kalico oracle", not mism,
              "%d accepted by Kalico, mismatches %s" % (len(accepted), mism[:5]))
        missed = [l for l in accepted if not passes_prefilter(l.encode("utf-8"))]
        check("fuzz: prefilter keeps every line Kalico accepts", not missed, missed[:5])
        fz = write(gdir, "fuzz.gcode", ("\n".join(lines) + "\n").encode("utf-8"))
        vals = [oracle(l) for l in lines if oracle(l) is not None]
        want = (max(vals) if vals else None, len(vals))
        check("fuzz: scan_file(file) == oracle (max, count)", cpap_flap.scan_file(fz) == want,
              (cpap_flap.scan_file(fz), want))
        small = [cpap_flap.scan_file(fz, chunk_size=c) for c in (1, 7, 64, 4096)]
        check("fuzz: chunk boundaries (chunk 1/7/64/4096 B) give the same result",
              all(r == want for r in small), small)

    # ---- 2. synthetic files ------------------------------------------
    p = write(gdir, "empty.gcode", b"")
    check("empty file -> (None, 0)", cpap_flap.scan_file(p) == (None, 0))
    check("empty file fan_max 0.0", cpap_flap.fan_max_from_s(None) == 0.0)

    p = write(gdir, "aux.gcode", b"M106 P2 S255\nM106 P2 S0\nM106 P3 S255 ; exhaust\nM106 P1 S200\n")
    check("aux-only file -> (None, 0)", cpap_flap.scan_file(p) == (None, 0), cpap_flap.scan_file(p))

    mixed = (
        b"; M106 S255 comment line\n"
        b"M106 S100 ; enable fan\n"
        b"M106 P0 S200\r\n"
        b"M106 P3 S255\n"
        b"M106 P2 S254 ; aux\n"
        b"M106.1 S255\n"
        b"M1060 S255\n"
        b"M106 S0\n"
        b"G1 X1 ; M106 S255\n"
        b"M106 S127.5\n"
        b"M107\n"
        b"M106 S166 P2\n"
    )
    p = write(gdir, "mixed.gcode", mixed)
    check("mixed/comments -> (200, 4)", cpap_flap.scan_file(p) == (200.0, 4), cpap_flap.scan_file(p))

    p = write(gdir, "bare.gcode", b"G1 X1\nM106\nM106 S10\n")
    check("bare M106 counts as S255", cpap_flap.scan_file(p) == (255.0, 2), cpap_flap.scan_file(p))

    # virtual_sdcard work_handler drops a last line without "\n" (never runs it)
    p = write(gdir, "nonl.gcode", b"M106 S50")
    check("last line without newline is ignored (like virtual_sdcard)", cpap_flap.scan_file(p) == (None, 0), cpap_flap.scan_file(p))
    p = write(gdir, "nonl2.gcode", b"M106 S40\nM106 S50")
    check("...earlier lines still count", cpap_flap.scan_file(p) == (40.0, 1), cpap_flap.scan_file(p))
    p = write(gdir, "nl.gcode", b"M106 S50\n")
    check("terminated last line counts", cpap_flap.scan_file(p) == (50.0, 1), cpap_flap.scan_file(p))

    p = write(gdir, "over.gcode", b"M106 S300\n")
    check("S > 255 clamps fan_max to 1.0", cpap_flap.fan_max_from_s(cpap_flap.scan_file(p)[0]) == 1.0)

    # ---- 3. plugin + child process (spawn) ----------------------------
    import shutil
    fx_rel = "fixture copy.gcode"  # name with a space, like the user's files
    fx_path = os.path.join(gdir, fx_rel)
    shutil.copyfile(FIXTURE_GCODE, fx_path)

    printer, obj = make_plugin(gdir)
    check("CPAP_SCAN registered", "CPAP_SCAN" in printer.gcode.commands)
    st = obj.get_status(0)
    check("initial status fail-safe 1.0", st["fan_max"] == 1.0 and not st["valid"], st)

    g = FakeGcmd(FILE=fx_rel)
    t0 = time.perf_counter()
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("process scan FILE=relative", st["valid"] and abs(st["fan_max"] - 0.651) < 0.0005 and st["m106_count"] == FIXTURE_EXPECT[1],
          "%s (%.2f s wall incl. spawn)" % ({k: st[k] for k in ("fan_max", "max_s", "m106_count", "scan_time")}, time.perf_counter() - t0))
    check("one-line summary", len(g.msgs) == 1 and "65.1 %" in g.msgs[0], g.msgs)

    g = FakeGcmd(FILE=fx_path)
    obj.cmd_CPAP_SCAN(g)
    check("process scan FILE=absolute", obj.get_status(0)["valid"] and obj.get_status(0)["m106_count"] == FIXTURE_EXPECT[1])

    printer.events["virtual_sdcard:load_file"]()
    st = obj.get_status(0)
    check("load_file event resets to 1.0", st["fan_max"] == 1.0 and not st["valid"], st["error"])

    printer.vsd.current = fx_path
    g = FakeGcmd()
    obj.cmd_CPAP_SCAN(g)
    check("default = file being printed", obj.get_status(0)["valid"] and obj.get_status(0)["file"] == fx_path)

    g = FakeGcmd(FILE="")  # CPAP_SCAN FILE=  (or FILE="")
    obj.cmd_CPAP_SCAN(g)
    check("empty FILE= -> file being printed", obj.get_status(0)["valid"] and obj.get_status(0)["file"] == fx_path, g.msgs)

    printer.vsd.current = None
    g = FakeGcmd()
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("not printing, no FILE -> fail safe 1.0 + warning", st["fan_max"] == 1.0 and not st["valid"] and "WARNING" in g.msgs[0])

    g = FakeGcmd(FILE="")
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("not printing, empty FILE= -> fail safe 1.0", st["fan_max"] == 1.0 and "no file is being printed" in st["error"], st["error"])

    for target, label in ((child_empty_err, "empty"), (child_blank_err, "blank")):
        obj.child_target = target
        g = FakeGcmd(FILE=fx_rel)
        obj.cmd_CPAP_SCAN(g)
        st = obj.get_status(0)
        check("%s error text from child -> fail safe, no IndexError" % label,
              st["fan_max"] == 1.0 and st["error"] == "scan failed (no details)", st["error"])
    obj.child_target = cpap_flap._scan_child

    g = FakeGcmd(FILE="does_not_exist.gcode")
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("missing file -> fail safe 1.0 + warning", st["fan_max"] == 1.0 and "not found" in st["error"] and "WARNING" in g.msgs[0])

    g = FakeGcmd(FILE="empty.gcode")
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("empty file via process -> 0.0 valid", st["valid"] and st["fan_max"] == 0.0 and st["max_s"] is None, g.msgs)

    # child raises (path is a directory) -> error text, fail safe
    os.mkdir(os.path.join(gdir, "adir.gcode"))
    obj.status["fan_max"] = 0.5
    g = FakeGcmd(FILE=os.path.join(gdir, "adir.gcode"))
    obj._resolve_path = lambda f: f  # bypass isfile() so the child gets a directory
    obj.cmd_CPAP_SCAN(g)
    del obj._resolve_path
    st = obj.get_status(0)
    check("child exception -> fail safe 1.0", st["fan_max"] == 1.0 and not st["valid"], st["error"])

    # child dies without sending -> no hang, fail safe
    obj.child_target = child_dies
    g = FakeGcmd(FILE=fx_rel)
    t0 = time.perf_counter()
    obj.cmd_CPAP_SCAN(g)
    st = obj.get_status(0)
    check("child dies without result -> fail safe, no hang", st["fan_max"] == 1.0 and "exit code 3" in st["error"],
          "%s (%.2f s)" % (st["error"], time.perf_counter() - t0))

    # timeout -> child killed, fail safe
    obj.child_target = child_sleeps
    obj.timeout = 2.0
    g = FakeGcmd(FILE=fx_rel)
    t0 = time.perf_counter()
    obj.cmd_CPAP_SCAN(g)
    dt = time.perf_counter() - t0
    st = obj.get_status(0)
    check("timeout -> fail safe 1.0", st["fan_max"] == 1.0 and "timed out" in st["error"], "%s (%.2f s)" % (st["error"], dt))
    import multiprocessing
    check("timeout: no child left running", not multiprocessing.active_children(), multiprocessing.active_children())
    check("busy flag cleared", obj.busy is False)

    print()
    print("FAILED: %s" % FAILS if FAILS else "ALL TESTS PASSED")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
