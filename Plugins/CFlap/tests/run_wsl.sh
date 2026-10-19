#!/usr/bin/env bash
# Runs the CFlap test suites under Linux (fork start method), using only files
# inside Plugins/CFlap. Writes no .pyc: -B for every run, and the syntax check
# compiles in memory only.
#   PYTHON=python3 (default) or e.g. a venv python that has jinja2==3.1.6
#   CPAP_BIG_GCODE=/path/to/full.gcode  (optional 59 MB check + timing)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PYTHON:-python3}"
export CPAP_ASSETS="$HERE"
cd /tmp || exit 1
"$PY" -I -B -c 'import sys; compile(open(sys.argv[1], encoding="utf-8").read(), sys.argv[1], "exec"); print("compile OK (Linux Python " + sys.version.split()[0] + ")")' "$HERE/cpap_flap.py"
CPAP_START_METHOD=fork "$PY" -I -B "$HERE/tests/test_cpap_flap.py" 2>&1 | grep -E '^FAIL|ALL TESTS|FAILED|start method|timing|SKIP|oracle:'
"$PY" -I -B "$HERE/tests/test_mpc_adapter.py" 2>&1 | grep -E '^FAIL|ALL ADAPTER|FAILED'
"$PY" -I -B "$HERE/tests/sim_macros.py" 2>&1 | grep -E '^FAIL|ALL MACRO|FAILED|jinja2|pip install'
