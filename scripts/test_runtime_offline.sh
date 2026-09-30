#!/bin/sh
# 仅限已经审计的 Runtime 测试；逐文件运行，绝不发现旧实验测试。
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
python_bin=${PYTHON:-python}
export PYTHONPATH="$repo_root/src:$repo_root/tests${PYTHONPATH:+:$PYTHONPATH}"
cd "$repo_root"

run_file() {
    printf '\n[OFFLINE] %s\n' "$1"
    "$python_bin" -B -S "tests/$1" -v
}

run_file test_public_surface.py
run_file test_runtime_cli.py
run_file test_environment.py
run_file test_workflows.py
run_file test_replay.py
run_file test_teaching.py
run_file test_drag.py
run_file test_recording.py
run_file test_motion.py
run_file test_waypoints.py
run_file test_readonly_entry.py
run_file test_runtime_snapshot.py
run_file test_runtime_acquisition.py
