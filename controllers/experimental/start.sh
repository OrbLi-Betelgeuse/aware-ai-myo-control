#!/bin/sh
# Local no-Myo demonstration. Override AWARE_RUNTIME on another computer.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
runtime_dir=${AWARE_RUNTIME:-/Users/betelgeuseorb/Documents/Codex/2026-10-22/new-chat/jackal-runtime}
cd "$repo_dir"
case "${1:-}" in
  bridge)
    exec "$runtime_dir/bin/jackal-bridge" --host 127.0.0.1 --source ws --no-lsl --serve-site --site-dir ./jackal-site --quiet
    ;;
  controller)
    exec "$runtime_dir/bin/python" controllers/experimental/3.2a/simulate_3_2a.py
    ;;
  keyboard)
    exec "$runtime_dir/bin/python" controllers/experimental/keyboard-demo/simulate_3_2aTest.py
    ;;
  *)
    printf 'Usage: sh controllers/experimental/start.sh bridge|controller|keyboard\n'
    exit 2
    ;;
esac
