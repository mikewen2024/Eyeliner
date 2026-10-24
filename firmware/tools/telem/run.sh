#!/bin/zsh
# Runs the telemetry logger in its own venv (created on first use).
# usage: tools/telem/run.sh [--port PORT] [--no-plot] [--window SEC] [--duration SEC]
set -e
HERE=${0:A:h}
if [ ! -x "$HERE/.venv/bin/python" ]; then
  echo "creating venv in $HERE/.venv ..."
  python3 -m venv "$HERE/.venv"
  "$HERE/.venv/bin/pip" install -q pyserial matplotlib
fi
exec "$HERE/.venv/bin/python" "$HERE/telem_logger.py" "$@"
