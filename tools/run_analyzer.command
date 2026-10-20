#!/bin/bash
cd "$(dirname "$0")"
PY=../gui/.venv/bin/python; [ -x "$PY" ] || PY=python3
exec "$PY" ExArMoLogAnalyzer_v2.4.py "$@"
