#!/bin/bash
# ExArMo Lab motor tester (exarmolab.py g7-C) — macOS / Linux launcher (double-click on a Mac, or: bash run_lab.command)
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  echo "Creating virtual environment ..."
  python3 -m venv .venv || { echo "python3 not found — install from python.org or 'brew install python'"; exit 1; }
  .venv/bin/python -m pip install --upgrade pip
  .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python exarmolab.py
