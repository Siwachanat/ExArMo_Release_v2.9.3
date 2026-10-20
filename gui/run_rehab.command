#!/bin/bash
# ExArMo Rehab v2.9.3 (therapy modes + games) — macOS / Linux launcher. Shares .venv with run_gui.command.
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  echo "Creating virtual environment ..."
  python3 -m venv .venv || { echo "python3 not found — install from python.org"; exit 1; }
  .venv/bin/python -m pip install --upgrade pip
  .venv/bin/python -m pip install -r requirements.txt
fi
.venv/bin/python -c "import tkinter" 2>/dev/null || { echo "This Python has no tkinter. Use the python.org installer, or: brew install python-tk"; exit 1; }
exec .venv/bin/python ExArMoRehab_v2.9.3.py
