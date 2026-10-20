@echo off
rem ExArMo Lab motor tester (exarmolab.py g7-C) — Windows launcher (makes gui/.venv on first run).
cd /d "%~dp0"
where py >nul 2>nul && set PY=py -3 || set PY=python
if not exist ".venv\Scripts\python.exe" (
  echo Creating virtual environment ...
  %PY% -m venv .venv || goto :nopy
  ".venv\Scripts\python.exe" -m pip install --upgrade pip
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt
)
".venv\Scripts\python.exe" exarmolab.py
pause
exit /b
:nopy
echo Python 3 was not found. Install it from https://www.python.org/downloads/ (tick "Add python.exe to PATH") and run again.
pause
