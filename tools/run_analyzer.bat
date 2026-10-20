@echo off
cd /d "%~dp0"
if exist "..\gui\.venv\Scripts\python.exe" (set PY="..\gui\.venv\Scripts\python.exe") else (set PY=python)
%PY% ExArMoLogAnalyzer_v2.4.py %1
pause
