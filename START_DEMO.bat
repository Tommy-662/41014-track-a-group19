@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Set up the project environment first:
    echo py -3.11 -m venv .venv
    echo .venv\Scripts\python.exe -m pip install -r requirements-lock.txt
    pause
    exit /b 1
)
".venv\Scripts\python.exe" "run_sim.py" --camera-preview --output results/local_gui
if errorlevel 1 pause
