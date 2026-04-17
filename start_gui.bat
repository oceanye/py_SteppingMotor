@echo off
cd /d "%~dp0"

echo [1/3] Working dir: %CD%

if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] .venv\Scripts\python.exe not found.
    echo Please run run.bat first to create venv and install dependencies.
    pause
    exit /b 1
)

echo [2/3] Activating venv...
call ".venv\Scripts\activate.bat"
if errorlevel 1 (
    echo [ERROR] activate.bat failed.
    pause
    exit /b 1
)

echo [3/3] Launching GUI...
python pc_gui.py
set RC=%errorlevel%

if not "%RC%"=="0" (
    echo.
    echo [GUI exited with code %RC%]
    pause
)
