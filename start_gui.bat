@echo off
cd /d "%~dp0"

if not exist ".venv\Scripts\pythonw.exe" (
    echo [ERROR] .venv\Scripts\pythonw.exe not found.
    echo Please run run.bat first to create venv and install dependencies.
    pause
    exit /b 1
)

rem Launch without a console window. This removes the CMD box entirely,
rem which sidesteps two known issues: (1) wheel-scroll in the console box
rem did nothing, (2) resizing the console box after a move killed the app.
rem Runtime logs go to the GUI log pane and logs\gui_<date>.log; crash
rem tracebacks are captured to logs\gui_stderr.log by pc_gui.py itself.
rem Use start_gui_console.bat when you need live tracebacks for debugging.
start "" ".venv\Scripts\pythonw.exe" "pc_gui.py"
