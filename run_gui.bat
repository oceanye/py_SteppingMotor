@echo off
setlocal EnableExtensions DisableDelayedExpansion

rem Standalone GUI launcher. No installs, activation or persistent env changes.
rem Prefer a complete project venv; otherwise use this PC's Tk-enabled Python.
rem The bundled pyserial is pure Python and can be reused by system Python.
rem Use: run_gui.bat --check   to validate without starting the controller.

if not exist "%~dp0pc_gui.py" goto missing_project
pushd "%~dp0"
if errorlevel 1 goto missing_project

set "PYTHONPATH=%~dp0.venv\Lib\site-packages;%PYTHONPATH%"
set "GUI_PYTHON="
set "GUI_PYTHONW="
call :probe_python "%~dp0.venv\Scripts\python.exe"
if defined GUI_PYTHON goto ready
call :probe_python "C:\Python313\python.exe"
if defined GUI_PYTHON goto ready

echo [ERROR] No usable Python with Tk and pyserial was found.
echo Checked the project .venv and C:\Python313.
echo Nothing was installed or changed. Check your Python/Tk/pyserial setup.
goto failed

:ready
echo [OK] Python: "%GUI_PYTHON%"
if /i "%~1"=="--check" (
    echo [OK] Tk and pyserial are available. GUI was not started.
    popd
    exit /b 0
)

rem pc_gui.py retains the existing single-instance guard and crash logging.
start "" "%GUI_PYTHONW%" -B "%~dp0pc_gui.py"
if errorlevel 1 (
    echo [ERROR] Could not start the GUI process.
    goto failed
)
echo [OK] Launch requested. Startup errors: logs\gui_stderr.log
popd
exit /b 0

:probe_python
if not exist "%~1" exit /b 1
if not exist "%~dp1pythonw.exe" exit /b 1
"%~1" -B -c "import tkinter as tk, serial; root = tk.Tk(); root.withdraw(); root.update_idletasks(); root.destroy()" >nul 2>&1
if errorlevel 1 exit /b 1
set "GUI_PYTHON=%~1"
set "GUI_PYTHONW=%~dp1pythonw.exe"
exit /b 0

:failed
popd
if /i "%~1"=="--check" exit /b 1
pause
exit /b 1

:missing_project
echo [ERROR] Cannot access the project folder or pc_gui.py is missing.
echo Keep this launcher in the same folder as pc_gui.py.
if /i "%~1"=="--check" exit /b 1
pause
exit /b 1
