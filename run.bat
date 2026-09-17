@echo off
setlocal EnableExtensions EnableDelayedExpansion
chcp 65001 >nul
cd /d "%~dp0"

:: 不只检查文件存在，还验证解释器可启动；复制自旧机器的 venv 需要原位重建。
set "VENV_READY="
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -c "import sys" >nul 2>nul
    if not errorlevel 1 set "VENV_READY=1"
)
if not defined VENV_READY (
    echo [*] 正在创建虚拟环境...
    set "PYTHON_BOOTSTRAP="
    where python.exe >nul 2>nul && set "PYTHON_BOOTSTRAP=python.exe"
    if not defined PYTHON_BOOTSTRAP if exist "%USERPROFILE%\.platformio\penv\Scripts\python.exe" set "PYTHON_BOOTSTRAP=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
    if not defined PYTHON_BOOTSTRAP (
        echo [错误] 未找到 Python；请安装 Python 3，或先安装 PlatformIO Core
        pause
        exit /b 1
    )
    "!PYTHON_BOOTSTRAP!" -m venv --clear .venv
    if errorlevel 1 (
        echo [错误] 创建虚拟环境失败
        pause
        exit /b 1
    )
)

:: 激活虚拟环境
call .venv\Scripts\activate.bat

:: 安装依赖（仅首次或更新时需要）
pip install -q -r requirements.txt
if errorlevel 1 (
    echo [错误] Python 依赖安装失败
    deactivate
    exit /b 1
)

:: CI/维护用：只完成环境搭建和导入检查，不启动 Tk GUI。
if /I "%~1"=="--setup-only" (
    python -c "import serial, tkinter; print('Environment OK: pyserial', serial.VERSION)"
    set "SETUP_RESULT=!ERRORLEVEL!"
    deactivate
    exit /b !SETUP_RESULT!
)

:: 运行GUI
python pc_gui.py

deactivate
