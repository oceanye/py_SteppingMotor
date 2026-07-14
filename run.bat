@echo off
chcp 65001 >nul
cd /d "%~dp0"

:: 如果venv不存在则创建
if not exist ".venv\Scripts\activate.bat" (
    echo [*] 正在创建虚拟环境...
    python -m venv .venv
    if errorlevel 1 (
        echo [错误] 创建虚拟环境失败，请确认已安装 Python 并加入 PATH
        pause
        exit /b 1
    )
)

:: 激活虚拟环境
call .venv\Scripts\activate.bat

:: 安装依赖（仅首次或更新时需要）
pip install -q -r requirements.txt

:: 运行GUI
python pc_gui.py

deactivate
