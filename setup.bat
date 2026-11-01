@echo off
REM One-time environment bootstrap for Windows.
REM Precaution: clear PYTHONPATH so any system ROS settings don't leak in.

setlocal enableextensions
set "PYTHONPATH="

set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%"
set "PROJECT_ROOT=%CD%"

where python >nul 2>nul
if errorlevel 1 (
    echo [setup.bat] Khong tim thay Python. Cai Python 3.10+ roi thu lai.
    popd
    exit /b 1
)

if not exist "%PROJECT_ROOT%\.venv\Scripts\python.exe" (
    echo [setup.bat] Tao moi truong ao tai %PROJECT_ROOT%\.venv
    python -m venv "%PROJECT_ROOT%\.venv"
)
call "%PROJECT_ROOT%\.venv\Scripts\activate.bat"
set "PYTHONPATH="
python -m pip install --upgrade pip wheel
python -m pip install -r "%PROJECT_ROOT%\requirements.txt"

if not exist "%PROJECT_ROOT%\logs"   mkdir "%PROJECT_ROOT%\logs"
if not exist "%PROJECT_ROOT%\output" mkdir "%PROJECT_ROOT%\output"
if not exist "%PROJECT_ROOT%\data"   mkdir "%PROJECT_ROOT%\data"
if not exist "%PROJECT_ROOT%\models" mkdir "%PROJECT_ROOT%\models"

echo [setup.bat] Hoan tat. Chay 'run.bat' de khoi dong pipeline mac dinh.
popd
endlocal
