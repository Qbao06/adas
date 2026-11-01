@echo off
REM Single command launcher for Windows.
REM Detects project root, sets up .venv if missing, installs deps, runs the
REM pipeline. Keeps the window open on failure so the user sees the message.

setlocal enableextensions enabledelayedexpansion

REM Prevent system ROS Python from leaking in (Windows-only precaution;
REM same effect on Linux is handled in run.sh).
set "PYTHONPATH="

set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%"
set "PROJECT_ROOT=%CD%"

where python >nul 2>nul
if errorlevel 1 (
    echo [run.bat] Khong tim thay Python. Cai Python 3.10+ roi thu lai.
    pause
    popd
    exit /b 1
)

if not exist "%PROJECT_ROOT%\.venv\Scripts\python.exe" (
    echo [run.bat] Tao moi truong ao tai %PROJECT_ROOT%\.venv
    python -m venv "%PROJECT_ROOT%\.venv"
    if errorlevel 1 (
        echo [run.bat] Khong tao duoc venv.
        pause
        popd
        exit /b 1
    )
)

call "%PROJECT_ROOT%\.venv\Scripts\activate.bat"

REM Clear PYTHONPATH AFTER activating venv so our venv wins.
set "PYTHONPATH="

python -m pip install --upgrade pip wheel >nul
if exist "%PROJECT_ROOT%\requirements.txt" (
    python -m pip install -r "%PROJECT_ROOT%\requirements.txt"
    if errorlevel 1 (
        echo [run.bat] Canh bao: pip install that bai. Pipeline co the van chay o che do mock.
    )
)

if not exist "%PROJECT_ROOT%\logs"   mkdir "%PROJECT_ROOT%\logs"
if not exist "%PROJECT_ROOT%\output" mkdir "%PROJECT_ROOT%\output"
if not exist "%PROJECT_ROOT%\data"   mkdir "%PROJECT_ROOT%\data"
if not exist "%PROJECT_ROOT%\models" mkdir "%PROJECT_ROOT%\models"

if not exist "%PROJECT_ROOT%\config\adas.yaml" (
    if not exist "%PROJECT_ROOT%\config\calibration.yaml" (
        echo [run.bat] Thieu file config. Hay chay setup.bat truoc.
    )
)

echo [run.bat] Khoi dong ADAS ...
python "%PROJECT_ROOT%\run.py" %*
set "STATUS=%ERRORLEVEL%"
if not "%STATUS%"=="0" (
    echo [run.bat] Pipeline loi ma %STATUS%.
    pause
)

popd
endlocal & exit /b %STATUS%
