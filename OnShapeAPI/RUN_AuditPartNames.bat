@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "CSV_FILE=%SCRIPT_DIR%OnShape_PartNameAudit.csv"
set "PYTHON_SCRIPT=Audit_CSV_PartNames.py"  :: Change if your Python file has a different name

echo Robotics Part Naming Convention Checker
echo =======================================
echo.

echo Looking for CSV file: %CSV_FILE%
echo.

if not exist "%CSV_FILE%" (
    echo ERROR: File "OnShape_PartNameAudit.csv" not found in this folder!
    echo.
    echo Place your OnShape export CSV in the same folder as this batch file.
    echo.
    pause
    exit /b 1
)

echo Found file!

echo Installing/upgrading required Python library (pandas)...
python -m pip install --upgrade pandas >nul 2>&1
if errorlevel 1 (
    echo.
    echo WARNING: Standard install failed. Trying with --user flag...
    python -m pip install --user --upgrade pandas
)

echo.
echo Running naming convention audit...
echo =======================================
echo.

python "%SCRIPT_DIR%%PYTHON_SCRIPT%" "%CSV_FILE%"

echo.
echo =======================================
if errorlevel 1 (
    echo AUDIT SCRIPT FINISHED WITH ERRORS (see above)
) else (
    echo AUDIT COMPLETE!
    echo Non-compliant parts saved to: flagged_OnShape_PartNameAudit.csv
)
echo.

echo Press any key to close this window...
pause >nul