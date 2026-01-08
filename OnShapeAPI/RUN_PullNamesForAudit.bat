@echo off
cd /d "%~dp0"

echo Installing requests...
python -m pip install --upgrade requests >nul 2>&1

echo.
echo === PULLING ALL NAMES ===
echo.

python onshape_pull_names.py --access_key on_8vP7VwWGW0fu7SjQkqkEy --secret_key GIAvtlymCfk9tUD0toSnCYpsaOHGzSzHPY6ooQqIVA1O2DUF --did ef0adc57c00190fb854e110f --wid 5c34bc1b6445b5d2202a5dcd --csv

echo.
echo SUCCESS!
pause