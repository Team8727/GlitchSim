@echo off
cd /d "%~dp0"

echo Installing requests...
python -m pip install --upgrade requests >nul 2>&1

echo.
echo === EXPORTING EVERYTHING IN GLTF FORMAT ===
echo.

python onshape_batch_export.py --access_key on_8vP7VwWGW0fu7SjQkqkEy --secret_key GIAvtlymCfk9tUD0toSnCYpsaOHGzSzHPY6ooQqIVA1O2DUF --did ef0adc57c00190fb854e110f --wid 5c34bc1b6445b5d2202a5dcd --output "2025 Robot - glTF Files"

echo.
echo SUCCESS!
pause