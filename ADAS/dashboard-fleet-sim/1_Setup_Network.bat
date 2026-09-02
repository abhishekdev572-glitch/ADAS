@echo off
color 0C
echo ===================================================
echo LaneX Ecosystem Setup (Requires Run as Administrator)
echo ===================================================

:: 1. Check for Admin privileges
net session >nul 2>&1
if %errorLevel% == 0 (
    echo [OK] Administrative privileges confirmed.
) else (
    echo [ERROR] You must right-click this file and select "Run as administrator"!
    pause
    exit
)

echo.
echo [1/3] Installing Python Dependencies...
python -m pip install paho-mqtt flask

echo.
echo [2/3] Terminating default Windows Mosquitto Service...
net stop mosquitto >nul 2>&1
echo [OK] Ports cleared.

echo.
echo [3/3] Launching LaneX MQTT Broker...
echo ---------------------------------------------------
echo DO NOT CLOSE THIS WINDOW DURING THE PRESENTATION!
echo ---------------------------------------------------
"C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\mosquitto.conf" -v

pause