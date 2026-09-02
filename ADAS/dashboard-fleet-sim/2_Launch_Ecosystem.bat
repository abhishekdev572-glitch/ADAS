@echo off
echo Initiating LaneX Digital Twin Ecosystem...

:: Navigate automatically to whatever folder this script is currently inside
cd /d "%~dp0"

:: 1. Launch Dashboard Backend (Cyan text)
start "LaneX Web Server" cmd /k "title LaneX Web Server && color 0B && python admin_dashboard.py"

:: Wait 1 second to ensure the server starts
timeout /t 1 >nul

:: 2. Launch Master Node (Green text)
start "LaneX Master Node Brain" cmd /k "title LaneX Master Node && color 0A && python master_node.py"

:: Wait 1 second to ensure the Master Node is listening
timeout /t 1 >nul

:: 3. Launch Fleet Swarm Simulator (Yellow text)
start "LaneX Fleet Swarm" cmd /k "title LaneX Swarm Hardware && color 0E && python fleet_swarm.py"

:: 4. Wait 2 seconds for everything to sync, then open Chrome to the Dashboard
timeout /t 2 >nul
start http://localhost:5000

exit