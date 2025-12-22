@echo off
echo ========================================
echo  GEN72 Real Robot Multi-View Capture
echo ========================================
echo.
echo WARNING: This will control the PHYSICAL robot arm!
echo Make sure:
echo   1. Robot is powered on and connected to 192.168.1.18
echo   2. Emergency stop is accessible
echo   3. Workspace is clear of obstacles
echo.
pause

echo Starting Dora daemon...
start /B cmd /c "dora up"

echo Waiting for daemon...
timeout /t 5 /nobreak >nul

cd config

echo Building dataflow...
dora build dataflow_gen72_real.yml

echo Starting real robot control...
dora start dataflow_gen72_real.yml
