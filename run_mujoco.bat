@echo off
echo Starting GEN72 MuJoCo Simulation...

start /B cmd /c "dora up"
timeout /t 5 /nobreak >nul

dora build config/dataflow_gen72_mujoco.yml
dora start config/dataflow_gen72_mujoco.yml
