@echo off
echo Starting Hunter SE with GEN72 Arm MuJoCo Simulation...

start /B cmd /c "dora up"
timeout /t 5 /nobreak >nul

dora build config/dataflow_hunter_arm_mujoco.yml
dora start config/dataflow_hunter_arm_mujoco.yml
