# GEN72 Multi-View Capture System

A complete robotic arm control system for industrial pipeline inspection using the GEN72 7-DOF robotic arm. Built on Dora-rs dataflow architecture with MuJoCo simulation and real robot support.

## Overview

This system enables automated multi-viewpoint photography for pipeline inspection tasks. The robot arm moves to predefined viewpoints, captures images, and returns to home position - all with collision-free motion planning.

```
┌─────────────────────────────────────────────────────────────────┐
│                    GEN72 System Architecture                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   multi_view_capture_node.py (Workflow Controller)             │
│       │                                                          │
│       ├──► ik_op.py (TracIK Solver)                            │
│       │         └──► Cartesian pose → Joint angles             │
│       │                                                          │
│       ├──► planner_ompl_with_collision_op.py (RRT-Connect)     │
│       │         ├──► Collision-free path planning              │
│       │         └──► Point cloud + geometric collision         │
│       │                                                          │
│       ├──► trajectory_executor.py (Interpolation)              │
│       │         └──► Smooth waypoint interpolation             │
│       │                                                          │
│       └──► Robot Control Node                                   │
│             ├──► MuJoCo Simulator (main.py)                    │
│             └──► Real GEN72 Robot (gen72_robot_node.py)        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Key Features

- **Dual Mode Operation**: MuJoCo simulation + Real GEN72 robot support
- **Advanced IK**: TracIK solver with ~95% success rate
- **Collision Detection**:
  - Geometric collision (spheres, boxes, cylinders)
  - 3D LiDAR point cloud integration (ready, disabled by default)
  - Realman SDK collision detection framework
- **Multi-View Capture**: Automated 3-viewpoint photography workflow
- **Smooth Motion**: Trajectory interpolation with configurable speed
- **HOLD Logic**: Prevents control drift after motion completion

## Quick Start

### Prerequisites

```bash
# Python environment
conda create -n dora-moveit python=3.9
conda activate dora-moveit

# Install dependencies
pip install -r requirements.txt

# Install Dora
pip install dora-rs

# Install MuJoCo (for simulation)
pip install mujoco
pip install -e dora-mujoco/
```

### Running Simulation

```bash
# Start MuJoCo simulation
./run_mujoco.bat

# Or manually:
dora up
dora build dataflow_gen72_mujoco.yml
dora start dataflow_gen72_mujoco.yml
```

**What you'll see**:
1. MuJoCo window opens with GEN72 robot at HOME position
2. System plans collision-free paths to 3 viewpoints
3. Robot captures images at each viewpoint
4. Returns to HOME and enters idle state

### Running Real Robot

**IMPORTANT**: Ensure robot is powered on, workspace is clear, and emergency stop is accessible.

```bash
# Start real robot control
./run_real_robot.bat

# Or manually:
dora up
dora build dataflow_gen72_real.yml
dora start dataflow_gen72_real.yml
```

## System Configuration

### GEN72 Robot Parameters

- **DOF**: 7 joints
- **HOME Position**: `[0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0]` (radians)
- **IP Address**: 192.168.1.19:8080
- **Control Mode**: Position control via Realman SDK

### Capture Viewpoints

Defined in `multi_view_capture_node.py`:

```python
targets = [
    CaptureTarget("view1", [0.4, 0.2, 0.5]),   # Right viewpoint
    CaptureTarget("view2", [0.4, -0.2, 0.5]),  # Left viewpoint
    CaptureTarget("view3", [0.5, 0.0, 0.4])    # Center viewpoint
]
```

### Control Frequencies

| Mode | Frequency | Interpolation Speed |
|------|-----------|---------------------|
| MuJoCo Simulation | 20 Hz (50ms) | 0.05 |
| Real Robot | 5 Hz (200ms) | 0.05 |

## File Structure

```
dora-moveit/
├── run_mujoco.bat                    # Launch MuJoCo simulation
├── run_real_robot.bat                # Launch real robot control
│
├── dataflow_gen72_mujoco.yml         # MuJoCo dataflow config
├── dataflow_gen72_real.yml           # Real robot dataflow config
│
├── multi_view_capture_node.py        # Main workflow controller
├── trajectory_executor.py            # Trajectory interpolation
├── ik_op.py                          # TracIK inverse kinematics
├── planner_ompl_with_collision_op.py # RRT-Connect motion planner
├── planning_scene_op.py              # Scene management
│
├── gen72_robot_node.py               # Real robot control (Realman SDK)
├── robot_config.py                   # GEN72 parameters & collision geometry
│
├── rm_robot_interface.py             # Realman SDK interface
├── rm_ctypes_wrap.py                 # Realman SDK C wrapper
├── api_c.dll                         # Realman SDK library
│
├── requirements.txt                  # Python dependencies
├── README.md                         # This file
└── RELEASE.md                        # Version history
```

## Component Details

### multi_view_capture_node.py

Workflow controller that orchestrates the entire capture sequence:

**Inputs**:
- `joint_positions`: Current robot state
- `ik_solution`, `ik_status`: IK solver results
- `trajectory`, `plan_status`: Motion planner results
- `execution_status`: Trajectory execution feedback

**Outputs**:
- `ik_request`: Target poses for IK
- `plan_request`: Motion planning requests
- `scene_command`: Scene updates

**State Machine**:
1. `idle` → Request IK for viewpoint
2. `waiting_for_ik` → Request motion plan
3. `waiting_for_planning` → Wait for trajectory
4. `waiting_for_execution` → Monitor execution
5. `capturing` → Capture image
6. Repeat for all viewpoints
7. `returning_home` → Return to HOME
8. `idle` → Complete

### trajectory_executor.py

Interpolates between waypoints for smooth motion:

**Key Parameters**:
- `interpolation_speed`: 0.05 (5% progress per tick)
- `tick_rate`: 50ms (MuJoCo) / 200ms (Real robot)

**Features**:
- Linear interpolation between waypoints
- HOLD mode: Returns current joint state when idle
- Prevents control drift after trajectory completion

### ik_op.py

TracIK-based inverse kinematics solver:

**Solver**: TracIK (advanced, ~95% success rate)
**Fallback**: Numerical IK (damped least squares)

**Input**: 6D pose `[x, y, z, roll, pitch, yaw]`
**Output**: 7D joint configuration

### planner_ompl_with_collision_op.py

RRT-Connect motion planner with collision detection:

**Algorithm**: RRT-Connect (bidirectional RRT)
**Collision Margin**: 10mm safety buffer
**Max Planning Time**: 5 seconds

**Collision Detection**:
- Self-collision checking (adjacent links skipped)
- Environment collision (obstacles, ground plane)
- Point cloud collision (framework ready)

### gen72_robot_node.py

Real robot control via Realman SDK:

**Connection**:
- IP: 192.168.1.19:8080
- Thread mode: Triple-threaded (mode=2)
- Level: 3

**Unit Conversion**:
- System uses radians internally
- SDK uses degrees
- Automatic conversion: `np.deg2rad()` / `np.rad2deg()`

**Control**:
- Command: `rm_movej(joints_deg, speed=10%, r=0, connect=0, block=1)`
- Update rate: 5 Hz (200ms tick)

### robot_config.py

GEN72 robot configuration:

**Parameters**:
- Joint limits, HOME position, URDF path
- Collision geometry (spheres for each link)
- Link dimensions and offsets

**Collision Spheres**:
```python
link_spheres = {
    "link1": [([0, 0, 0.1], 0.08)],
    "link2": [([0, 0, 0.15], 0.07)],
    ...
}
```

## Critical Fixes Applied

### 1. MuJoCo Control Drift (FIXED)

**Problem**: Joint1 continued rotating after workflow completion.

**Root Cause**: MuJoCo's PD controller used stale target positions, causing continuous error accumulation.

**Solution**: Implemented timeout-based HOLD logic in `main.py:127-137`:
```python
if now - self.last_cmd_time > self.cmd_timeout:
    self.data.ctrl[:self.num_joints] = current_q  # Force HOLD
```

### 2. Trajectory Executor Drift (FIXED)

**Problem**: Executor returned stale waypoint positions when idle.

**Solution**: Modified `trajectory_executor.py:step()` to return current joint state:
```python
if not self.is_executing:
    return self.current_joints.copy()  # Not last_command
```

### 3. Real Robot API Integration (FIXED)

**Problem**: Connection failures and incorrect API usage.

**Solution**:
- Use `rm_thread_mode_e(2)` for thread mode
- Check `handle.id == -1` for connection errors
- Parse tuple return values: `result[1].joint`
- Add degree/radian conversion

## Known Issues & Limitations

### Real Robot Behavior

**Issue**: Real robot returns to HOME between each viewpoint, while MuJoCo moves continuously.

**Root Cause**: `self.current_joints` not updating fast enough on real robot due to:
- Lower update frequency (5 Hz vs event-driven)
- SDK communication delays
- Motion not complete when planning next move

**Impact**: Each motion plans from HOME instead of previous viewpoint.

**Workaround**: Acceptable for current use case (safety-first approach).

### Point Cloud Collision

**Status**: Framework implemented but disabled by default.

**Reason**: Point cloud processing handled by other team members.

**Enable**: Set `use_point_cloud=True` in planner configuration.

## Performance Metrics

| Metric | MuJoCo | Real Robot |
|--------|--------|------------|
| IK Success Rate | ~95% | ~95% |
| Planning Time | 0.05-0.15s | 0.05-0.15s |
| Motion Speed | 1.0 waypoint/s | 0.25 waypoint/s |
| Control Frequency | 20 Hz | 5 Hz |
| Typical Workflow Time | ~30s | ~60s |

## Troubleshooting

### MuJoCo Issues

**Problem**: "Model not found" error
```bash
# Solution: Set MODEL_NAME environment variable
export MODEL_NAME="path/to/GEN72_with_actuators.xml"
```

**Problem**: Joint1 still rotating after completion
```bash
# Solution: Check HOLD logic timeout
# In main.py: self.cmd_timeout = 0.2  # Increase if needed
```

### Real Robot Issues

**Problem**: Connection failed
```bash
# Check:
1. Robot powered on
2. Network connection (ping 192.168.1.19)
3. Firewall settings
4. SDK DLL in correct path
```

**Problem**: Robot moves jerkily
```bash
# Solution: Adjust control frequency
# In dataflow_gen72_real.yml:
tick: dora/timer/millis/200  # Increase for smoother motion
```

**Problem**: "Error reading joints: 'tuple' object has no attribute 'joint'"
```bash
# Solution: Check SDK return value parsing
# In gen72_robot_node.py:67-71
result = self.robot.rm_get_current_arm_state()
joint_data = result[1]  # Tuple unpacking
```

## Safety Guidelines

### Before Running Real Robot

1. **Power Check**: Ensure robot is powered and initialized
2. **Workspace**: Clear all obstacles from robot workspace
3. **Emergency Stop**: Verify emergency stop button is accessible
4. **Limits**: Confirm joint limits are properly configured
5. **Speed**: Start with low speed (10%) for testing

### Emergency Procedures

- **Immediate Stop**: Press emergency stop button
- **Software Stop**: Press `Ctrl+C` in terminal
- **Power Off**: Use main power switch if needed

### Collision Avoidance

- System uses 10mm safety margin
- Ground plane at z=-0.05m
- Self-collision checking enabled
- Adjacent links (±1) skipped for flexibility

## Development

### Adding New Viewpoints

Edit `multi_view_capture_node.py`:

```python
self.targets = [
    CaptureTarget("view1", [0.4, 0.2, 0.5]),
    CaptureTarget("view4", [0.3, 0.3, 0.6]),  # Add new viewpoint
]
```

### Adjusting Motion Speed

**Simulation** (`dataflow_gen72_mujoco.yml`):
```yaml
tick: dora/timer/millis/50  # Decrease for faster
```

**Real Robot** (`dataflow_gen72_real.yml`):
```yaml
tick: dora/timer/millis/200  # Decrease for faster (min: 50ms)
```

**Interpolation** (`trajectory_executor.py`):
```python
self.interpolation_speed = 0.05  # Increase for faster (max: 0.2)
```

**Real Robot Speed** (`gen72_robot_node.py`):
```python
self.robot.rm_movej(joint_deg, 10, ...)  # Increase 10% (max: 100%)
```

### Enabling Point Cloud Collision

1. Ensure point cloud data is available
2. Modify `planner_ompl_with_collision_op.py`:
```python
self.use_point_cloud = True
```
3. Connect point cloud input in dataflow

## References

- [Dora-rs Documentation](https://github.com/dora-rs/dora)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [TracIK Paper](https://ieeexplore.ieee.org/document/7363472)
- [RRT-Connect Algorithm](http://msl.cs.illinois.edu/~lavalle/papers/LavKuf01.pdf)
- [Realman Robotics](https://www.realman-robotics.com/)

## Version History

See [RELEASE.md](RELEASE.md) for detailed version history and changelog.

## License

This project is for industrial pipeline inspection research and development.

## Contact

For technical support or questions about the GEN72 system, please contact the development team.
