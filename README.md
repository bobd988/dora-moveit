# Dora-MoveIt: Mini Motion Planning Framework

A lightweight MoveIt-like motion planning framework built on Dora-rs. This example demonstrates how to build modular robotics systems using Dora's dataflow architecture.

## 🎯 Overview

Dora-MoveIt implements the core components of a motion planning pipeline:

```
┌─────────────────────────────────────────────────────────────────┐
│                        Dora-MoveIt Architecture                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   motion_commander / demo_node (Controller)                     │
│       │                                                          │
│       ├──► planning_scene_op (Scene Manager)                    │
│       │         │                                                │
│       │         ├──► Manages world objects (obstacles, tables)  │
│       │         ├──► Tracks robot state                         │
│       │         └──► Broadcasts scene updates                   │
│       │                                                          │
│       ├──► planner_ompl_op (Motion Planner)                     │
│       │         │                                                │
│       │         ├──► RRT / RRT-Connect algorithms               │
│       │         └──► Uses collision_lib for validity checking   │
│       │                                                          │
│       ├──► ik_op (Inverse Kinematics)                           │
│       │         │                                                │
│       │         └──► Pose → Joint conversion                    │
│       │                                                          │
│       └──► collision_check_op (Collision Checker)               │
│                 │                                                │
│                 └──► Validates configurations                   │
│                                                                  │
│   collision_lib.py (Shared Library)                             │
│       └──► Core collision primitives (sphere, box, cylinder)   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## ✅ Key Features

- **🔧 IK Operator** (`ik_op.py`): Pose → joints conversion with pluggable solver
- **💥 Collision Library** (`collision_lib.py`): Reusable collision detection functions
- **🔍 Collision Check Operator** (`collision_check_op.py`): Collision checking as a service
- **🗺️ OMPL Planner** (`planner_ompl_with_collision_op.py`): RRT/RRT-Connect with embedded collision
- **🎬 Planning Scene** (`planning_scene_op.py`): Central scene manager (like MoveIt's PlanningScene)
- **🤖 MuJoCo Integration** (`dataflow_mujoco.yml`): Test with 3D robot simulation

## 🚀 Quick Start

### Installation

```bash
cd examples/dora-moveit
pip install -r requirements.txt
```

## 🧪 Testing Options

### Option 1: MuJoCo Integration (⭐ Recommended - 3D Visualization)

Test with the Panda robot in MuJoCo simulator:

```bash
# Install dora-mujoco node
pip install -e ../../node-hub/dora-mujoco

# Run with MuJoCo
dora build dataflow_mujoco.yml
dora start dataflow_mujoco.yml
```

**What you'll see**: MuJoCo window with Panda robot executing pick-and-place motions with collision avoidance.

**MuJoCo Demo Flow**:
1. 🎬 MuJoCo window opens with Panda robot
2. 📦 Obstacle (box) added to the scene
3. 🗺️ Planner generates collision-free paths
4. 🤖 Robot executes pick-and-place sequence

### Option 2: Basic Demo (Terminal Only)

```bash
# Build and start the dataflow
dora build dataflow.yml
dora start dataflow.yml
```

**Expected Output**:
```
============================================================
       Dora-MoveIt Demo - Mini Motion Planning Framework
============================================================

=== Dora-MoveIt Demo Node ===
Initial configuration: [0.0, -0.785, 0.0]...
Demo will run through all components automatically

[Demo Step 1] Setting up planning scene
  Sent robot state: [0.0, -0.785, 0.0]...

[Demo Step 2] Adding obstacle to scene
  Added obstacle: test_obstacle

[Demo Step 3] Testing IK solver
  Requested IK for pose: [0.5, 0.0, 0.5]
  ✅ IK succeeded with error 0.000234

[Demo Step 4] Planning motion to goal
  Requested motion plan: [0.0, -0.785]... → [0.0, 0.2]...
  ✅ Planning succeeded in 0.234s

[Demo Step 5] Executing planned trajectory
  Received trajectory with 12 waypoints
  Executing waypoint 1/12
  ...
  Trajectory execution complete!

[Demo Step 6] Validating final configuration
  ✅ No collision (min distance: 0.0523m)

✅ Demo complete! All Dora-MoveIt components tested.
```

### Option 3: Rerun Visualization

```bash
# Install rerun
pip install rerun-sdk

# Add rerun node to dataflow for real-time visualization
# (See examples/rerun-viewer for reference)
```

### Option 4: ROS2/RViz (If ROS2 Installed)

```bash
# Use cxx-ros2-dataflow pattern
# Publish joint states to /joint_states topic
# View in RViz with robot_description URDF
```

## 📁 File Structure

```
dora-moveit/
├── dataflow.yml                       # Basic demo (terminal)
├── dataflow_mujoco.yml                # MuJoCo integration ⭐
│
├── collision_lib.py                   # Core collision library (shared)
├── ik_op.py                           # Inverse Kinematics operator
├── collision_check_op.py              # Collision checking operator
├── planner_ompl_with_collision_op.py  # OMPL motion planner
├── planning_scene_op.py               # Scene manager
│
├── demo_node.py                       # Basic demo controller
├── motion_commander.py                # MuJoCo test controller ⭐
├── trajectory_executor.py             # Trajectory execution ⭐
│
├── requirements.txt                   # Python dependencies
└── README.md                          # This file
```

## 🔌 Component Details

### collision_lib.py

Core collision detection library with:
- Primitive collision functions (sphere-sphere, sphere-box, box-box, sphere-cylinder)
- `CollisionChecker` class for robot collision checking
- Factory functions for creating collision objects
- Self-collision and environment collision detection

```python
from collision_lib import CollisionChecker, create_sphere, create_box

checker = CollisionChecker()
checker.add_environment_object(create_box("table", [0.5, 0, 0.4], [0.6, 0.8, 0.02]))

is_valid, result = checker.is_state_valid(link_transforms)
```

### ik_op.py

Inverse Kinematics operator:
- **Input**: `ik_request` (6D pose: x,y,z,r,p,y or 7D: x,y,z,qw,qx,qy,qz)
- **Output**: `ik_solution` (joint positions), `ik_status` (success/error)
- Numerical IK using damped least squares (pluggable for other solvers)

### collision_check_op.py

Collision checking service:
- **Input**: `check_request` (joint positions), `scene_update` (obstacles)
- **Output**: `collision_result` (collision status and details)
- Maintains scene state and responds to collision queries

### planner_ompl_with_collision_op.py

OMPL-like motion planner:
- **Input**: `plan_request` (start/goal configurations)
- **Output**: `trajectory` (waypoints), `plan_status` (result)
- Supports RRT, RRT-Connect algorithms
- `is_state_valid()` callback uses collision_lib

### planning_scene_op.py

Central scene manager (like MoveIt's PlanningScene):
- Manages world objects (obstacles, tables)
- Tracks robot state
- Handles attached objects (pick/place)
- Broadcasts scene updates to all operators

### trajectory_executor.py (MuJoCo Integration)

Trajectory execution node:
- Receives planned trajectories from planner
- Interpolates between waypoints
- Sends joint commands to MuJoCo

### motion_commander.py (MuJoCo Integration)

High-level motion commander:
- Sends planning requests
- Manages pick-and-place sequences
- Coordinates scene and robot state

## 🎮 Usage Examples

### Add an Obstacle

```python
command = {
    "action": "add",
    "object": {
        "name": "obstacle1",
        "type": "sphere",
        "position": [0.4, 0.2, 0.6],
        "dimensions": [0.1]
    }
}
node.send_output("scene_command", json.dumps(command).encode())
```

### Request Motion Plan

```python
plan_request = {
    "start": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "goal": [0.5, 0.2, 0.0, -1.5, 0.0, 1.7, 0.785],
    "planner": "rrt_connect",
    "max_time": 5.0
}
node.send_output("plan_request", json.dumps(plan_request).encode())
```

### Request IK Solution

```python
target_pose = [0.5, 0.0, 0.5, 180.0, 0.0, 90.0]  # x,y,z,roll,pitch,yaw
node.send_output("ik_request", pa.array(target_pose, type=pa.float32()))
```

## 🔄 Comparison with MoveIt

| Feature | MoveIt | Dora-MoveIt |
|---------|--------|-------------|
| Architecture | ROS-based | Dora dataflow |
| Planning Scene | `PlanningSceneInterface` | `planning_scene_op.py` |
| Motion Planning | OMPL integration | Pure Python OMPL-like |
| IK | KDL/IKFast/etc | Numerical solver |
| Collision | FCL/Bullet | Custom geometric |
| Communication | ROS topics/services | Dora channels |
| Simulation | Gazebo/RViz | MuJoCo |

## 🔧 Extending

### Add a Custom IK Solver

Modify `ik_op.py`:

```python
class MyIKSolver:
    def solve(self, request: IKRequest) -> IKResult:
        # Your IK implementation (PyKDL, ikfast, etc.)
        pass

# Use in IKOperator
self.solver = MyIKSolver()
```

### Add a New Planner

Modify `planner_ompl_with_collision_op.py`:

```python
def plan_prm(self, request: PlanRequest) -> PlanResult:
    # PRM implementation
    pass
```

### Add New Collision Primitives

Extend `collision_lib.py`:

```python
@staticmethod
def mesh_mesh_collision(mesh1, mesh2, margin=0.0):
    # Mesh collision implementation
    pass
```

### Connect to Different Simulator

Create new dataflow (e.g., `dataflow_pybullet.yml`):

```yaml
nodes:
  - id: pybullet_sim
    path: pybullet_node.py
    inputs:
      control_input: trajectory_executor/joint_commands
    outputs:
      - joint_positions
```

## 🎯 Design Principles

1. **Modularity**: Each operator is independent and reusable
2. **Dora Native**: Uses Dora's dataflow for communication
3. **Collision First**: `collision_lib.py` is shared across operators
4. **MoveIt Patterns**: Follows MoveIt's architectural patterns
5. **Pluggable**: Easy to swap IK solvers, planners, collision engines
6. **Simulator Agnostic**: Can work with MuJoCo, PyBullet, ROS/Gazebo

## 📚 References

- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/)
- [OMPL Library](https://ompl.kavrakilab.org/)
- [Dora-rs Documentation](https://github.com/dora-rs/dora)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)

## 🤝 Contributing

This is a demonstration project. For production use, consider:
- Using FCL or Bullet for collision detection
- Integrating with URDF robot models
- Using proper FK from robot kinematics (pinocchio, PyKDL)
- Adding trajectory time parameterization (TOPP-RA)
- Integrating with real robot hardware drivers
