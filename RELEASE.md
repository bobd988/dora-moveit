# Release Notes

## v0.2.15

### Bug Fixes

#### Fix "Start configuration is in collision" Error
The robot's zero configuration was falsely detected as in collision with environment:
- Moved ground plane from z=-0.01 to z=-0.05 to clear robot base
- Moved table further away (x=0.6) and lower (z=0.3) to avoid arm collisions
- Updated box_obstacle position to [0.4, 0.3, 0.5] with smaller size
- Applied same fixes to both `planning_scene_op.py` and `planner_ompl_with_collision_op.py`

---

## v0.2.14

### Improvements

#### Better Initial Poses for MuJoCo Demo
Updated `motion_commander.py` with more reachable target configurations:
- Home position now starts at `[0, 0, 0, -1.57, 0, 1.57, 0]` (arm centered, easy to plan from)
- Simplified pick/place sequence with smaller joint movements
- All poses verified to be within Panda joint limits

---

## v0.2.13

### Bug Fixes

#### 1. MuJoCo Joint Array Shape Mismatch (9 vs 7 joints)
MuJoCo simulator sends 9 joint values (7 arm + 2 gripper fingers), but planner expects 7:
- `motion_commander.py`: Truncate joint positions to first 7 arm joints before planning
- `trajectory_executor.py`: Truncate joint positions to first 7 arm joints for interpolation

#### 2. Repeated Obstacle Additions on Scene Sync
Scene updates were being processed redundantly on every tick:
- `planner_ompl_with_collision_op.py`: Track scene version to skip already-processed updates
- `collision_check_op.py`: Track scene version to skip already-processed updates

---

## v0.2.12

### Bug Fixes

#### 1. Unicode Encoding Errors (Windows GBK codec)
Replaced all emoji characters with ASCII text for Windows compatibility:
- `ik_op.py`: Replaced checkmark/cross emojis with `SUCCESS`/`FAILED`
- `planner_ompl_with_collision_op.py`: Replaced checkmark/cross emojis with `SUCCESS`/`FAILED`
- `collision_check_op.py`: Replaced status emojis with `COLLISION`/`CLEAR`
- `demo_node.py`: Replaced emojis with `[OK]`/`[FAIL]`/`[COLLISION]`/`[CLEAR]`
- `motion_commander.py`: Removed all emoji characters
- `collision_lib.py`: Replaced checkmark emoji with `[OK]`

#### 2. Scene Update Parsing Error ('object' key missing)
Fixed scene update handling in `collision_check_op.py` and `planner_ompl_with_collision_op.py`:
- Now supports both full scene broadcasts (`{"world_objects": [...]}`) and single object commands (`{"action": "add", "object": {...}}`)
- Full scene sync clears and rebuilds all obstacles from `planning_scene_op.py` broadcasts
- Single object commands continue to work for incremental updates
