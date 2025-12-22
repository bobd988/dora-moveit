# GEN72 项目结构说明

## 目录组织

项目已按照数据流环节重新组织,每个模块职责清晰:

```
dora-moveit/
├── workflow/                    # 工作流控制层
│   ├── multi_view_capture_node.py   # 多视角拍照主控制器
│   ├── motion_commander.py          # 运动命令接口
│   └── demo_node.py                 # 演示节点
│
├── ik_solver/                   # 逆运动学求解
│   ├── ik_op.py                     # IK Dora节点
│   └── advanced_ik_solver.py        # TracIK求解器实现
│
├── motion_planner/              # 运动规划
│   ├── planner_ompl_with_collision_op.py  # RRT-Connect规划器
│   └── planning_scene_op.py         # 场景管理器
│
├── trajectory_execution/        # 轨迹执行
│   └── trajectory_executor.py       # 轨迹插值执行器
│
├── robot_control/               # 机械臂控制
│   ├── gen72_robot_node.py          # GEN72实体控制节点
│   ├── rm_robot_interface.py        # Realman SDK接口
│   ├── rm_ctypes_wrap.py            # C库封装
│   └── api_c.dll                    # Realman SDK动态库
│
├── collision_detection/         # 碰撞检测
│   ├── collision_lib.py             # 几何碰撞检测库
│   ├── collision_check_op.py        # 碰撞检测节点
│   └── pointcloud_collision.py      # 点云碰撞(框架)
│
├── config/                      # 配置文件
│   ├── robot_config.py              # GEN72机械臂参数
│   ├── dataflow_gen72_mujoco.yml    # MuJoCo仿真配置
│   ├── dataflow_gen72_real.yml      # 实体机械臂配置
│   ├── dataflow.yml                 # 通用配置
│   ├── GEN72_base.xml               # MuJoCo模型
│   └── scene_with_obstacles.xml     # 带障碍物场景
│
├── utils/                       # 工具脚本
│   ├── visualize_scene.py           # 场景可视化
│   └── create_scene_with_obstacles.py  # 场景创建
│
├── run_mujoco.bat               # 启动MuJoCo仿真
├── run_real_robot.bat           # 启动实体机械臂
├── README.md                    # 项目说明
└── RELEASE.md                   # 版本历史
```

## 数据流路径

### MuJoCo仿真模式
```
workflow/multi_view_capture_node.py (主控)
    ↓ ik_request
ik_solver/ik_op.py (IK求解)
    ↓ ik_solution
workflow/multi_view_capture_node.py
    ↓ plan_request
motion_planner/planner_ompl_with_collision_op.py (规划)
    ↓ trajectory
trajectory_execution/trajectory_executor.py (执行)
    ↓ joint_commands
dora-mujoco (MuJoCo仿真)
```

### 实体机械臂模式
```
workflow/multi_view_capture_node.py (主控)
    ↓ ik_request
ik_solver/ik_op.py (IK求解)
    ↓ ik_solution
workflow/multi_view_capture_node.py
    ↓ plan_request
motion_planner/planner_ompl_with_collision_op.py (规划)
    ↓ trajectory
trajectory_execution/trajectory_executor.py (执行)
    ↓ joint_commands
robot_control/gen72_robot_node.py (实体控制)
```

## 启动方式

### MuJoCo仿真
```bash
./run_mujoco.bat
```

### 实体机械臂
```bash
./run_real_robot.bat
```

## 模块依赖关系

```
workflow → config, ik_solver, motion_planner
ik_solver → config
motion_planner → config, collision_detection
trajectory_execution → config
robot_control → (Realman SDK)
collision_detection → config
```

## 重要说明

1. **所有Python模块现在都是包** - 每个目录都有`__init__.py`
2. **导入路径已更新** - 使用相对包导入(如`from config.robot_config import`)
3. **配置文件路径已更新** - dataflow中的path指向新的模块路径
4. **启动脚本已更新** - 指向config目录下的dataflow文件

## 迁移完成检查清单

✅ 文件已按模块分类移动
✅ 所有import语句已更新
✅ dataflow配置文件路径已更新
✅ 启动脚本路径已更新
✅ __init__.py文件已创建
✅ 项目结构文档已创建
