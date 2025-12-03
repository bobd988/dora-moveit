#!/usr/bin/env python3
"""
OMPL Motion Planner Operator for Dora-MoveIt
============================================

Motion planning operator using OMPL-like algorithms with integrated
collision checking. Supports multiple planning algorithms:

- RRT (Rapidly-exploring Random Trees)
- RRT-Connect
- PRM (Probabilistic Roadmap)
- RRT* (Optimal RRT)

The planner uses collision_lib.py for validity checking during planning.

Inputs:
    - plan_request: Planning request with start/goal configurations
    - scene_update: Updates to planning scene
    
Outputs:
    - trajectory: Planned trajectory (list of joint configurations)
    - plan_status: Planning result status

Key Feature: is_state_valid() callback embeds collision checking
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import List, Tuple, Optional, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
from dora import Node

# Import collision library
from collision_lib import (
    CollisionChecker,
    CollisionObject,
    CollisionObjectType,
    create_sphere,
    create_box,
    create_cylinder,
    create_robot_link
)


class PlannerType(Enum):
    """Available planning algorithms"""
    RRT = "rrt"
    RRT_CONNECT = "rrt_connect"
    RRT_STAR = "rrt_star"
    PRM = "prm"


@dataclass
class PlanRequest:
    """Motion planning request"""
    start_config: np.ndarray
    goal_config: np.ndarray
    planner_type: PlannerType = PlannerType.RRT_CONNECT
    max_planning_time: float = 5.0
    goal_tolerance: float = 0.05
    check_start_goal: bool = True


@dataclass
class PlanResult:
    """Motion planning result"""
    success: bool
    trajectory: List[np.ndarray] = field(default_factory=list)
    planning_time: float = 0.0
    path_length: float = 0.0
    num_nodes: int = 0
    message: str = ""


@dataclass
class TreeNode:
    """Node in the RRT tree"""
    config: np.ndarray
    parent_idx: int = -1
    cost: float = 0.0


class SimpleFK:
    """Simple forward kinematics for collision checking"""
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        
    def compute_link_transforms(self, joint_positions: np.ndarray) -> Dict[str, np.ndarray]:
        """Compute link positions from joint angles"""
        q = joint_positions
        transforms = {}
        
        c1, s1 = np.cos(q[0]), np.sin(q[0])
        c2, s2 = np.cos(q[1]), np.sin(q[1])
        c4, s4 = np.cos(q[3]), np.sin(q[3])
        c6, s6 = np.cos(q[5]), np.sin(q[5])
        
        l1, l2, l3 = 0.316, 0.384, 0.107
        
        transforms["link0"] = np.array([0.0, 0.0, 0.15])
        transforms["link1"] = np.array([0.0, 0.0, 0.333])
        transforms["link2"] = np.array([l1 * s2 * c1, l1 * s2 * s1, 0.333 + l1 * c2])
        transforms["link3"] = transforms["link2"] + np.array([0.0825 * c1, 0.0825 * s1, 0.0])
        transforms["link4"] = transforms["link3"] + np.array([l2 * s4 * c1, l2 * s4 * s1, l2 * c4])
        transforms["link5"] = transforms["link4"].copy()
        transforms["link6"] = transforms["link5"] + np.array([0.088 * c6 * c1, 0.088 * c6 * s1, 0.088 * s6])
        transforms["link7"] = transforms["link6"] + np.array([l3 * c1, l3 * s1, 0.0])
        
        return transforms


class OMPLPlanner:
    """
    OMPL-like motion planner with embedded collision checking.
    
    Implements RRT, RRT-Connect, and RRT* algorithms.
    """
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        
        # Joint limits (Panda robot)
        self.joint_limits_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.joint_limits_upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        
        # Planning parameters
        self.step_size = 0.2  # Maximum step in joint space
        self.goal_bias = 0.1  # Probability of sampling goal
        self.max_iterations = 5000
        
        # Collision checker
        self.collision_checker = CollisionChecker()
        self.fk = SimpleFK(num_joints)
        
        # Setup robot collision model
        self._setup_robot()
        
        # Statistics
        self.collision_checks = 0
        
    def _setup_robot(self):
        """Set up robot collision model"""
        links = [
            create_robot_link("link0", CollisionObjectType.CYLINDER, np.array([0.06, 0.15]), 0),
            create_robot_link("link1", CollisionObjectType.CYLINDER, np.array([0.06, 0.12]), 1),
            create_robot_link("link2", CollisionObjectType.CYLINDER, np.array([0.05, 0.15]), 2),
            create_robot_link("link3", CollisionObjectType.CYLINDER, np.array([0.05, 0.12]), 3),
            create_robot_link("link4", CollisionObjectType.CYLINDER, np.array([0.045, 0.15]), 4),
            create_robot_link("link5", CollisionObjectType.CYLINDER, np.array([0.04, 0.08]), 5),
            create_robot_link("link6", CollisionObjectType.SPHERE, np.array([0.04]), 6),
            create_robot_link("link7", CollisionObjectType.SPHERE, np.array([0.05]), 7),
        ]
        self.collision_checker.set_robot_links(links)
        
    def add_obstacle(self, obj: CollisionObject):
        """Add obstacle to planning scene"""
        self.collision_checker.add_environment_object(obj)
        
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.collision_checker.clear_environment()
        
    def is_state_valid(self, config: np.ndarray) -> bool:
        """
        Check if a configuration is valid (collision-free and within limits).
        
        This is the key callback used by OMPL planners.
        
        Args:
            config: Joint configuration
            
        Returns:
            True if valid (collision-free), False otherwise
        """
        self.collision_checks += 1
        
        # Check joint limits
        if np.any(config < self.joint_limits_lower) or np.any(config > self.joint_limits_upper):
            return False
        
        # Compute link positions
        link_transforms = self.fk.compute_link_transforms(config)
        
        # Check collision
        is_valid, _ = self.collision_checker.is_state_valid(link_transforms)
        
        return is_valid
    
    def is_motion_valid(self, config1: np.ndarray, config2: np.ndarray, resolution: float = 0.05) -> bool:
        """
        Check if motion between two configs is valid.
        
        Args:
            config1: Start configuration
            config2: End configuration
            resolution: Interpolation resolution
            
        Returns:
            True if entire motion is collision-free
        """
        dist = np.linalg.norm(config2 - config1)
        num_steps = max(2, int(dist / resolution))
        
        for i in range(num_steps + 1):
            t = i / num_steps
            config = config1 + t * (config2 - config1)
            if not self.is_state_valid(config):
                return False
        
        return True
    
    def sample_random_config(self) -> np.ndarray:
        """Sample a random configuration within joint limits"""
        return np.random.uniform(self.joint_limits_lower, self.joint_limits_upper)
    
    def nearest_neighbor(self, nodes: List[TreeNode], config: np.ndarray) -> int:
        """Find nearest node in tree to given configuration"""
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, node in enumerate(nodes):
            dist = np.linalg.norm(node.config - config)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def steer(self, from_config: np.ndarray, to_config: np.ndarray) -> np.ndarray:
        """Steer from one config towards another, limited by step size"""
        direction = to_config - from_config
        dist = np.linalg.norm(direction)
        
        if dist <= self.step_size:
            return to_config
        else:
            return from_config + (direction / dist) * self.step_size
    
    def extract_path(self, nodes: List[TreeNode], goal_idx: int) -> List[np.ndarray]:
        """Extract path from tree by backtracking from goal"""
        path = []
        idx = goal_idx
        
        while idx != -1:
            path.append(nodes[idx].config)
            idx = nodes[idx].parent_idx
        
        return list(reversed(path))
    
    def smooth_path(self, path: List[np.ndarray], iterations: int = 50) -> List[np.ndarray]:
        """Shortcut-based path smoothing"""
        if len(path) <= 2:
            return path
        
        smoothed = list(path)
        
        for _ in range(iterations):
            if len(smoothed) <= 2:
                break
            
            # Pick two random waypoints
            i = np.random.randint(0, len(smoothed) - 1)
            j = np.random.randint(i + 1, len(smoothed))
            
            if j - i <= 1:
                continue
            
            # Try to connect directly
            if self.is_motion_valid(smoothed[i], smoothed[j]):
                # Remove intermediate waypoints
                smoothed = smoothed[:i+1] + smoothed[j:]
        
        return smoothed
    
    # ==================== RRT Algorithm ====================
    
    def plan_rrt(self, request: PlanRequest) -> PlanResult:
        """
        RRT (Rapidly-exploring Random Trees) planner.
        
        Args:
            request: Planning request
            
        Returns:
            PlanResult with trajectory or failure info
        """
        start_time = time.time()
        self.collision_checks = 0
        
        # Initialize tree with start configuration
        nodes = [TreeNode(config=request.start_config, parent_idx=-1)]
        
        for iteration in range(self.max_iterations):
            # Check timeout
            if time.time() - start_time > request.max_planning_time:
                break
            
            # Sample random configuration (with goal bias)
            if np.random.random() < self.goal_bias:
                q_rand = request.goal_config
            else:
                q_rand = self.sample_random_config()
            
            # Find nearest node
            nearest_idx = self.nearest_neighbor(nodes, q_rand)
            q_near = nodes[nearest_idx].config
            
            # Steer towards random config
            q_new = self.steer(q_near, q_rand)
            
            # Check if motion is valid
            if self.is_motion_valid(q_near, q_new):
                # Add new node
                new_node = TreeNode(config=q_new, parent_idx=nearest_idx)
                nodes.append(new_node)
                
                # Check if goal reached
                if np.linalg.norm(q_new - request.goal_config) < request.goal_tolerance:
                    # Connect to goal
                    if self.is_motion_valid(q_new, request.goal_config):
                        goal_node = TreeNode(config=request.goal_config, parent_idx=len(nodes)-1)
                        nodes.append(goal_node)
                        
                        # Extract and smooth path
                        path = self.extract_path(nodes, len(nodes)-1)
                        path = self.smooth_path(path)
                        
                        # Compute path length
                        path_length = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))
                        
                        return PlanResult(
                            success=True,
                            trajectory=path,
                            planning_time=time.time() - start_time,
                            path_length=path_length,
                            num_nodes=len(nodes),
                            message=f"RRT found path with {len(path)} waypoints"
                        )
        
        return PlanResult(
            success=False,
            planning_time=time.time() - start_time,
            num_nodes=len(nodes),
            message=f"RRT failed after {self.max_iterations} iterations"
        )
    
    # ==================== RRT-Connect Algorithm ====================
    
    def plan_rrt_connect(self, request: PlanRequest) -> PlanResult:
        """
        RRT-Connect planner (bidirectional RRT).
        
        Args:
            request: Planning request
            
        Returns:
            PlanResult with trajectory or failure info
        """
        start_time = time.time()
        self.collision_checks = 0
        
        # Initialize two trees
        tree_a = [TreeNode(config=request.start_config, parent_idx=-1)]
        tree_b = [TreeNode(config=request.goal_config, parent_idx=-1)]
        
        for iteration in range(self.max_iterations):
            # Check timeout
            if time.time() - start_time > request.max_planning_time:
                break
            
            # Sample random configuration
            q_rand = self.sample_random_config()
            
            # Extend tree_a towards q_rand
            nearest_a = self.nearest_neighbor(tree_a, q_rand)
            q_new_a = self.steer(tree_a[nearest_a].config, q_rand)
            
            if self.is_motion_valid(tree_a[nearest_a].config, q_new_a):
                tree_a.append(TreeNode(config=q_new_a, parent_idx=nearest_a))
                
                # Try to connect tree_b to q_new_a
                nearest_b = self.nearest_neighbor(tree_b, q_new_a)
                q_connect = tree_b[nearest_b].config
                
                # Greedy extend towards q_new_a
                while True:
                    q_step = self.steer(q_connect, q_new_a)
                    
                    if not self.is_motion_valid(q_connect, q_step):
                        break
                    
                    tree_b.append(TreeNode(config=q_step, parent_idx=nearest_b))
                    nearest_b = len(tree_b) - 1
                    q_connect = q_step
                    
                    # Check if trees connected
                    if np.linalg.norm(q_step - q_new_a) < 0.01:
                        # Extract path from both trees
                        path_a = self.extract_path(tree_a, len(tree_a)-1)
                        path_b = self.extract_path(tree_b, len(tree_b)-1)
                        
                        # Combine paths
                        path = path_a + list(reversed(path_b))[1:]
                        path = self.smooth_path(path)
                        
                        # Compute path length
                        path_length = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))
                        
                        return PlanResult(
                            success=True,
                            trajectory=path,
                            planning_time=time.time() - start_time,
                            path_length=path_length,
                            num_nodes=len(tree_a) + len(tree_b),
                            message=f"RRT-Connect found path with {len(path)} waypoints"
                        )
            
            # Swap trees
            tree_a, tree_b = tree_b, tree_a
        
        return PlanResult(
            success=False,
            planning_time=time.time() - start_time,
            num_nodes=len(tree_a) + len(tree_b),
            message=f"RRT-Connect failed after {self.max_iterations} iterations"
        )
    
    def plan(self, request: PlanRequest) -> PlanResult:
        """
        Plan a path based on request parameters.
        
        Args:
            request: Planning request
            
        Returns:
            PlanResult with trajectory or failure info
        """
        # Validate start and goal if requested
        if request.check_start_goal:
            if not self.is_state_valid(request.start_config):
                return PlanResult(
                    success=False,
                    message="Start configuration is in collision"
                )
            if not self.is_state_valid(request.goal_config):
                return PlanResult(
                    success=False,
                    message="Goal configuration is in collision"
                )
        
        # Select planner
        if request.planner_type == PlannerType.RRT:
            result = self.plan_rrt(request)
        elif request.planner_type == PlannerType.RRT_CONNECT:
            result = self.plan_rrt_connect(request)
        else:
            result = self.plan_rrt_connect(request)  # Default to RRT-Connect
        
        print(f"  Collision checks: {self.collision_checks}")
        return result


class PlannerOperator:
    """
    Dora operator for OMPL motion planning.
    """

    def __init__(self):
        self.planner = OMPLPlanner(num_joints=7)
        self.plan_count = 0
        self.last_scene_version = -1  # Track scene version to avoid redundant updates

        # Add default obstacles
        self.planner.add_obstacle(create_box("table", np.array([0.5, 0.0, 0.4]), np.array([0.6, 0.8, 0.02])))
        self.planner.add_obstacle(create_box("ground", np.array([0.0, 0.0, -0.01]), np.array([2.0, 2.0, 0.02])))

        print("OMPL Planner operator initialized")
        print(f"  Obstacles: {len(self.planner.collision_checker.environment_objects)}")
        
    def process_plan_request(self, request_data: dict) -> Tuple[List[np.ndarray], dict]:
        """
        Process a planning request.
        
        Args:
            request_data: Dictionary with planning request
            
        Returns:
            Tuple of (trajectory, status_dict)
        """
        self.plan_count += 1
        
        start_config = np.array(request_data["start"])
        goal_config = np.array(request_data["goal"])
        planner_type = PlannerType(request_data.get("planner", "rrt_connect"))
        max_time = request_data.get("max_time", 5.0)
        
        request = PlanRequest(
            start_config=start_config,
            goal_config=goal_config,
            planner_type=planner_type,
            max_planning_time=max_time
        )
        
        print(f"\n[Plan #{self.plan_count}] {planner_type.value}")
        print(f"  Start: {start_config[:3]}...")
        print(f"  Goal:  {goal_config[:3]}...")
        
        result = self.planner.plan(request)
        
        status = {
            "plan_id": self.plan_count,
            "success": result.success,
            "planning_time": result.planning_time,
            "path_length": result.path_length,
            "num_waypoints": len(result.trajectory),
            "num_nodes": result.num_nodes,
            "message": result.message
        }
        
        if result.success:
            print(f"  SUCCESS: {len(result.trajectory)} waypoints, {result.planning_time:.3f}s")
        else:
            print(f"  FAILED: {result.message}")
        
        return result.trajectory, status
    
    def process_scene_update(self, update_data: dict):
        """
        Process scene update.

        Args:
            update_data: Dictionary with scene update. Can be:
                - Single object update: {"action": "add/remove/clear", "object": {...}}
                - Full scene broadcast: {"world_objects": [...], "attached_objects": [...]}
        """
        # Check if this is a full scene broadcast from planning_scene_op
        if "world_objects" in update_data:
            # Check scene version to avoid redundant updates
            scene_version = update_data.get("version", 0)
            if scene_version <= self.last_scene_version:
                return  # Already processed this version
            self.last_scene_version = scene_version

            # Full scene sync - clear and rebuild
            self.planner.clear_obstacles()
            for obj_data in update_data.get("world_objects", []):
                self._add_obstacle_from_data(obj_data)
            return

        # Otherwise handle as single object command
        action = update_data.get("action", "add")

        if action == "add":
            if "object" in update_data:
                self._add_obstacle_from_data(update_data["object"])
        elif action == "remove":
            self.planner.collision_checker.remove_environment_object(update_data["name"])
            print(f"[Planner] Removed obstacle: {update_data['name']}")
        elif action == "clear":
            self.planner.clear_obstacles()
            print("[Planner] Cleared all obstacles")

    def _add_obstacle_from_data(self, obj_data: dict):
        """Helper to add an obstacle from object data dictionary"""
        if obj_data["type"] == "sphere":
            obj = create_sphere(obj_data["name"], np.array(obj_data["position"]), obj_data["dimensions"][0])
        elif obj_data["type"] == "box":
            obj = create_box(obj_data["name"], np.array(obj_data["position"]), np.array(obj_data["dimensions"]))
        else:
            obj = create_cylinder(obj_data["name"], np.array(obj_data["position"]),
                                  obj_data["dimensions"][0], obj_data["dimensions"][1])
        self.planner.add_obstacle(obj)
        print(f"[Planner] Added obstacle: {obj_data['name']}")


def main():
    """Main entry point for Dora OMPL planner operator"""
    print("=== Dora-MoveIt OMPL Planner Operator ===")
    
    node = Node()
    planner_op = PlannerOperator()
    
    print("Planner ready, waiting for requests...")
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "plan_request":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        request_bytes = bytes(value.to_pylist())
                    else:
                        request_bytes = bytes(value)
                    
                    request_data = json.loads(request_bytes.decode('utf-8'))
                    
                    trajectory, status = planner_op.process_plan_request(request_data)
                    
                    # Send status
                    status_bytes = json.dumps(status).encode('utf-8')
                    node.send_output(
                        "plan_status",
                        pa.array(list(status_bytes), type=pa.uint8()),
                        metadata={"success": status["success"]}
                    )
                    
                    # Send trajectory if successful
                    if trajectory:
                        # Flatten trajectory for transmission
                        traj_flat = np.array([q for q in trajectory]).flatten()
                        node.send_output(
                            "trajectory",
                            pa.array(traj_flat, type=pa.float32()),
                            metadata={
                                "num_waypoints": len(trajectory),
                                "num_joints": planner_op.planner.num_joints
                            }
                        )
                        
                except Exception as e:
                    print(f"[Planner] Error: {e}")
                    import traceback
                    traceback.print_exc()
                    
            elif input_id == "scene_update":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        update_bytes = bytes(value.to_pylist())
                    else:
                        update_bytes = bytes(value)
                    
                    update_data = json.loads(update_bytes.decode('utf-8'))
                    planner_op.process_scene_update(update_data)
                    
                except Exception as e:
                    print(f"[Planner] Scene update error: {e}")
                    
        elif event_type == "STOP":
            print("OMPL planner stopping...")
            break


if __name__ == "__main__":
    main()

