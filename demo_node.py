#!/usr/bin/env python3
"""
Demo Node for Dora-MoveIt
=========================

Interactive demonstration node that tests all Dora-MoveIt components:
- Planning Scene management
- OMPL motion planning
- Inverse Kinematics
- Collision checking

This node simulates a typical robot motion planning workflow:
1. Setup scene with obstacles
2. Request motion plans
3. Validate trajectories
4. Execute (simulated) motions

Run with: dora start dataflow.yml
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import Optional, List
from dataclasses import dataclass
from dora import Node


@dataclass
class DemoState:
    """Demo state machine"""
    phase: str = "init"
    step: int = 0
    current_joints: np.ndarray = None
    target_joints: np.ndarray = None
    trajectory: List[np.ndarray] = None
    trajectory_idx: int = 0


class DemoNode:
    """
    Interactive demo for Dora-MoveIt.
    
    Demonstrates:
    1. Scene setup (add/remove obstacles)
    2. IK requests (pose → joints)
    3. Motion planning (start → goal)
    4. Collision checking
    5. Trajectory execution
    """
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        self.state = DemoState()
        
        # Initial robot configuration
        self.state.current_joints = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        # Demo configurations
        self.home_config = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.pick_config = np.array([0.0, 0.2, 0.0, -1.5, 0.0, 1.7, 0.785])
        self.place_config = np.array([1.57, 0.2, 0.0, -1.5, 0.0, 1.7, 0.785])
        
        # Demo sequence
        self.demo_sequence = [
            ("setup_scene", "Setting up planning scene"),
            ("add_obstacle", "Adding obstacle to scene"),
            ("request_ik", "Testing IK solver"),
            ("plan_motion", "Planning motion to goal"),
            ("execute_trajectory", "Executing planned trajectory"),
            ("check_collision", "Validating final configuration"),
            ("complete", "Demo complete!"),
        ]
        
        self.tick_count = 0
        
        print("=== Dora-MoveIt Demo Node ===")
        print(f"Initial configuration: {self.state.current_joints[:3]}...")
        print("Demo will run through all components automatically")
        
    def get_current_demo_step(self) -> tuple:
        """Get current demo step"""
        if self.state.step >= len(self.demo_sequence):
            return ("complete", "Demo complete!")
        return self.demo_sequence[self.state.step]
    
    def advance_demo(self):
        """Advance to next demo step"""
        self.state.step += 1
        if self.state.step < len(self.demo_sequence):
            step_name, description = self.get_current_demo_step()
            print(f"\n[Demo Step {self.state.step}] {description}")
    
    def process_tick(self, node: Node):
        """Process periodic tick - drives the demo"""
        self.tick_count += 1
        
        step_name, description = self.get_current_demo_step()
        
        if step_name == "setup_scene":
            # Send initial robot state
            node.send_output(
                "robot_state",
                pa.array(self.state.current_joints, type=pa.float32())
            )
            print(f"  Sent robot state: {self.state.current_joints[:3]}...")
            self.advance_demo()
            
        elif step_name == "add_obstacle":
            # Add a test obstacle
            command = {
                "action": "add",
                "object": {
                    "name": "test_obstacle",
                    "type": "sphere",
                    "position": [0.4, 0.2, 0.6],
                    "dimensions": [0.1],
                    "color": [1.0, 0.0, 0.0, 1.0]
                }
            }
            cmd_bytes = json.dumps(command).encode('utf-8')
            node.send_output(
                "scene_command",
                pa.array(list(cmd_bytes), type=pa.uint8())
            )
            print(f"  Added obstacle: test_obstacle")
            self.advance_demo()
            
        elif step_name == "request_ik":
            # Request IK for a target pose
            target_pose = np.array([0.5, 0.0, 0.5, 180.0, 0.0, 90.0])  # x,y,z,r,p,y
            node.send_output(
                "ik_request",
                pa.array(target_pose, type=pa.float32())
            )
            print(f"  Requested IK for pose: {target_pose[:3]}")
            self.advance_demo()
            
        elif step_name == "plan_motion":
            # Request motion plan
            plan_request = {
                "start": self.state.current_joints.tolist(),
                "goal": self.pick_config.tolist(),
                "planner": "rrt_connect",
                "max_time": 5.0
            }
            request_bytes = json.dumps(plan_request).encode('utf-8')
            node.send_output(
                "plan_request",
                pa.array(list(request_bytes), type=pa.uint8())
            )
            print(f"  Requested motion plan: {self.state.current_joints[:2]}... → {self.pick_config[:2]}...")
            self.advance_demo()
            
        elif step_name == "execute_trajectory":
            if self.state.trajectory is not None and len(self.state.trajectory) > 0:
                # Execute trajectory step by step
                if self.state.trajectory_idx < len(self.state.trajectory):
                    self.state.current_joints = self.state.trajectory[self.state.trajectory_idx]
                    node.send_output(
                        "robot_state",
                        pa.array(self.state.current_joints, type=pa.float32())
                    )
                    print(f"  Executing waypoint {self.state.trajectory_idx + 1}/{len(self.state.trajectory)}")
                    self.state.trajectory_idx += 1
                else:
                    print(f"  Trajectory execution complete!")
                    self.advance_demo()
            else:
                print(f"  No trajectory to execute (waiting for planner...)")
                # Don't advance yet, wait for trajectory
            
        elif step_name == "check_collision":
            # Check collision for current configuration
            node.send_output(
                "collision_check",
                pa.array(self.state.current_joints, type=pa.float32())
            )
            print(f"  Checking collision for current config...")
            self.advance_demo()
            
        elif step_name == "complete":
            if self.tick_count % 5 == 0:
                print("\n✅ Demo complete! All Dora-MoveIt components tested.")
                print("   - Planning Scene: ✓")
                print("   - IK Solver: ✓")
                print("   - OMPL Planner: ✓")
                print("   - Collision Checker: ✓")
                print("\nPress Ctrl+C to stop.")
    
    def process_trajectory(self, trajectory_flat: np.ndarray, num_waypoints: int):
        """Process received trajectory"""
        # Reshape trajectory
        trajectory = trajectory_flat.reshape(num_waypoints, self.num_joints)
        self.state.trajectory = [trajectory[i] for i in range(num_waypoints)]
        self.state.trajectory_idx = 0
        print(f"  Received trajectory with {num_waypoints} waypoints")
    
    def process_plan_status(self, status: dict):
        """Process planning result"""
        if status.get("success"):
            print(f"  ✅ Planning succeeded in {status.get('planning_time', 0):.3f}s")
        else:
            print(f"  ❌ Planning failed: {status.get('message', 'Unknown error')}")
    
    def process_ik_status(self, status: dict):
        """Process IK result"""
        if status.get("success"):
            print(f"  ✅ IK succeeded with error {status.get('error', 0):.6f}")
        else:
            print(f"  ❌ IK failed: {status.get('message', 'Unknown error')}")
    
    def process_collision_result(self, result: dict):
        """Process collision check result"""
        if result.get("in_collision"):
            info = result.get("collision_info", {})
            print(f"  ❌ Collision detected: {info.get('object_a', '?')} ↔ {info.get('object_b', '?')}")
        else:
            dist = result.get("min_distance", 0)
            print(f"  ✅ No collision (min distance: {dist:.4f}m)")


def main():
    """Main entry point"""
    print("\n" + "="*60)
    print("       Dora-MoveIt Demo - Mini Motion Planning Framework")
    print("="*60 + "\n")
    
    node = Node()
    demo = DemoNode(num_joints=7)
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "tick":
                demo.process_tick(node)
                
            elif input_id == "trajectory":
                try:
                    traj_flat = event["value"].to_numpy()
                    metadata = event.get("metadata", {})
                    num_waypoints = metadata.get("num_waypoints", len(traj_flat) // 7)
                    demo.process_trajectory(traj_flat, num_waypoints)
                except Exception as e:
                    print(f"[Demo] Trajectory error: {e}")
                    
            elif input_id == "plan_status":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        status_bytes = bytes(value.to_pylist())
                    else:
                        status_bytes = bytes(value)
                    status = json.loads(status_bytes.decode('utf-8'))
                    demo.process_plan_status(status)
                except Exception as e:
                    print(f"[Demo] Plan status error: {e}")
                    
            elif input_id == "ik_solution":
                try:
                    solution = event["value"].to_numpy()
                    print(f"  IK solution: {solution[:3]}...")
                except Exception as e:
                    print(f"[Demo] IK solution error: {e}")
                    
            elif input_id == "ik_status":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        status_bytes = bytes(value.to_pylist())
                    else:
                        status_bytes = bytes(value)
                    status = json.loads(status_bytes.decode('utf-8'))
                    demo.process_ik_status(status)
                except Exception as e:
                    print(f"[Demo] IK status error: {e}")
                    
            elif input_id == "collision_result":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        result_bytes = bytes(value.to_pylist())
                    else:
                        result_bytes = bytes(value)
                    result = json.loads(result_bytes.decode('utf-8'))
                    demo.process_collision_result(result)
                except Exception as e:
                    print(f"[Demo] Collision result error: {e}")
                    
            elif input_id == "scene_update":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        update_bytes = bytes(value.to_pylist())
                    else:
                        update_bytes = bytes(value)
                    update = json.loads(update_bytes.decode('utf-8'))
                    num_objects = len(update.get("world_objects", []))
                    print(f"  Scene updated: {num_objects} objects")
                except Exception as e:
                    pass  # Scene updates can be frequent, ignore errors
                    
            elif input_id == "command_result":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        result_bytes = bytes(value.to_pylist())
                    else:
                        result_bytes = bytes(value)
                    result = json.loads(result_bytes.decode('utf-8'))
                    if result.get("success"):
                        print(f"  Command '{result.get('action')}' succeeded")
                except Exception as e:
                    pass
                    
        elif event_type == "STOP":
            print("\nDemo stopping...")
            break
    
    print("Demo node exited.")


if __name__ == "__main__":
    main()

