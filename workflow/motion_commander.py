#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Motion Commander for Dora-MoveIt + MuJoCo Test
==============================================

Interactive test interface that sends motion commands to the planner.
Demonstrates pick-and-place style movements.
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import Optional
from dora import Node


class MotionCommander:
    """High-level motion command interface"""
    
    def __init__(self):
        self.current_joints: Optional[np.ndarray] = None
        self.state = "init"
        self.step = 0
        self.waiting_for_plan = False
        self.waiting_for_execution = False
        
        # Define target configurations for GEN72 robot
        # Joint limits from URDF:
        # joint1: [-3.0014, 3.0014], joint2: [-1.8323, 1.8323], joint3: [-3.0014, 3.0014]
        # joint4: [-2.8792, 0.9597], joint5: [-3.0014, 3.0014], joint6: [-1.707, 1.783]
        # joint7: [-3.0014, 3.0014]

        # Home position - safe starting pose
        self.home_config = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])

        # Simple pick/place configurations - small movements, easy to plan
        self.poses = [
            # First go to a safe home position
            ("home", np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])),
            # Move to right side
            ("right", np.array([0.5, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])),
            # Move down slightly
            ("right_low", np.array([0.5, -0.3, 0.0, -0.5, 0.0, 0.5, 0.0])),
            # Back up
            ("right", np.array([0.5, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])),
            # Move to left side
            ("left", np.array([-0.5, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])),
            # Move down slightly
            ("left_low", np.array([-0.5, -0.3, 0.0, -0.5, 0.0, 0.5, 0.0])),
            # Back to home
            ("home", np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])),
        ]
        
        self.pose_idx = 0
        self.tick_count = 0
        
        print("=== Motion Commander ===")
        print("Will execute pick-and-place demo sequence")
        
    def update_joints(self, joints: np.ndarray):
        """Update current joint state (only first 7 arm joints, ignore gripper)"""
        # MuJoCo sends 9 values (7 arm + 2 gripper), we only need the 7 arm joints
        self.current_joints = joints[:7].copy()
        
    def get_next_command(self, node: Node) -> bool:
        """
        Generate next motion command.
        Returns True if command was sent.
        """
        self.tick_count += 1
        
        if self.waiting_for_plan or self.waiting_for_execution:
            return False
        
        if self.current_joints is None:
            return False
        
        if self.state == "init":
            # Add an obstacle to the scene
            print("\n[Commander] Setting up scene with obstacle...")
            command = {
                "action": "add",
                "object": {
                    "name": "box_obstacle",
                    "type": "box",
                    "position": [0.4, 0.3, 0.5],
                    "dimensions": [0.08, 0.08, 0.15],
                    "color": [1.0, 0.3, 0.3, 1.0]
                }
            }
            cmd_bytes = json.dumps(command).encode('utf-8')
            node.send_output("scene_command", pa.array(list(cmd_bytes), type=pa.uint8()))
            self.state = "planning"
            return True
            
        elif self.state == "planning":
            if self.pose_idx >= len(self.poses):
                print("\n[Commander] Demo sequence complete! Restarting...")
                self.pose_idx = 0
                return False
            
            pose_name, target_config = self.poses[self.pose_idx]
            print(f"\n[Commander] Planning motion to: {pose_name}")
            
            # Send planning request
            plan_request = {
                "start": self.current_joints.tolist(),
                "goal": target_config.tolist(),
                "planner": "rrt_connect",
                "max_time": 5.0
            }
            request_bytes = json.dumps(plan_request).encode('utf-8')
            node.send_output("plan_request", pa.array(list(request_bytes), type=pa.uint8()))
            
            self.waiting_for_plan = True
            return True
        
        return False
    
    def on_plan_result(self, status: dict):
        """Handle planning result"""
        self.waiting_for_plan = False
        
        if status.get("success"):
            print(f"[Commander] Plan succeeded: {status.get('num_waypoints', 0)} waypoints")
            self.waiting_for_execution = True
        else:
            print(f"[Commander] Plan failed: {status.get('message', 'Unknown')}")
            # Skip to next pose
            self.pose_idx += 1
            
    def on_execution_complete(self, status: dict):
        """Handle execution completion"""
        if not status.get("is_executing", True):
            self.waiting_for_execution = False
            self.pose_idx += 1
            print(f"[Commander] Execution complete, moving to next pose ({self.pose_idx}/{len(self.poses)})")


def main():
    print("\n" + "="*60)
    print("    Dora-MoveIt + MuJoCo Integration Test")
    print("="*60 + "\n")
    
    node = Node()
    commander = MotionCommander()
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "joint_positions":
                try:
                    joints = event["value"].to_numpy()
                    commander.update_joints(joints)
                except:
                    pass
                    
            elif input_id == "tick":
                commander.get_next_command(node)
                
            elif input_id == "plan_status":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        status_bytes = bytes(value.to_pylist())
                    else:
                        status_bytes = bytes(value)
                    status = json.loads(status_bytes.decode('utf-8'))
                    commander.on_plan_result(status)
                except Exception as e:
                    print(f"[Commander] Plan status error: {e}")
                    
            elif input_id == "execution_status":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        status_bytes = bytes(value.to_pylist())
                    else:
                        status_bytes = bytes(value)
                    status = json.loads(status_bytes.decode('utf-8'))
                    commander.on_execution_complete(status)
                except:
                    pass
                    
            elif input_id == "ik_status":
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        status_bytes = bytes(value.to_pylist())
                    else:
                        status_bytes = bytes(value)
                    status = json.loads(status_bytes.decode('utf-8'))
                    if status.get("success"):
                        print(f"[Commander] IK: OK error={status.get('error', 0):.6f}")
                    else:
                        print(f"[Commander] IK: FAILED {status.get('message', '')}")
                except:
                    pass
                    
        elif event_type == "STOP":
            print("\nMotion commander stopping...")
            break


if __name__ == "__main__":
    main()

