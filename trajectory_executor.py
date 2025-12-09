#!/usr/bin/env python3
"""
Trajectory Executor for Dora-MoveIt + MuJoCo
============================================

Executes planned trajectories by sending joint commands to MuJoCo.
Interpolates between waypoints for smooth motion.
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import List, Optional
from dora import Node


class TrajectoryExecutor:
    """Executes motion trajectories on the robot"""
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        self.trajectory: List[np.ndarray] = []
        self.current_waypoint_idx = 0
        self.current_joints: Optional[np.ndarray] = None
        self.target_joints: Optional[np.ndarray] = None
        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.02  # Progress per tick (slower for visibility)
        self.is_executing = False
        self.execution_count = 0
        
    def set_trajectory(self, trajectory: List[np.ndarray]):
        """Set a new trajectory to execute"""
        self.trajectory = trajectory
        self.current_waypoint_idx = 0
        self.interpolation_progress = 0.0
        self.is_executing = True
        self.execution_count += 1
        
        if len(trajectory) > 0:
            self.target_joints = trajectory[0]
            print(f"[Executor] New trajectory with {len(trajectory)} waypoints")
        
    def update_current_joints(self, joints: np.ndarray):
        """Update current joint positions from robot (only first 7 arm joints)"""
        # MuJoCo sends 9 values (7 arm + 2 gripper), we only need the 7 arm joints
        self.current_joints = joints[:self.num_joints].copy()
        
    def step(self) -> Optional[np.ndarray]:
        """
        Execute one step of trajectory.
        Returns joint command or None if not executing.
        """
        if not self.is_executing or len(self.trajectory) == 0:
            return None
        
        if self.current_joints is None:
            return None
        
        # Get current target waypoint
        target = self.trajectory[self.current_waypoint_idx]
        
        # Interpolate towards target
        self.interpolation_progress += self.interpolation_speed
        
        if self.interpolation_progress >= 1.0:
            # Reached waypoint, move to next
            self.current_waypoint_idx += 1
            self.interpolation_progress = 0.0
            
            if self.current_waypoint_idx >= len(self.trajectory):
                # Trajectory complete
                self.is_executing = False
                print(f"[Executor] Trajectory #{self.execution_count} complete!")
                return self.trajectory[-1]  # Return final position
            
            target = self.trajectory[self.current_waypoint_idx]
        
        # Interpolate between current position and target
        t = min(self.interpolation_progress, 1.0)
        command = self.current_joints + t * (target - self.current_joints)
        
        return command
    
    def get_status(self) -> dict:
        """Get execution status"""
        return {
            "is_executing": self.is_executing,
            "execution_count": self.execution_count,
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": len(self.trajectory),
            "progress": self.interpolation_progress
        }


def main():
    print("=== Dora-MoveIt Trajectory Executor ===")
    
    node = Node()
    executor = TrajectoryExecutor(num_joints=7)
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "trajectory":
                # Receive new trajectory from planner
                try:
                    traj_flat = event["value"].to_numpy()
                    metadata = event.get("metadata", {})
                    num_waypoints = metadata.get("num_waypoints", len(traj_flat) // 7)
                    num_joints = metadata.get("num_joints", 7)
                    
                    trajectory = traj_flat.reshape(num_waypoints, num_joints)
                    trajectory_list = [trajectory[i] for i in range(num_waypoints)]
                    executor.set_trajectory(trajectory_list)
                    
                except Exception as e:
                    print(f"[Executor] Trajectory error: {e}")
                    
            elif input_id == "joint_positions":
                # Update current robot state
                try:
                    joints = event["value"].to_numpy()
                    executor.update_current_joints(joints)
                except Exception as e:
                    pass
                    
            elif input_id == "tick":
                # Execute trajectory step
                command = executor.step()
                
                if command is not None:
                    node.send_output(
                        "joint_commands",
                        pa.array(command, type=pa.float32())
                    )
                
                # Send status periodically
                status = executor.get_status()
                if status["is_executing"]:
                    status_bytes = json.dumps(status).encode('utf-8')
                    node.send_output(
                        "execution_status",
                        pa.array(list(status_bytes), type=pa.uint8())
                    )
                    
        elif event_type == "STOP":
            print("Trajectory executor stopping...")
            break


if __name__ == "__main__":
    main()

