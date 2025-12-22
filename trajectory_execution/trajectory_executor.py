#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Trajectory Executor for Dora-MoveIt + MuJoCo
============================================

Executes planned trajectories by sending joint commands to MuJoCo.
Interpolates between waypoints for smooth motion.
"""

import json
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
        self.prev_waypoint: Optional[np.ndarray] = None

        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.1

        self.is_executing = False
        self.execution_count = 0
        self.current_trajectory_hash: Optional[int] = None

        self.current_joints: Optional[np.ndarray] = None
        self.last_command: Optional[np.ndarray] = None

    def set_trajectory(self, trajectory: List[np.ndarray], trajectory_hash: int):
        """Set a new trajectory to execute"""
        if self.current_trajectory_hash == trajectory_hash:
            return

        self.trajectory = trajectory
        self.current_trajectory_hash = trajectory_hash
        self.interpolation_progress = 0.0
        self.is_executing = True
        self.execution_count += 1

        if len(trajectory) > 0:
            self.prev_waypoint = trajectory[0]
            self.current_waypoint_idx = 1 if len(trajectory) > 1 else 0
            self.last_command = trajectory[0].copy()
            print(f"[Executor] New trajectory with {len(trajectory)} waypoints")

    def update_current_joints(self, joints: np.ndarray):
        """Update current joint positions from MuJoCo"""
        self.current_joints = joints[:self.num_joints].copy()

    def step(self) -> Optional[np.ndarray]:
        """
        Execute one step.
        ALWAYS output current joint state when idle.
        """

        # =========================
        # IDLE / HOLD MODE
        # =========================
        if not self.is_executing or len(self.trajectory) == 0:
            if self.current_joints is not None:
                return self.current_joints.copy()
            return self.last_command

        if self.prev_waypoint is None:
            return self.current_joints.copy() if self.current_joints is not None else self.last_command

        # =========================
        # EXECUTION MODE
        # =========================
        target = self.trajectory[self.current_waypoint_idx]
        self.interpolation_progress += self.interpolation_speed

        if self.interpolation_progress >= 1.0:
            self.prev_waypoint = target
            self.current_waypoint_idx += 1
            self.interpolation_progress = 0.0

            # ===== Trajectory finished =====
            if self.current_waypoint_idx >= len(self.trajectory):
                self.is_executing = False
                print(f"[Executor] Trajectory #{self.execution_count} complete!")

                # CRITICAL FIX:
                # Do NOT output last waypoint
                # Hold current real joint state
                if self.current_joints is not None:
                    self.last_command = self.current_joints.copy()
                    return self.current_joints.copy()

                return self.last_command

            target = self.trajectory[self.current_waypoint_idx]

        t = min(self.interpolation_progress, 1.0)
        command = self.prev_waypoint + t * (target - self.prev_waypoint)
        self.last_command = command.copy()
        return command

    def get_status(self) -> dict:
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

    from config.robot_config import GEN72Config
    executor.current_joints = GEN72Config.SAFE_CONFIG.copy()
    executor.last_command = GEN72Config.SAFE_CONFIG.copy()
    print(f"Initialized with safe config: {executor.current_joints[:6]}...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "trajectory":
                traj_flat = event["value"].to_numpy()
                metadata = event.get("metadata", {})
                num_waypoints = metadata.get("num_waypoints", len(traj_flat) // 7)
                num_joints = metadata.get("num_joints", 7)

                trajectory = traj_flat.reshape(num_waypoints, num_joints)
                trajectory_list = [trajectory[i] for i in range(num_waypoints)]

                if executor.current_joints is not None:
                    trajectory_list.insert(0, executor.current_joints.copy())

                traj_hash = hash(traj_flat.tobytes())
                executor.set_trajectory(trajectory_list, traj_hash)

            elif input_id == "joint_positions":
                executor.update_current_joints(event["value"].to_numpy())

            elif input_id == "tick":
                command = executor.step()

                if command is not None:
                    node.send_output(
                        "joint_commands",
                        pa.array(command, type=pa.float32())
                    )

                status_bytes = json.dumps(executor.get_status()).encode("utf-8")
                node.send_output(
                    "execution_status",
                    pa.array(list(status_bytes), type=pa.uint8())
                )

        elif event["type"] == "STOP":
            print("Trajectory executor stopping...")
            break


if __name__ == "__main__":
    main()
