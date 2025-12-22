#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
GEN72 Real Robot Control Node
==============================
Controls physical GEN72 robot arm via Realman SDK.
"""

import numpy as np
import pyarrow as pa
from dora import Node
import time
from typing import Optional

try:
    from robot_control.rm_robot_interface import *
except ImportError:
    print("ERROR: Realman SDK not found")
    RoboticArm = None


class GEN72RobotNode:
    """Real GEN72 robot control node"""

    def __init__(self):
        self.robot = None
        self.handle = None
        self.num_joints = 7
        self.current_joints: Optional[np.ndarray] = None
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.2
        self.target_q = None

        self.connect()

    def connect(self):
        """Connect to GEN72 robot"""
        if RoboticArm is None:
            print("ERROR: Realman SDK not available")
            return False

        try:
            thread_mode = rm_thread_mode_e(2)
            self.robot = RoboticArm(thread_mode)
            self.handle = self.robot.rm_create_robot_arm("192.168.1.19", 8080, 3)

            if self.handle.id == -1:
                print("Failed to connect to GEN72")
                return False

            print(f"Connected to GEN72: handle {self.handle.id}")
            self.robot.rm_set_arm_power(1)
            time.sleep(0.5)

            self.update_joint_state()
            return True

        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def update_joint_state(self):
        """Read current joint positions from robot"""
        if self.robot is None:
            return

        try:
            result = self.robot.rm_get_current_arm_state()
            if isinstance(result, tuple) and len(result) > 0:
                joint_data = result[1]
                if hasattr(joint_data, 'joint'):
                    self.current_joints = np.deg2rad(np.array(joint_data.joint[:7], dtype=np.float32))
        except Exception as e:
            print(f"Error reading joints: {e}")

    def apply_control(self, command: np.ndarray):
        """Send joint command to robot"""
        if self.robot is None:
            return

        self.last_cmd_time = time.time()
        self.target_q = command[:self.num_joints].copy()

        try:
            joint_deg = np.rad2deg(command[:self.num_joints]).tolist()
            self.robot.rm_movej(joint_deg, 10, 0, 0, 1)
        except Exception as e:
            print(f"Error sending command: {e}")

    def hold_position(self):
        """Hold current position when idle"""
        now = time.time()

        if self.target_q is None:
            return

        if now - self.last_cmd_time > self.cmd_timeout:
            if self.current_joints is not None:
                self.target_q = self.current_joints.copy()

    def disconnect(self):
        """Disconnect from robot"""
        if self.robot:
            self.robot.rm_delete_robot_arm()
            print("Disconnected from GEN72")


def main():
    print("=== GEN72 Real Robot Node ===")

    node = Node()
    robot_node = GEN72RobotNode()

    if robot_node.robot is None:
        print("Failed to initialize robot. Exiting.")
        return

    try:
        for event in node:
            if event["type"] == "INPUT":
                input_id = event["id"]

                if input_id == "control_input":
                    command = event["value"].to_numpy()
                    robot_node.apply_control(command)

                elif input_id == "tick":
                    robot_node.update_joint_state()
                    robot_node.hold_position()

                    if robot_node.current_joints is not None:
                        node.send_output(
                            "joint_positions",
                            pa.array(robot_node.current_joints, type=pa.float32()),
                            {"timestamp": time.time(), "encoding": "jointstate"}
                        )

                        node.send_output(
                            "joint_velocities",
                            pa.array(np.zeros(7), type=pa.float32()),
                            {"timestamp": time.time()}
                        )

            elif event["type"] == "STOP":
                print("Stopping robot node...")
                break

    finally:
        robot_node.disconnect()


if __name__ == "__main__":
    main()
