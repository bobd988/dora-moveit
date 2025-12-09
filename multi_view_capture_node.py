#!/usr/bin/env python3
"""
Multi-View Capture Node for GEN72
==================================
Implements multi-viewpoint photography workflow:
1. Move to target position 1 -> capture
2. Move to target position 2 -> capture
3. Move to target position 3 -> capture

If target unreachable (collision/IK fail), find nearby reachable position.
"""

import os
import json
import time
import numpy as np
import pyarrow as pa
from dataclasses import dataclass
from dora import Node
from robot_config import GEN72Config


@dataclass
class CaptureTarget:
    """Target position for capture"""
    name: str
    position: np.ndarray  # [x, y, z, roll, pitch, yaw]
    search_radius: float = 0.05  # 5cm search radius if unreachable


class MultiViewCaptureNode:
    """Multi-view capture workflow controller"""

    def __init__(self):
        # Capture targets (3 viewpoints) - larger movements for visibility
        self.targets = [
            CaptureTarget("view1", np.array([0.3, 0.3, 0.4, 3.14, 0.0, 0.0])),
            CaptureTarget("view2", np.array([0.5, -0.3, 0.6, 3.14, 0.0, 1.57])),
            CaptureTarget("view3", np.array([-0.3, 0.0, 0.3, 3.14, 0.0, 3.14])),
        ]

        self.current_target_idx = 0
        self.current_joints = GEN72Config.SAFE_CONFIG.copy()
        self.state = "init"  # init -> moving -> capturing -> complete
        self.ik_attempts = 0
        self.max_ik_attempts = 5

        # Camera configuration (configurable via environment variables)
        # CAPTURE_CAMERA_INDEX: camera index (default 0)
        # CAPTURE_OUTPUT_DIR: output directory (default ./captures)
        self.camera_index = int(os.getenv("CAPTURE_CAMERA_INDEX", "0"))
        self.output_dir = os.getenv("CAPTURE_OUTPUT_DIR", "captures")
        os.makedirs(self.output_dir, exist_ok=True)
        self.cap = None  # Lazy camera open

        print("=== Multi-View Capture Node ===")
        print(f"Targets: {len(self.targets)} viewpoints")
        print(f"Initial config: {self.current_joints[:3]}...")

    def run(self):
        node = Node()

        # Initial robot state (not actually used in current dataflow,
        # but kept for compatibility / future extension)
        self._send_robot_state(node, self.current_joints)
        time.sleep(0.5)

        # Start first target
        self._next_target(node)

        for event in node:
            if event["type"] == "INPUT":
                self._handle_input(node, event)
            elif event["type"] == "STOP":
                break

        print("\nMulti-view capture workflow complete!")

    def _handle_input(self, node: Node, event):
        event_id = event["id"]

        if event_id == "ik_solution":
            # ik_op.py: sends pyarrow.FloatArray(solution)
            self._handle_ik_solution(node, event["value"])
        elif event_id == "trajectory":
            # planner: sends pyarrow.FloatArray(flattened trajectory), with metadata
            self._handle_trajectory(node, event)
        elif event_id == "plan_status":
            # planner: sends JSON bytes wrapped into pyarrow.UInt8Array
            self._handle_plan_status(node, event["value"])

    def _handle_ik_solution(self, node: Node, data):
        """Handle IK solution"""
        try:
            if hasattr(data, "to_numpy"):
                joints = data.to_numpy()
            else:
                joints = np.frombuffer(data, dtype=np.float32)
        except Exception as e:
            print(f"[Capture] Error decoding IK solution: {e}")
            joints = np.array([], dtype=np.float32)

        if len(joints) == 0:
            print(f"  IK failed for {self.targets[self.current_target_idx].name}")
            self.ik_attempts += 1

            if self.ik_attempts < self.max_ik_attempts:
                # Try nearby position
                self._request_nearby_ik(node)
            else:
                print(f"  Skipping {self.targets[self.current_target_idx].name}")
                self._next_target(node)
        else:
            print(f"  IK solved: {joints[:3]}...")
            # Request motion plan
            self._request_plan(node, self.current_joints, joints)

    def _handle_plan_status(self, node: Node, data):
        """Handle planning status"""
        # planner_ompl_with_collision_op.py: plan_status is JSON -> bytes -> pyarrow.UInt8Array
        try:
            if hasattr(data, "to_pylist"):
                status_bytes = bytes(data.to_pylist())
            else:
                status_bytes = bytes(data)
            status = json.loads(status_bytes.decode("utf-8"))
        except Exception as e:
            print(f"[Capture] Error decoding plan_status: {e}")
            return

        if not status.get("success", False):
            msg = status.get("message", status.get("error", "unknown"))
            print(f"  Planning failed: {msg}")
            self.ik_attempts += 1

            if self.ik_attempts < self.max_ik_attempts:
                self._request_nearby_ik(node)
            else:
                print(f"  Skipping {self.targets[self.current_target_idx].name}")
                self._next_target(node)
        else:
            print("  Planning succeeded")

    def _handle_trajectory(self, node: Node, event):
        """Handle planned trajectory"""
        value = event["value"]
        metadata = event.get("metadata", {}) if isinstance(event, dict) else {}

        try:
            if hasattr(value, "to_numpy"):
                traj_flat = value.to_numpy()
            else:
                traj_flat = np.frombuffer(value, dtype=np.float32)
        except Exception as e:
            print(f"[Capture] Error decoding trajectory: {e}")
            return

        num_joints = GEN72Config.NUM_JOINTS
        num_waypoints = metadata.get("num_waypoints", len(traj_flat) // num_joints)
        if num_waypoints <= 0:
            print("[Capture] Invalid trajectory: num_waypoints <= 0")
            return

        try:
            waypoints = traj_flat.reshape(num_waypoints, num_joints)
        except Exception as e:
            print(f"[Capture] Error reshaping trajectory: {e}")
            return

        print(f"  Trajectory received: {len(waypoints)} waypoints")

        # 1. 更新当前关节
        self.current_joints = waypoints[-1]

        # 2. 在当前视角拍照
        self._capture_image(node)

        # 3. 切换到下一个视角
        self.current_target_idx += 1
        if self.current_target_idx >= len(self.targets):
            print("\nAll captures complete!")
            return

        # 4. 继续下一个视角的 IK + 规划
        time.sleep(0.5)
        self._next_target(node)


    # --------------------- Core Logic --------------------- #

    def _capture_image(self, node: Node):
        """Capture an image from camera and save to disk"""
        target = self.targets[self.current_target_idx]
        print("\nCapturing image at", target.name)
        print("   Position:", target.position[:3])

        try:
            import cv2 as cv
        except ImportError:
            print("   OpenCV not installed, skipping actual capture")
            return

        # Lazily open camera
        if self.cap is None:
            self.cap = cv.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print("   Failed to open camera index", self.camera_index)
                self.cap.release()
                self.cap = None
                return

        # Grab a single frame
        ok, frame = self.cap.read()
        if not ok or frame is None:
            print("   Failed to grab frame from camera")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{target.name}_{timestamp}.png"
        filepath = os.path.join(self.output_dir, filename)

        if cv.imwrite(filepath, frame):
            print("   Image saved:", filepath)
        else:
            print("   Failed to write image file")

    def _next_target(self, node: Node):
        """Move to next capture target"""
        self.ik_attempts = 0

        if self.current_target_idx >= len(self.targets):
            print("\nAll captures complete!")
            return

        target = self.targets[self.current_target_idx]
        print(f"\n[Target {self.current_target_idx + 1}/{len(self.targets)}] Moving to {target.name}")
        self._request_ik(node, target.position)

    def _request_ik(self, node: Node, pose: np.ndarray):
        """Send IK request to ik_op (as float32 Arrow array: [x, y, z, roll, pitch, yaw])"""
        pose = np.asarray(pose, dtype=np.float32)
        if pose.shape[0] != 6:
            print(f"[Capture] Warning: expected 6D pose, got {pose.shape[0]}")
        node.send_output(
            "ik_request",
            pa.array(pose, type=pa.float32())
        )

    def _request_nearby_ik(self, node: Node):
        """Try IK for nearby target pose"""
        target = self.targets[self.current_target_idx]
        radius = target.search_radius

        # Sample small random offset around target
        offset = np.random.uniform(-radius, radius, size=3)
        new_pose = target.position.copy()
        new_pose[:3] += offset

        print("  Trying nearby pose offset:", offset)
        self._request_ik(node, new_pose)

    def _request_plan(self, node: Node, start_joints: np.ndarray, goal_joints: np.ndarray):
        """Send planning request to planner"""
        request = {
            "start": np.asarray(start_joints, dtype=float).tolist(),
            "goal": np.asarray(goal_joints, dtype=float).tolist(),
            "planner": "rrt_connect",
            "max_time": 5.0
        }
        request_bytes = json.dumps(request).encode("utf-8")
        node.send_output(
            "plan_request",
            pa.array(list(request_bytes), type=pa.uint8())
        )

    def _send_robot_state(self, node: Node, joints: np.ndarray):
        """Send robot state (currently not wired in dataflow, kept for extension)"""
        state = {"joints": np.asarray(joints, dtype=float).tolist()}
        state_bytes = json.dumps(state).encode("utf-8")
        node.send_output(
            "robot_state",
            pa.array(list(state_bytes), type=pa.uint8())
        )


if __name__ == "__main__":
    node = MultiViewCaptureNode()
    node.run()
