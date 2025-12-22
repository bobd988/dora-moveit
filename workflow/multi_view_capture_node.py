#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
from config.robot_config import GEN72Config


@dataclass
class CaptureTarget:
    """Target position for capture"""
    name: str
    position: np.ndarray  # [x, y, z, roll, pitch, yaw]
    search_radius: float = 0.05  # 5cm search radius if unreachable


class MultiViewCaptureNode:
    """Multi-view capture workflow controller"""

    def __init__(self):
        # Capture targets (3 viewpoints) - adjusted for reachability
        self.targets = [
            CaptureTarget("view1", np.array([0.4,  0.2, 0.5, 0.0, 0.0, 0.0])),
            CaptureTarget("view2", np.array([0.4, -0.2, 0.5, 0.0, 0.0, 0.0])),
            CaptureTarget("view3", np.array([0.3,  0.0, 0.6, 0.0, 0.0, 0.0])),
        ]

        self.current_target_idx = 0
        self.current_joints = GEN72Config.SAFE_CONFIG.copy()
        self.state = "init"  # init -> moving -> capturing -> complete
        self.ik_attempts = 0
        self.max_ik_attempts = 5
        self.waiting_for_execution = False
        self.waiting_for_planning = False
        self.expected_execution_count = 0
        self.waiting_for_joint_update = False
        self.joint_update_wait_ticks = 0
        self.joint_update_max_ticks = 5

        # Camera configuration (configurable via environment variables)
        # CAPTURE_CAMERA_INDEX: camera index (default 0)
        # CAPTURE_OUTPUT_DIR: output directory (default ./captures)
        self.camera_index = int(os.getenv("CAPTURE_CAMERA_INDEX", "0"))
        self.output_dir = os.getenv("CAPTURE_OUTPUT_DIR", "captures")
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize camera immediately
        self.cap = None
        try:
            import cv2 as cv
            self.cap = cv.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                print(f"Camera {self.camera_index} initialized successfully")
                # Warm up camera by reading a few frames
                for _ in range(5):
                    self.cap.read()
                print("Camera warmed up and ready")
            else:
                print(f"Warning: Failed to open camera {self.camera_index}")
                self.cap.release()
                self.cap = None
        except ImportError:
            print("Warning: OpenCV not installed, camera capture disabled")

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

        if event_id == "joint_positions":
            # Update current joints from MuJoCo/Real Robot
            try:
                joints = event["value"].to_numpy()
                self.current_joints = joints[:7].copy()

                # If waiting for joint update, increment tick counter
                if self.waiting_for_joint_update:
                    self.joint_update_wait_ticks += 1
                    if self.joint_update_wait_ticks >= self.joint_update_max_ticks:
                        # Wait complete, continue to next target
                        self.waiting_for_joint_update = False
                        self.joint_update_wait_ticks = 0
                        print(f"  Joint state updated: {self.current_joints[:3]}...")
                        self._next_target(node)
            except Exception:
                pass
        elif event_id == "execution_status":
            # Check if trajectory execution completed
            self._handle_execution_status(node, event["value"])
        elif event_id == "ik_solution":
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
        # Ignore if already waiting for planning or execution
        if self.waiting_for_planning or self.waiting_for_execution:
            return

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
            # Mark as waiting for planning
            self.waiting_for_planning = True
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

    def _handle_execution_status(self, node: Node, data):
        """Handle trajectory execution status"""
        if not self.waiting_for_execution:
            return

        try:
            status_bytes = bytes(data.to_numpy())
            status = json.loads(status_bytes.decode('utf-8'))

            # Check if this is the expected execution and it completed
            exec_count = status.get("execution_count", 0)
            is_executing = status.get("is_executing", True)

            if exec_count == self.expected_execution_count and not is_executing:
                self.waiting_for_execution = False
                self._on_execution_complete(node)
        except Exception as e:
            print(f"[Capture] Error handling execution status: {e}")

    def _on_execution_complete(self, node: Node):
        """Called when trajectory execution completes"""
        print("  Execution complete!")

        # Check if we're returning home (final step)
        if self.state == "returning_home":
            print("\nReached home position. Workflow complete!")
            print("Staying idle at HOME position. Press Ctrl+C to exit.")
            self.state = "idle"
            return

        # Reached target position, capture image
        self._capture_image(node)

        # Move to next target
        self.current_target_idx += 1

        if self.current_target_idx >= len(self.targets):
            print("\nAll captures complete! Returning to home position...")
            self.state = "returning_home"
            self.waiting_for_planning = True
            # Send robot back to HOME position to stop motion
            self._return_to_home(node)
            return

        # Wait for joint state to update before planning next move
        print("  Waiting for joint state to update...")
        self.waiting_for_joint_update = True
        self.joint_update_wait_ticks = 0
        # _next_target(node) will be called in _handle_input() after wait completes

    def _handle_trajectory(self, node: Node, event):
        """Handle planned trajectory"""
        # Ignore if already waiting for execution
        if self.waiting_for_execution:
            return

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

        # Mark as waiting for execution to complete
        self.waiting_for_planning = False
        self.waiting_for_execution = True
        self.expected_execution_count += 1
        print(f"  Waiting for execution #{self.expected_execution_count} to complete...")


    # --------------------- Core Logic --------------------- #

    def _capture_image(self, node: Node):
        """Capture an image from camera and save to disk"""
        target = self.targets[self.current_target_idx]
        print("\nCapturing image at", target.name)
        print("   Position:", target.position[:3])

        if self.cap is None:
            print("   Camera not available, skipping capture")
            return

        try:
            import cv2 as cv
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
        except Exception as e:
            print(f"   Capture error: {e}")

    def _next_target(self, node: Node):
        """Move to next capture target"""
        if self.state == "complete":
            return

        self.ik_attempts = 0

        if self.current_target_idx >= len(self.targets):
            print("\nAll captures complete!")
            self.state = "complete"
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

    def _return_to_home(self, node: Node):
        """Return robot to home position after completing all captures"""
        home_config = GEN72Config.HOME_CONFIG
        print(f"Planning return to home: {home_config[:3]}...")
        self._request_plan(node, self.current_joints, home_config)
        # After this completes, the workflow will naturally end

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
