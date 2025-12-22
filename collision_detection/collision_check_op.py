#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Collision Check Operator for Dora-MoveIt
=========================================

Dora operator that provides collision checking as a service.
Uses collision_lib.py for core collision detection.

Inputs:
    - check_request: Joint positions to check for collision
    - scene_update: Updates to the planning scene (add/remove objects)
    
Outputs:
    - collision_result: JSON with collision status and details
    - distance_result: Minimum distance to obstacles

This operator maintains the collision checking state and can be used
by multiple other operators (planners, controllers, etc.)
"""

import json
import numpy as np
import pyarrow as pa
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from dora import Node

# Import collision library
from collision_detection.collision_lib import (
    CollisionChecker,
    CollisionObject,
    CollisionObjectType,
    RobotLink,
    create_sphere,
    create_box,
    create_cylinder,
    create_robot_link
)
from pointcloud_collision import PointCloudCollisionChecker
from config.robot_config import GEN72Config

# Realman SDK (optional - only if USE_REALMAN_API is True)
USE_REALMAN_API = False  # Set to True to use Realman official collision detection
try:
    from Robotic_Arm.rm_robot_interface import *
    REALMAN_AVAILABLE = True
except ImportError:
    REALMAN_AVAILABLE = False
    if USE_REALMAN_API:
        print("[Warning] Realman SDK not found, falling back to built-in collision checker")


@dataclass
class CollisionCheckRequest:
    """Request to check collision for a set of joint positions"""
    joint_positions: np.ndarray
    check_self_collision: bool = True
    check_environment: bool = True
    compute_distance: bool = False


class GEN72FK:
    """
    Forward kinematics for GEN72 robot using actual URDF transforms.

    Note: Returns joint frame positions which are used as collision sphere centers.
    Using sphere-based collision geometry avoids orientation issues that would
    occur with oriented cylinders.
    """

    def __init__(self):
        self.link_transforms = GEN72Config.LINK_TRANSFORMS

    def compute_link_transforms(self, joint_positions: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Compute collision geometry positions for all links.
        Returns positions suitable for collision checking (cylinder base centers).

        Args:
            joint_positions: Joint angles [7]

        Returns:
            Dictionary mapping link names to collision geometry base positions [x, y, z]
        """
        q = joint_positions

        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        T = np.eye(4)
        transforms = {}
        collision_geoms = GEN72Config.COLLISION_GEOMETRY

        # Base link - cylinder base at origin
        transforms["link0"] = np.array([0.0, 0.0, 0.0])

        for i, (joint_angle, link_tf) in enumerate(zip(q, self.link_transforms)):
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]

            # Build rotation from RPY
            R_link = rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            T = T @ T_link

            # Apply joint rotation
            T_joint = np.eye(4)
            T_joint[:3, :3] = rot_z(joint_angle)
            T = T @ T_joint

            # Store joint frame position as collision geometry center
            # (Using sphere approximations, so position = sphere center)
            transforms[f"link{i+1}"] = T[:3, 3].copy()

        return transforms


class CollisionCheckOperator:
    """
    Dora operator for collision checking.

    Maintains collision checking state and responds to collision queries.
    """

    def __init__(self, enable_pointcloud: bool = False, robot_ip: str = "192.168.1.18", robot_port: int = 8080):
        self.checker = CollisionChecker()
        self.fk = GEN72FK()
        self.check_count = 0
        self.last_scene_version = -1
        self.enable_pointcloud = enable_pointcloud

        # Initialize Realman robot connection if using official API
        self.realman_handle = None
        if USE_REALMAN_API and REALMAN_AVAILABLE:
            try:
                self.realman_handle = rm_create_robot_arm(robot_ip, robot_port)
                if self.realman_handle:
                    print(f"[Collision] Connected to Realman robot at {robot_ip}:{robot_port}")
                else:
                    print("[Collision] Failed to connect to Realman robot, using built-in checker")
            except Exception as e:
                print(f"[Collision] Realman connection error: {e}, using built-in checker")

        # Initialize point cloud collision checker (interface ready, disabled by default)
        if enable_pointcloud:
            self.pointcloud_checker = PointCloudCollisionChecker(
                safety_margin=0.05,
                voxel_size=0.01
            )
        else:
            self.pointcloud_checker = None

        # Initialize robot links
        self._setup_robot_collision_model()

        print("Collision check operator initialized")
        print(f"  Robot links: {len(self.checker.robot_links)}")
        print(f"  Environment objects: {len(self.checker.environment_objects)}")
        print(f"  Point cloud collision: {'enabled' if enable_pointcloud else 'disabled (interface ready)'}")
        print(f"  Self-collision method: {'Realman API' if self.realman_handle else 'Built-in'}")
        
    def _setup_robot_collision_model(self):
        """Set up robot collision geometries - using GEN72Config parameters"""
        # Use collision geometry from robot_config.py
        collision_geoms = GEN72Config.COLLISION_GEOMETRY
        links = []
        for i, (geom_type, dimensions) in enumerate(collision_geoms):
            obj_type = CollisionObjectType.CYLINDER if geom_type == "cylinder" else CollisionObjectType.SPHERE
            # Use zero padding for self-collision (padding only for environment collision)
            links.append(create_robot_link(f"link{i}", obj_type, np.array(dimensions), i, padding=0.0))
        self.checker.set_robot_links(links)
        
    def add_environment_object(self, obj_data: dict):
        """
        Add an object to the environment.
        
        Args:
            obj_data: Dictionary with object specification:
                - name: Object name
                - type: "sphere", "box", "cylinder"
                - position: [x, y, z]
                - dimensions: Type-specific dimensions
                - padding: Optional safety margin
        """
        obj_type_map = {
            "sphere": CollisionObjectType.SPHERE,
            "box": CollisionObjectType.BOX,
            "cylinder": CollisionObjectType.CYLINDER
        }
        
        name = obj_data["name"]
        obj_type = obj_type_map.get(obj_data["type"], CollisionObjectType.SPHERE)
        position = np.array(obj_data["position"])
        dimensions = np.array(obj_data["dimensions"])
        padding = obj_data.get("padding", 0.0)
        
        pose = np.zeros(7)
        pose[:3] = position
        pose[3] = 1.0  # qw
        
        obj = CollisionObject(
            name=name,
            obj_type=obj_type,
            pose=pose,
            dimensions=dimensions,
            padding=padding
        )
        
        self.checker.add_environment_object(obj)
        print(f"[Collision] Added object: {name} ({obj_data['type']})")
        
    def remove_environment_object(self, name: str) -> bool:
        """Remove an object from the environment"""
        removed = self.checker.remove_environment_object(name)
        if removed:
            print(f"[Collision] Removed object: {name}")
        return removed
        
    def clear_environment(self):
        """Clear all environment objects"""
        self.checker.clear_environment()
        print("[Collision] Cleared all environment objects")

    def update_pointcloud(self, points: np.ndarray):
        """
        Update point cloud from 3D LiDAR.

        Args:
            points: Point cloud array [N x 3] in robot base frame
        """
        if self.pointcloud_checker is not None:
            self.pointcloud_checker.set_point_cloud(points)
            print(f"[Collision] Updated point cloud: {len(points)} points")
        
    def check_collision(self, request: CollisionCheckRequest) -> dict:
        """
        Check if a robot configuration is in collision.

        Args:
            request: Collision check request

        Returns:
            Dictionary with collision result
        """
        self.check_count += 1

        # Compute link transforms from joint positions
        link_transforms = self.fk.compute_link_transforms(request.joint_positions)

        # Check self-collision using Realman API if available
        self_collision = False
        collision_result = None

        if request.check_self_collision:
            if self.realman_handle:
                # Use Realman official API for self-collision
                try:
                    ret = rm_get_joint_collision(self.realman_handle, request.joint_positions.tolist())
                    if isinstance(ret, tuple) and len(ret) >= 2:
                        self_collision = (ret[1] == 1)  # 1 means collision detected
                except Exception as e:
                    print(f"[Collision] Realman API error: {e}, using built-in checker")
                    is_valid, collision_result = self.checker.is_state_valid(
                        link_transforms, check_self=True, check_environment=False
                    )
                    self_collision = not is_valid
            else:
                # Use built-in self-collision checker
                is_valid, collision_result = self.checker.is_state_valid(
                    link_transforms, check_self=True, check_environment=False
                )
                self_collision = not is_valid

        # Check environment collision (always use built-in checker)
        env_collision = False
        if request.check_environment:
            is_valid, env_result = self.checker.is_state_valid(
                link_transforms, check_self=False, check_environment=True
            )
            env_collision = not is_valid
            if env_collision and env_result:
                collision_result = env_result

        result = {
            "check_id": self.check_count,
            "in_collision": self_collision or env_collision,
            "joint_positions": request.joint_positions.tolist()
        }

        if (self_collision or env_collision) and collision_result:
            result["collision_info"] = {
                "object_a": collision_result.object_a,
                "object_b": collision_result.object_b,
                "penetration_depth": float(collision_result.penetration_depth),
                "type": "self-collision" if self_collision else "environment"
            }
            if collision_result.contact_points:
                result["collision_info"]["contact_point"] = collision_result.contact_points[0].tolist()

        # Check collision with point cloud (if enabled)
        if self.pointcloud_checker is not None:
            pc_collision, pc_result = self.pointcloud_checker.check_robot_collision(
                link_transforms,
                GEN72Config.COLLISION_GEOMETRY
            )

            if pc_collision:
                result["in_collision"] = True
                if "collision_info" not in result:
                    result["collision_info"] = {}
                result["collision_info"]["pointcloud_collision"] = {
                    "link": pc_result.link_name,
                    "distance": float(pc_result.min_distance),
                    "closest_point": pc_result.closest_point.tolist() if pc_result.closest_point is not None else None,
                    "type": "pointcloud"
                }

        # Compute distance if requested
        if request.compute_distance:
            min_dist = self.checker.get_minimum_distance(link_transforms)
            result["min_distance"] = float(min_dist)
            if self.pointcloud_checker is not None:
                pc_collision, pc_result = self.pointcloud_checker.check_robot_collision(
                    link_transforms,
                    GEN72Config.COLLISION_GEOMETRY
                )
                if pc_result and pc_result.min_distance < min_dist:
                    result["min_distance"] = float(pc_result.min_distance)

        return result
    
    def process_scene_update(self, update_data: dict):
        """
        Process a scene update.

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
            self.clear_environment()
            for obj_data in update_data.get("world_objects", []):
                self.add_environment_object(obj_data)
            return

        # Otherwise handle as single object command
        action = update_data.get("action", "add")

        if action == "add":
            if "object" in update_data:
                self.add_environment_object(update_data["object"])
        elif action == "remove":
            self.remove_environment_object(update_data["name"])
        elif action == "clear":
            self.clear_environment()
        else:
            print(f"[Collision] Unknown action: {action}")


def main():
    """Main entry point for Dora collision check operator"""
    print("=== Dora-MoveIt Collision Check Operator ===")

    node = Node()
    # Point cloud disabled by default, set enable_pointcloud=True when ready
    collision_op = CollisionCheckOperator(enable_pointcloud=False)
    
    # Add some default environment objects
    collision_op.add_environment_object({
        "name": "table",
        "type": "box",
        "position": [0.5, 0.0, 0.4],
        "dimensions": [0.6, 0.8, 0.02]
    })
    
    collision_op.add_environment_object({
        "name": "ground",
        "type": "box",
        "position": [0.0, 0.0, -0.01],
        "dimensions": [2.0, 2.0, 0.02]
    })
    
    print(f"Environment has {len(collision_op.checker.environment_objects)} objects")
    print("Collision checker ready, waiting for requests...")
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "check_request":
                # Parse joint positions
                try:
                    value = event["value"]
                    if hasattr(value, 'to_numpy'):
                        joint_pos = value.to_numpy()
                    else:
                        joint_pos = np.frombuffer(bytes(value), dtype=np.float32)
                    
                    request = CollisionCheckRequest(
                        joint_positions=joint_pos,
                        check_self_collision=True,
                        check_environment=True,
                        compute_distance=True
                    )
                    
                    result = collision_op.check_collision(request)
                    
                    # Send result
                    result_bytes = json.dumps(result).encode('utf-8')
                    node.send_output(
                        "collision_result",
                        pa.array(list(result_bytes), type=pa.uint8()),
                        metadata={"in_collision": result["in_collision"]}
                    )
                    
                    status = "COLLISION" if result["in_collision"] else "CLEAR"
                    print(f"[Check #{result['check_id']}] {status}", end="")
                    if "min_distance" in result:
                        print(f" (dist={result['min_distance']:.4f}m)")
                    else:
                        print()
                        
                except Exception as e:
                    print(f"[Collision] Error processing request: {e}")
                    
            elif input_id == "scene_update":
                # Parse scene update
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        update_bytes = bytes(value.to_pylist())
                    else:
                        update_bytes = bytes(value)

                    update_data = json.loads(update_bytes.decode('utf-8'))
                    collision_op.process_scene_update(update_data)

                    # Send updated scene info
                    scene_info = {
                        "num_objects": len(collision_op.checker.environment_objects),
                        "object_names": [obj.name for obj in collision_op.checker.environment_objects]
                    }
                    scene_bytes = json.dumps(scene_info).encode('utf-8')
                    node.send_output(
                        "scene_info",
                        pa.array(list(scene_bytes), type=pa.uint8())
                    )

                except Exception as e:
                    print(f"[Collision] Error processing scene update: {e}")

            elif input_id == "pointcloud":
                # Update point cloud from LiDAR
                try:
                    value = event["value"]
                    if hasattr(value, 'to_numpy'):
                        points = value.to_numpy().reshape(-1, 3)
                    else:
                        points = np.frombuffer(bytes(value), dtype=np.float32).reshape(-1, 3)

                    collision_op.update_pointcloud(points)

                except Exception as e:
                    print(f"[Collision] Error processing point cloud: {e}")
                    
        elif event_type == "STOP":
            print("Collision check operator stopping...")
            break


if __name__ == "__main__":
    main()

