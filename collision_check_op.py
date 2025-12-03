#!/usr/bin/env python3
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
from collision_lib import (
    CollisionChecker, 
    CollisionObject, 
    CollisionObjectType,
    RobotLink,
    create_sphere,
    create_box,
    create_cylinder,
    create_robot_link
)


@dataclass
class CollisionCheckRequest:
    """Request to check collision for a set of joint positions"""
    joint_positions: np.ndarray
    check_self_collision: bool = True
    check_environment: bool = True
    compute_distance: bool = False


class SimpleFK:
    """
    Simple forward kinematics to compute link positions from joint angles.
    In production, use proper FK from URDF.
    """
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        # Link lengths (Panda-like robot)
        self.link_lengths = [0.333, 0.0, 0.316, 0.0825, 0.384, 0.0, 0.107]
        
    def compute_link_transforms(self, joint_positions: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Compute positions of all links given joint positions.
        
        Args:
            joint_positions: Joint angles [7]
            
        Returns:
            Dictionary mapping link names to positions [x, y, z]
        """
        q = joint_positions
        transforms = {}
        
        # Base link
        transforms["link0"] = np.array([0.0, 0.0, 0.15])
        
        # Simplified chain (approximate positions)
        c1, s1 = np.cos(q[0]), np.sin(q[0])
        c2, s2 = np.cos(q[1]), np.sin(q[1])
        c3, s3 = np.cos(q[2]), np.sin(q[2])
        c4, s4 = np.cos(q[3]), np.sin(q[3])
        c5, s5 = np.cos(q[4]), np.sin(q[4])
        c6, s6 = np.cos(q[5]), np.sin(q[5])
        
        # Link 1 (after joint 1)
        transforms["link1"] = np.array([
            0.0,
            0.0,
            0.333
        ])
        
        # Link 2 (after joint 2)
        l1 = 0.316
        transforms["link2"] = np.array([
            l1 * s2 * c1,
            l1 * s2 * s1,
            0.333 + l1 * c2
        ])
        
        # Link 3 (after joint 3)
        transforms["link3"] = transforms["link2"] + np.array([
            0.0825 * c1,
            0.0825 * s1,
            0.0
        ])
        
        # Link 4 (after joint 4)
        l2 = 0.384
        transforms["link4"] = transforms["link3"] + np.array([
            l2 * s4 * c1,
            l2 * s4 * s1,
            l2 * c4
        ])
        
        # Link 5 (after joint 5)
        transforms["link5"] = transforms["link4"] + np.array([
            0.0 * c1,
            0.0 * s1,
            0.0
        ])
        
        # Link 6 (after joint 6) - wrist
        l3 = 0.088
        transforms["link6"] = transforms["link5"] + np.array([
            l3 * c6 * c1,
            l3 * c6 * s1,
            l3 * s6
        ])
        
        # End effector / gripper
        transforms["link7"] = transforms["link6"] + np.array([
            0.107 * c1,
            0.107 * s1,
            0.0
        ])
        
        return transforms


class CollisionCheckOperator:
    """
    Dora operator for collision checking.

    Maintains collision checking state and responds to collision queries.
    """

    def __init__(self):
        self.checker = CollisionChecker()
        self.fk = SimpleFK()
        self.check_count = 0
        self.last_scene_version = -1  # Track scene version to avoid redundant updates

        # Initialize robot links
        self._setup_robot_collision_model()

        print("Collision check operator initialized")
        print(f"  Robot links: {len(self.checker.robot_links)}")
        print(f"  Environment objects: {len(self.checker.environment_objects)}")
        
    def _setup_robot_collision_model(self):
        """Set up robot collision geometries"""
        links = [
            create_robot_link("link0", CollisionObjectType.CYLINDER, np.array([0.06, 0.15]), 0),
            create_robot_link("link1", CollisionObjectType.CYLINDER, np.array([0.06, 0.12]), 1),
            create_robot_link("link2", CollisionObjectType.CYLINDER, np.array([0.05, 0.15]), 2),
            create_robot_link("link3", CollisionObjectType.CYLINDER, np.array([0.05, 0.12]), 3),
            create_robot_link("link4", CollisionObjectType.CYLINDER, np.array([0.045, 0.15]), 4),
            create_robot_link("link5", CollisionObjectType.CYLINDER, np.array([0.04, 0.08]), 5),
            create_robot_link("link6", CollisionObjectType.SPHERE, np.array([0.04]), 6),
            create_robot_link("link7", CollisionObjectType.SPHERE, np.array([0.05]), 7),  # Gripper
        ]
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
        
        # Check collision
        is_valid, collision_result = self.checker.is_state_valid(
            link_transforms,
            check_self=request.check_self_collision,
            check_environment=request.check_environment
        )
        
        result = {
            "check_id": self.check_count,
            "in_collision": not is_valid,
            "joint_positions": request.joint_positions.tolist()
        }
        
        if not is_valid and collision_result:
            result["collision_info"] = {
                "object_a": collision_result.object_a,
                "object_b": collision_result.object_b,
                "penetration_depth": float(collision_result.penetration_depth)
            }
            if collision_result.contact_points:
                result["collision_info"]["contact_point"] = collision_result.contact_points[0].tolist()
        
        # Compute distance if requested
        if request.compute_distance:
            min_dist = self.checker.get_minimum_distance(link_transforms)
            result["min_distance"] = float(min_dist)
        
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
    collision_op = CollisionCheckOperator()
    
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
                    
        elif event_type == "STOP":
            print("Collision check operator stopping...")
            break


if __name__ == "__main__":
    main()

