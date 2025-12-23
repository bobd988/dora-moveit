#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Planning Scene Operator for Dora-MoveIt
========================================

Central scene management operator similar to MoveIt's PlanningScene.
Acts as the single source of truth for:
- Environment objects (obstacles, tables, etc.)
- Robot state (current joint positions)
- Attached objects (objects held by the gripper)

Responsibilities:
1. Maintain world state
2. Broadcast scene updates to all interested operators
3. Handle scene queries
4. Manage attached objects (pick/place)

This is the "brain" that coordinates all other operators.

Inputs:
    - robot_state: Current robot joint positions
    - add_object: Add collision object to scene
    - remove_object: Remove object from scene
    - attach_object: Attach object to robot
    - detach_object: Detach object from robot
    - get_scene: Request current scene state
    
Outputs:
    - scene_update: Broadcasted scene changes
    - scene_state: Current scene state (on request)
    - planning_request: Forwarded planning requests
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import Dict, List, Optional, Any, Set
from dataclasses import dataclass, field
from enum import Enum
from dora import Node

from collision_detection.collision_lib import (
    CollisionObject,
    CollisionObjectType,
    create_sphere,
    create_box,
    create_cylinder
)


class ObjectState(Enum):
    """State of an object in the scene"""
    WORLD = "world"  # Object is in the world
    ATTACHED = "attached"  # Object is attached to robot


@dataclass
class SceneObject:
    """Object in the planning scene"""
    name: str
    obj_type: str  # "sphere", "box", "cylinder"
    position: np.ndarray
    dimensions: np.ndarray
    state: ObjectState = ObjectState.WORLD
    attached_link: Optional[str] = None
    color: List[float] = field(default_factory=lambda: [0.5, 0.5, 0.5, 1.0])


@dataclass
class RobotState:
    """Current robot state"""
    joint_positions: np.ndarray
    joint_velocities: Optional[np.ndarray] = None
    gripper_state: float = 0.0  # 0=open, 1=closed
    timestamp: float = 0.0


@dataclass
class PlanningSceneState:
    """Complete planning scene state"""
    robot_state: Optional[RobotState]
    world_objects: Dict[str, SceneObject]
    attached_objects: Dict[str, SceneObject]
    allowed_collision_pairs: Set[tuple]
    timestamp: float


class PlanningSceneOperator:
    """
    Central planning scene manager.
    
    Similar to MoveIt's PlanningScene, this operator:
    - Maintains the world state
    - Tracks robot state
    - Manages attached objects
    - Broadcasts updates to other operators
    """
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        
        # Robot state
        self.robot_state = RobotState(
            joint_positions=np.zeros(num_joints),
            joint_velocities=np.zeros(num_joints),
            timestamp=time.time()
        )
        
        # Scene objects
        self.world_objects: Dict[str, SceneObject] = {}
        self.attached_objects: Dict[str, SceneObject] = {}
        
        # Allowed collision pairs (link_a, object_b)
        self.allowed_collision_pairs: Set[tuple] = set()
        
        # Scene version for change tracking
        self.scene_version = 0
        
        # Statistics
        self.update_count = 0
        
        # Initialize with default scene
        self._setup_default_scene()
        
        print("Planning Scene operator initialized")
        
    def _setup_default_scene(self):
        """Set up default scene with ground plane"""
        # Ground plane - placed lower to avoid collision with robot base
        self.add_object(SceneObject(
            name="ground",
            obj_type="box",
            position=np.array([0.0, 0.0, -3.0]),
            dimensions=np.array([2.0, 2.0, 0.02]),
            color=[0.3, 0.3, 0.3, 1.0]
        ))

        # # Table moved further away and lower to avoid collisions
        # self.add_object(SceneObject(
        #     name="table",
        #     obj_type="box",
        #     position=np.array([0.6, 0.0, 0.3]),
        #     dimensions=np.array([0.4, 0.6, 0.02]),
        #     color=[0.6, 0.4, 0.2, 1.0]
        # ))

        print(f"  Default scene: {len(self.world_objects)} objects")
        
    def add_object(self, obj: SceneObject) -> bool:
        """
        Add an object to the world.
        
        Args:
            obj: Scene object to add
            
        Returns:
            True if added successfully
        """
        if obj.name in self.world_objects or obj.name in self.attached_objects:
            print(f"[Scene] Object '{obj.name}' already exists")
            return False
        
        obj.state = ObjectState.WORLD
        self.world_objects[obj.name] = obj
        self.scene_version += 1
        self.update_count += 1
        
        print(f"[Scene] Added object: {obj.name} ({obj.obj_type})")
        return True
    
    def remove_object(self, name: str) -> bool:
        """
        Remove an object from the scene.
        
        Args:
            name: Object name
            
        Returns:
            True if removed successfully
        """
        if name in self.world_objects:
            del self.world_objects[name]
            self.scene_version += 1
            self.update_count += 1
            print(f"[Scene] Removed object: {name}")
            return True
        elif name in self.attached_objects:
            del self.attached_objects[name]
            self.scene_version += 1
            self.update_count += 1
            print(f"[Scene] Removed attached object: {name}")
            return True
        
        print(f"[Scene] Object '{name}' not found")
        return False
    
    def attach_object(self, name: str, link: str = "link7") -> bool:
        """
        Attach an object to the robot (pick).
        
        Args:
            name: Object name
            link: Robot link to attach to
            
        Returns:
            True if attached successfully
        """
        if name not in self.world_objects:
            print(f"[Scene] Cannot attach: object '{name}' not in world")
            return False
        
        obj = self.world_objects.pop(name)
        obj.state = ObjectState.ATTACHED
        obj.attached_link = link
        self.attached_objects[name] = obj
        
        # Allow collision between attached object and gripper
        self.allowed_collision_pairs.add((link, name))
        self.allowed_collision_pairs.add(("link6", name))
        
        self.scene_version += 1
        self.update_count += 1
        
        print(f"[Scene] Attached '{name}' to {link}")
        return True
    
    def detach_object(self, name: str, position: Optional[np.ndarray] = None) -> bool:
        """
        Detach an object from the robot (place).
        
        Args:
            name: Object name
            position: New world position (if None, keeps current position)
            
        Returns:
            True if detached successfully
        """
        if name not in self.attached_objects:
            print(f"[Scene] Cannot detach: object '{name}' not attached")
            return False
        
        obj = self.attached_objects.pop(name)
        obj.state = ObjectState.WORLD
        
        if position is not None:
            obj.position = position
        
        # Remove from allowed collisions
        self.allowed_collision_pairs = {
            pair for pair in self.allowed_collision_pairs
            if name not in pair
        }
        
        obj.attached_link = None
        self.world_objects[name] = obj
        
        self.scene_version += 1
        self.update_count += 1
        
        print(f"[Scene] Detached '{name}' to world at {obj.position}")
        return True
    
    def update_robot_state(self, joint_positions: np.ndarray, 
                          joint_velocities: Optional[np.ndarray] = None,
                          gripper_state: Optional[float] = None):
        """
        Update the current robot state.
        
        Args:
            joint_positions: Joint positions
            joint_velocities: Optional joint velocities
            gripper_state: Optional gripper state (0=open, 1=closed)
        """
        self.robot_state.joint_positions = joint_positions.copy()
        self.robot_state.timestamp = time.time()
        
        if joint_velocities is not None:
            self.robot_state.joint_velocities = joint_velocities.copy()
        
        if gripper_state is not None:
            self.robot_state.gripper_state = gripper_state
    
    def get_scene_state(self) -> PlanningSceneState:
        """Get the complete current scene state"""
        return PlanningSceneState(
            robot_state=self.robot_state,
            world_objects=self.world_objects.copy(),
            attached_objects=self.attached_objects.copy(),
            allowed_collision_pairs=self.allowed_collision_pairs.copy(),
            timestamp=time.time()
        )
    
    def get_scene_update_message(self) -> dict:
        """
        Get scene update message to broadcast.
        
        Returns:
            Dictionary with scene update for other operators
        """
        world_objects_list = []
        for obj in self.world_objects.values():
            world_objects_list.append({
                "name": obj.name,
                "type": obj.obj_type,
                "position": obj.position.tolist(),
                "dimensions": obj.dimensions.tolist(),
                "color": obj.color
            })
        
        attached_objects_list = []
        for obj in self.attached_objects.values():
            attached_objects_list.append({
                "name": obj.name,
                "type": obj.obj_type,
                "position": obj.position.tolist(),
                "dimensions": obj.dimensions.tolist(),
                "attached_link": obj.attached_link
            })
        
        return {
            "version": self.scene_version,
            "timestamp": time.time(),
            "world_objects": world_objects_list,
            "attached_objects": attached_objects_list,
            "robot_state": {
                "joint_positions": self.robot_state.joint_positions.tolist(),
                "gripper_state": self.robot_state.gripper_state
            }
        }
    
    def process_command(self, command: dict) -> dict:
        """
        Process a scene command.
        
        Args:
            command: Command dictionary with 'action' and parameters
            
        Returns:
            Result dictionary
        """
        action = command.get("action", "")
        result = {"success": False, "action": action}
        
        if action == "add":
            obj_data = command.get("object", {})
            obj = SceneObject(
                name=obj_data.get("name", f"object_{len(self.world_objects)}"),
                obj_type=obj_data.get("type", "box"),
                position=np.array(obj_data.get("position", [0, 0, 0])),
                dimensions=np.array(obj_data.get("dimensions", [0.1, 0.1, 0.1])),
                color=obj_data.get("color", [0.5, 0.5, 0.5, 1.0])
            )
            result["success"] = self.add_object(obj)
            
        elif action == "remove":
            name = command.get("name", "")
            result["success"] = self.remove_object(name)
            
        elif action == "attach":
            name = command.get("name", "")
            link = command.get("link", "link7")
            result["success"] = self.attach_object(name, link)
            
        elif action == "detach":
            name = command.get("name", "")
            position = command.get("position")
            if position:
                position = np.array(position)
            result["success"] = self.detach_object(name, position)
            
        elif action == "clear":
            self.world_objects.clear()
            self.attached_objects.clear()
            self._setup_default_scene()
            result["success"] = True
            
        elif action == "get_state":
            result["success"] = True
            result["state"] = self.get_scene_update_message()
        
        else:
            result["error"] = f"Unknown action: {action}"
        
        result["scene_version"] = self.scene_version
        return result


def main():
    """Main entry point for Dora Planning Scene operator"""
    print("=== Dora-MoveIt Planning Scene Operator ===")
    
    node = Node()
    scene_op = PlanningSceneOperator(num_joints=7)
    
    # Broadcast initial scene
    print("Broadcasting initial scene state...")
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "robot_state":
                # Update robot state
                try:
                    joints = event["value"].to_numpy()
                    # Extract arm joints: skip freejoint (7) + wheels (6) = 13
                    if len(joints) >= 20:
                        arm_joints = joints[13:20]
                    else:
                        arm_joints = joints[:7]
                    scene_op.update_robot_state(arm_joints)
                except Exception as e:
                    print(f"[Scene] Robot state error: {e}")
                    
            elif input_id == "scene_command":
                # Process scene command
                try:
                    value = event["value"]
                    if hasattr(value, 'to_pylist'):
                        cmd_bytes = bytes(value.to_pylist())
                    else:
                        cmd_bytes = bytes(value)
                    
                    command = json.loads(cmd_bytes.decode('utf-8'))
                    result = scene_op.process_command(command)
                    
                    # Send result
                    result_bytes = json.dumps(result).encode('utf-8')
                    node.send_output(
                        "command_result",
                        pa.array(list(result_bytes), type=pa.uint8())
                    )
                    
                    # Broadcast scene update if scene changed
                    if result.get("success") and command.get("action") != "get_state":
                        update = scene_op.get_scene_update_message()
                        update_bytes = json.dumps(update).encode('utf-8')
                        node.send_output(
                            "scene_update",
                            pa.array(list(update_bytes), type=pa.uint8()),
                            metadata={"version": scene_op.scene_version}
                        )
                        
                except Exception as e:
                    print(f"[Scene] Command error: {e}")
                    import traceback
                    traceback.print_exc()
                    
            elif input_id == "tick":
                # Periodic scene broadcast
                update = scene_op.get_scene_update_message()
                update_bytes = json.dumps(update).encode('utf-8')
                node.send_output(
                    "scene_update",
                    pa.array(list(update_bytes), type=pa.uint8()),
                    metadata={"version": scene_op.scene_version}
                )
                    
        elif event_type == "STOP":
            print("Planning Scene operator stopping...")
            break
    
    print(f"Total scene updates: {scene_op.update_count}")


if __name__ == "__main__":
    main()

