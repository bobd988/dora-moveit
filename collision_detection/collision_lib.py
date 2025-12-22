#!/usr/bin/env python3
"""
Collision Detection Library for Dora-MoveIt
============================================

Core collision detection functions that can be reused by multiple operators.
Provides geometric collision checking for robot motion planning.

Supported collision objects:
- Spheres
- Boxes (AABB)
- Cylinders
- Mesh (simplified convex hull)

Features:
- Robot self-collision detection
- Environment collision detection
- Distance queries
- Collision margins/padding
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum


class CollisionObjectType(Enum):
    """Types of collision objects supported"""
    SPHERE = "sphere"
    BOX = "box"
    CYLINDER = "cylinder"
    CAPSULE = "capsule"


@dataclass
class CollisionObject:
    """Represents a collision object in the scene"""
    name: str
    obj_type: CollisionObjectType
    pose: np.ndarray  # [x, y, z, qw, qx, qy, qz] or [x, y, z] for position only
    dimensions: np.ndarray  # Depends on type: sphere=[r], box=[x,y,z], cylinder=[r,h]
    padding: float = 0.0  # Safety margin
    
    def get_position(self) -> np.ndarray:
        """Get position [x, y, z]"""
        return self.pose[:3]
    
    def get_padded_dimensions(self) -> np.ndarray:
        """Get dimensions with padding applied"""
        return self.dimensions + self.padding


@dataclass
class RobotLink:
    """Represents a robot link for collision checking"""
    name: str
    collision_geometry: CollisionObject
    parent_joint_idx: int = -1
    

@dataclass 
class CollisionResult:
    """Result of a collision check"""
    in_collision: bool
    contact_points: List[np.ndarray] = field(default_factory=list)
    penetration_depth: float = 0.0
    object_a: str = ""
    object_b: str = ""
    min_distance: float = float('inf')


class CollisionChecker:
    """
    Core collision checking class.
    Provides methods for checking collisions between various geometric primitives.
    """
    
    def __init__(self, robot_links: Optional[List[RobotLink]] = None):
        """
        Initialize collision checker.
        
        Args:
            robot_links: List of robot links with collision geometries
        """
        self.robot_links = robot_links or []
        self.environment_objects: List[CollisionObject] = []
        self.self_collision_pairs: List[Tuple[int, int]] = []
        self.collision_margin = 0.01  # 1cm default margin
        
    def add_environment_object(self, obj: CollisionObject):
        """Add an object to the environment"""
        self.environment_objects.append(obj)
        
    def remove_environment_object(self, name: str) -> bool:
        """Remove an object from the environment by name"""
        for i, obj in enumerate(self.environment_objects):
            if obj.name == name:
                self.environment_objects.pop(i)
                return True
        return False
    
    def clear_environment(self):
        """Clear all environment objects"""
        self.environment_objects.clear()
        
    def set_robot_links(self, links: List[RobotLink]):
        """Set robot links for self-collision checking"""
        self.robot_links = links
        self._compute_self_collision_pairs()
        
    def _compute_self_collision_pairs(self):
        """Compute which link pairs to check for self-collision"""
        self.self_collision_pairs = []
        n = len(self.robot_links)
        for i in range(n):
            for j in range(i + 3, n):  # Skip 2 adjacent links in kinematic chain
                self.self_collision_pairs.append((i, j))
    
    # ==================== Primitive Collision Functions ====================
    
    @staticmethod
    def sphere_sphere_collision(
        pos1: np.ndarray, r1: float,
        pos2: np.ndarray, r2: float,
        margin: float = 0.0
    ) -> CollisionResult:
        """
        Check collision between two spheres.
        
        Args:
            pos1: Center of sphere 1 [x, y, z]
            r1: Radius of sphere 1
            pos2: Center of sphere 2 [x, y, z]
            r2: Radius of sphere 2
            margin: Safety margin
            
        Returns:
            CollisionResult with collision information
        """
        diff = pos2 - pos1
        dist = np.linalg.norm(diff)
        combined_radius = r1 + r2 + margin
        
        result = CollisionResult(in_collision=False)
        result.min_distance = dist - (r1 + r2)
        
        if dist < combined_radius:
            result.in_collision = True
            result.penetration_depth = combined_radius - dist
            if dist > 1e-6:
                # Contact point is on the surface between the two spheres
                normal = diff / dist
                contact = pos1 + normal * r1
                result.contact_points = [contact]
        
        return result
    
    @staticmethod
    def sphere_box_collision(
        sphere_pos: np.ndarray, sphere_r: float,
        box_pos: np.ndarray, box_half_extents: np.ndarray,
        margin: float = 0.0
    ) -> CollisionResult:
        """
        Check collision between a sphere and an axis-aligned box.
        
        Args:
            sphere_pos: Center of sphere [x, y, z]
            sphere_r: Radius of sphere
            box_pos: Center of box [x, y, z]
            box_half_extents: Half dimensions of box [hx, hy, hz]
            margin: Safety margin
            
        Returns:
            CollisionResult with collision information
        """
        # Find closest point on box to sphere center
        box_min = box_pos - box_half_extents
        box_max = box_pos + box_half_extents
        
        closest_point = np.clip(sphere_pos, box_min, box_max)
        
        diff = sphere_pos - closest_point
        dist = np.linalg.norm(diff)
        
        result = CollisionResult(in_collision=False)
        result.min_distance = dist - sphere_r
        
        effective_radius = sphere_r + margin
        
        if dist < effective_radius:
            result.in_collision = True
            result.penetration_depth = effective_radius - dist
            result.contact_points = [closest_point]
        
        return result
    
    @staticmethod
    def box_box_collision(
        pos1: np.ndarray, half_extents1: np.ndarray,
        pos2: np.ndarray, half_extents2: np.ndarray,
        margin: float = 0.0
    ) -> CollisionResult:
        """
        Check collision between two axis-aligned boxes (AABB).
        
        Args:
            pos1: Center of box 1 [x, y, z]
            half_extents1: Half dimensions of box 1 [hx, hy, hz]
            pos2: Center of box 2 [x, y, z]
            half_extents2: Half dimensions of box 2 [hx, hy, hz]
            margin: Safety margin
            
        Returns:
            CollisionResult with collision information
        """
        # Compute min/max for each box
        min1 = pos1 - half_extents1 - margin
        max1 = pos1 + half_extents1 + margin
        min2 = pos2 - half_extents2
        max2 = pos2 + half_extents2
        
        result = CollisionResult(in_collision=False)
        
        # Check for overlap on all axes
        overlap = np.all(max1 >= min2) and np.all(max2 >= min1)
        
        if overlap:
            result.in_collision = True
            # Compute penetration depth (minimum overlap on any axis)
            overlap_min = np.minimum(max1, max2)
            overlap_max = np.maximum(min1, min2)
            penetration = overlap_min - overlap_max
            result.penetration_depth = np.min(penetration)
            # Contact point at center of overlap region
            result.contact_points = [(overlap_min + overlap_max) / 2]
        else:
            # Compute minimum distance
            gaps = np.maximum(min2 - max1, min1 - max2)
            gaps = np.maximum(gaps, 0)
            result.min_distance = np.linalg.norm(gaps)
        
        return result
    
    @staticmethod
    def sphere_cylinder_collision(
        sphere_pos: np.ndarray, sphere_r: float,
        cyl_pos: np.ndarray, cyl_r: float, cyl_h: float,
        margin: float = 0.0
    ) -> CollisionResult:
        """
        Check collision between a sphere and a vertical cylinder.

        Args:
            sphere_pos: Center of sphere [x, y, z]
            sphere_r: Radius of sphere
            cyl_pos: Center of cylinder base [x, y, z]
            cyl_r: Radius of cylinder
            cyl_h: Height of cylinder
            margin: Safety margin

        Returns:
            CollisionResult with collision information
        """
        result = CollisionResult(in_collision=False)

        # Project sphere center onto cylinder axis
        cyl_center = cyl_pos.copy()
        cyl_center[2] += cyl_h / 2  # Center of cylinder

        # Clamp z to cylinder height
        z_clamped = np.clip(sphere_pos[2], cyl_pos[2], cyl_pos[2] + cyl_h)

        # Distance in xy plane
        xy_diff = sphere_pos[:2] - cyl_pos[:2]
        xy_dist = np.linalg.norm(xy_diff)

        # Closest point on cylinder surface
        if xy_dist > 1e-6:
            closest_xy = cyl_pos[:2] + xy_diff / xy_dist * min(xy_dist, cyl_r)
        else:
            closest_xy = cyl_pos[:2]

        closest_point = np.array([closest_xy[0], closest_xy[1], z_clamped])

        diff = sphere_pos - closest_point
        dist = np.linalg.norm(diff)

        result.min_distance = dist - sphere_r
        effective_radius = sphere_r + margin

        if dist < effective_radius:
            result.in_collision = True
            result.penetration_depth = effective_radius - dist
            result.contact_points = [closest_point]

        return result

    @staticmethod
    def cylinder_cylinder_collision(
        cyl1_pos: np.ndarray, cyl1_r: float, cyl1_h: float,
        cyl2_pos: np.ndarray, cyl2_r: float, cyl2_h: float,
        margin: float = 0.0
    ) -> CollisionResult:
        """
        Check collision between two vertical cylinders.

        Args:
            cyl1_pos: Center of cylinder 1 base [x, y, z]
            cyl1_r: Radius of cylinder 1
            cyl1_h: Height of cylinder 1
            cyl2_pos: Center of cylinder 2 base [x, y, z]
            cyl2_r: Radius of cylinder 2
            cyl2_h: Height of cylinder 2
            margin: Safety margin

        Returns:
            CollisionResult with collision information
        """
        result = CollisionResult(in_collision=False)

        # Check Z-axis overlap
        z1_min, z1_max = cyl1_pos[2], cyl1_pos[2] + cyl1_h
        z2_min, z2_max = cyl2_pos[2], cyl2_pos[2] + cyl2_h

        z_overlap = min(z1_max, z2_max) - max(z1_min, z2_min)


        if z_overlap <= 0:
            # No Z overlap - compute distance
            z_gap = max(z1_min - z2_max, z2_min - z1_max)
            xy_dist = np.linalg.norm(cyl1_pos[:2] - cyl2_pos[:2])
            result.min_distance = np.sqrt(z_gap**2 + max(0, xy_dist - cyl1_r - cyl2_r)**2)
            return result

        # Z overlap exists - check XY distance
        xy_dist = np.linalg.norm(cyl1_pos[:2] - cyl2_pos[:2])
        radii_sum = cyl1_r + cyl2_r + margin

        if xy_dist < radii_sum:
            result.in_collision = True
            result.penetration_depth = radii_sum - xy_dist
            # Contact point at midpoint
            z_contact = (max(z1_min, z2_min) + min(z1_max, z2_max)) / 2
            if xy_dist > 1e-6:
                xy_dir = (cyl2_pos[:2] - cyl1_pos[:2]) / xy_dist
                contact_xy = cyl1_pos[:2] + xy_dir * cyl1_r
            else:
                contact_xy = cyl1_pos[:2]
            result.contact_points = [np.array([contact_xy[0], contact_xy[1], z_contact])]
        else:
            result.min_distance = xy_dist - cyl1_r - cyl2_r

        return result
    
    # ==================== High-Level Collision Functions ====================
    
    def check_collision(
        self, 
        obj1: CollisionObject, 
        obj2: CollisionObject
    ) -> CollisionResult:
        """
        Check collision between two collision objects.
        Dispatches to appropriate primitive collision function.
        
        Args:
            obj1: First collision object
            obj2: Second collision object
            
        Returns:
            CollisionResult with collision information
        """
        pos1 = obj1.get_position()
        pos2 = obj2.get_position()
        dims1 = obj1.get_padded_dimensions()
        dims2 = obj2.get_padded_dimensions()
        
        # Dispatch based on object types
        if obj1.obj_type == CollisionObjectType.SPHERE:
            if obj2.obj_type == CollisionObjectType.SPHERE:
                result = self.sphere_sphere_collision(
                    pos1, dims1[0], pos2, dims2[0], self.collision_margin
                )
            elif obj2.obj_type == CollisionObjectType.BOX:
                result = self.sphere_box_collision(
                    pos1, dims1[0], pos2, dims2 / 2, self.collision_margin
                )
            elif obj2.obj_type == CollisionObjectType.CYLINDER:
                result = self.sphere_cylinder_collision(
                    pos1, dims1[0], pos2, dims2[0], dims2[1], self.collision_margin
                )
            else:
                # Fallback: treat as sphere
                result = self.sphere_sphere_collision(
                    pos1, dims1[0], pos2, dims2[0], self.collision_margin
                )
        elif obj1.obj_type == CollisionObjectType.BOX:
            if obj2.obj_type == CollisionObjectType.SPHERE:
                result = self.sphere_box_collision(
                    pos2, dims2[0], pos1, dims1 / 2, self.collision_margin
                )
            elif obj2.obj_type == CollisionObjectType.BOX:
                result = self.box_box_collision(
                    pos1, dims1 / 2, pos2, dims2 / 2, self.collision_margin
                )
            else:
                # Approximate as box-sphere
                result = self.sphere_box_collision(
                    pos2, dims2[0], pos1, dims1 / 2, self.collision_margin
                )
        elif obj1.obj_type == CollisionObjectType.CYLINDER:
            if obj2.obj_type == CollisionObjectType.SPHERE:
                result = self.sphere_cylinder_collision(
                    pos2, dims2[0], pos1, dims1[0], dims1[1], self.collision_margin
                )
            elif obj2.obj_type == CollisionObjectType.CYLINDER:
                result = self.cylinder_cylinder_collision(
                    pos1, dims1[0], dims1[1], pos2, dims2[0], dims2[1], self.collision_margin
                )
            else:
                # Approximate as sphere
                r1 = np.max(dims1) if len(dims1) > 0 else 0.1
                r2 = np.max(dims2) if len(dims2) > 0 else 0.1
                result = self.sphere_sphere_collision(pos1, r1, pos2, r2, self.collision_margin)
        else:
            # Default sphere approximation for other types
            r1 = np.max(dims1) if len(dims1) > 0 else 0.1
            r2 = np.max(dims2) if len(dims2) > 0 else 0.1
            result = self.sphere_sphere_collision(pos1, r1, pos2, r2, self.collision_margin)
        
        result.object_a = obj1.name
        result.object_b = obj2.name
        return result
    
    def check_robot_self_collision(
        self, 
        link_transforms: Dict[str, np.ndarray]
    ) -> CollisionResult:
        """
        Check for self-collision between robot links.
        
        Args:
            link_transforms: Dictionary mapping link names to their current poses
            
        Returns:
            CollisionResult with first collision found, or no collision
        """
        for i, j in self.self_collision_pairs:
            link_i = self.robot_links[i]
            link_j = self.robot_links[j]

            if link_i.name in link_transforms and link_j.name in link_transforms:
                # Update link collision geometry positions
                obj_i = link_i.collision_geometry
                obj_j = link_j.collision_geometry
                obj_i.pose[:3] = link_transforms[link_i.name][:3]
                obj_j.pose[:3] = link_transforms[link_j.name][:3]

                result = self.check_collision(obj_i, obj_j)
                if result.in_collision:
                    return result
        
        return CollisionResult(in_collision=False)
    
    def check_robot_environment_collision(
        self, 
        link_transforms: Dict[str, np.ndarray]
    ) -> CollisionResult:
        """
        Check for collision between robot links and environment objects.
        
        Args:
            link_transforms: Dictionary mapping link names to their current poses
            
        Returns:
            CollisionResult with first collision found, or no collision
        """
        for link in self.robot_links:
            if link.name not in link_transforms:
                continue
                
            obj_link = link.collision_geometry
            obj_link.pose[:3] = link_transforms[link.name][:3]
            
            for env_obj in self.environment_objects:
                result = self.check_collision(obj_link, env_obj)
                if result.in_collision:
                    return result
        
        return CollisionResult(in_collision=False)
    
    def is_state_valid(
        self, 
        link_transforms: Dict[str, np.ndarray],
        check_self: bool = True,
        check_environment: bool = True
    ) -> Tuple[bool, Optional[CollisionResult]]:
        """
        Check if a robot state is valid (collision-free).
        
        This is the main function used by motion planners.
        
        Args:
            link_transforms: Dictionary mapping link names to their current poses
            check_self: Whether to check self-collision
            check_environment: Whether to check environment collision
            
        Returns:
            Tuple of (is_valid, collision_result if invalid else None)
        """
        if check_self:
            result = self.check_robot_self_collision(link_transforms)
            if result.in_collision:
                return False, result
        
        if check_environment:
            result = self.check_robot_environment_collision(link_transforms)
            if result.in_collision:
                return False, result
        
        return True, None
    
    def get_minimum_distance(
        self, 
        link_transforms: Dict[str, np.ndarray]
    ) -> float:
        """
        Get minimum distance between robot and all obstacles.
        
        Args:
            link_transforms: Dictionary mapping link names to their current poses
            
        Returns:
            Minimum distance (negative if in collision)
        """
        min_dist = float('inf')
        
        for link in self.robot_links:
            if link.name not in link_transforms:
                continue
                
            obj_link = link.collision_geometry
            obj_link.pose[:3] = link_transforms[link.name][:3]
            
            for env_obj in self.environment_objects:
                result = self.check_collision(obj_link, env_obj)
                if result.in_collision:
                    min_dist = min(min_dist, -result.penetration_depth)
                else:
                    min_dist = min(min_dist, result.min_distance)
        
        return min_dist


# ==================== Factory Functions ====================

def create_sphere(name: str, position: np.ndarray, radius: float, padding: float = 0.0) -> CollisionObject:
    """Create a sphere collision object"""
    pose = np.zeros(7)
    pose[:3] = position
    pose[3] = 1.0  # qw
    return CollisionObject(
        name=name,
        obj_type=CollisionObjectType.SPHERE,
        pose=pose,
        dimensions=np.array([radius]),
        padding=padding
    )


def create_box(name: str, position: np.ndarray, size: np.ndarray, padding: float = 0.0) -> CollisionObject:
    """Create a box collision object"""
    pose = np.zeros(7)
    pose[:3] = position
    pose[3] = 1.0  # qw
    return CollisionObject(
        name=name,
        obj_type=CollisionObjectType.BOX,
        pose=pose,
        dimensions=np.array(size),
        padding=padding
    )


def create_cylinder(name: str, position: np.ndarray, radius: float, height: float, padding: float = 0.0) -> CollisionObject:
    """Create a cylinder collision object"""
    pose = np.zeros(7)
    pose[:3] = position
    pose[3] = 1.0  # qw
    return CollisionObject(
        name=name,
        obj_type=CollisionObjectType.CYLINDER,
        pose=pose,
        dimensions=np.array([radius, height]),
        padding=padding
    )


def create_robot_link(
    name: str, 
    geometry_type: CollisionObjectType,
    dimensions: np.ndarray,
    parent_joint_idx: int = -1,
    padding: float = 0.02
) -> RobotLink:
    """Create a robot link with collision geometry"""
    collision_obj = CollisionObject(
        name=f"{name}_collision",
        obj_type=geometry_type,
        pose=np.array([0, 0, 0, 1, 0, 0, 0], dtype=float),
        dimensions=dimensions,
        padding=padding
    )
    return RobotLink(
        name=name,
        collision_geometry=collision_obj,
        parent_joint_idx=parent_joint_idx
    )


# ==================== Example Usage ====================

if __name__ == "__main__":
    # Demo: Create a simple collision checking scenario
    print("=== Collision Library Demo ===\n")
    
    # Create collision checker
    checker = CollisionChecker()
    
    # Add environment objects
    table = create_box("table", np.array([0.5, 0.0, 0.4]), np.array([0.6, 0.8, 0.02]))
    obstacle = create_sphere("obstacle", np.array([0.4, 0.0, 0.6]), 0.1)
    
    checker.add_environment_object(table)
    checker.add_environment_object(obstacle)
    
    print(f"Added {len(checker.environment_objects)} environment objects")
    
    # Create robot links (GEN72 robot - example)
    links = [
        create_robot_link("link0", CollisionObjectType.CYLINDER, np.array([0.040, 0.218]), 0),
        create_robot_link("link1", CollisionObjectType.CYLINDER, np.array([0.032, 0.10]), 1),
        create_robot_link("link2", CollisionObjectType.CYLINDER, np.array([0.030, 0.28]), 2),
        create_robot_link("link3", CollisionObjectType.CYLINDER, np.array([0.028, 0.15]), 3),
        create_robot_link("link4", CollisionObjectType.CYLINDER, np.array([0.025, 0.2525]), 4),
        create_robot_link("link5", CollisionObjectType.CYLINDER, np.array([0.022, 0.12]), 5),
        create_robot_link("link6", CollisionObjectType.CYLINDER, np.array([0.020, 0.08]), 6),
        create_robot_link("link7", CollisionObjectType.SPHERE, np.array([0.025]), 7),
    ]
    checker.set_robot_links(links)
    
    print(f"Robot has {len(checker.robot_links)} links")
    print(f"Self-collision pairs: {len(checker.self_collision_pairs)}")
    
    # Test collision checking with sample transforms (GEN72 safe config)
    link_transforms = {
        "link0": np.array([0.0, 0.0, 0.0]),
        "link1": np.array([0.0, 0.0, 0.218]),
        "link2": np.array([0.0, 0.0, 0.218]),
        "link3": np.array([0.0, -0.28, 0.218]),
        "link4": np.array([0.04, -0.28, 0.218]),
        "link5": np.array([0.021, -0.0275, 0.218]),
        "link6": np.array([0.021, -0.0275, 0.218]),
        "link7": np.array([0.1115, 0.0395, 0.218]),
    }
    
    # Check if state is valid
    is_valid, collision_result = checker.is_state_valid(link_transforms)
    
    print(f"\nState valid: {is_valid}")
    if not is_valid:
        print(f"Collision between: {collision_result.object_a} and {collision_result.object_b}")
        print(f"Penetration depth: {collision_result.penetration_depth:.4f}m")
    
    # Get minimum distance
    min_dist = checker.get_minimum_distance(link_transforms)
    print(f"Minimum distance to obstacles: {min_dist:.4f}m")
    
    print("\n[OK] Collision library ready for use!")

