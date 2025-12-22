#!/usr/bin/env python3
"""
Point Cloud Collision Detection for Dora-MoveIt
================================================

Integrates 3D LiDAR point cloud data into collision checking.
Uses octree-based spatial indexing for efficient collision queries.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from scipy.spatial import cKDTree


@dataclass
class PointCloudCollisionResult:
    """Result of point cloud collision check"""
    in_collision: bool
    min_distance: float
    closest_point: Optional[np.ndarray] = None
    link_name: Optional[str] = None


class OctreePointCloud:
    """
    Octree-based point cloud representation for fast collision queries.
    Uses KD-tree for efficient nearest neighbor search.
    """

    def __init__(self, points: np.ndarray, voxel_size: float = 0.01):
        """
        Initialize octree from point cloud.

        Args:
            points: Point cloud array [N x 3]
            voxel_size: Voxel size for downsampling (meters)
        """
        self.raw_points = points
        self.voxel_size = voxel_size

        # Downsample using voxel grid
        self.points = self._voxel_downsample(points, voxel_size)

        # Build KD-tree for fast nearest neighbor queries
        self.kdtree = cKDTree(self.points)

        print(f"[PointCloud] Loaded {len(points)} points, downsampled to {len(self.points)}")

    def _voxel_downsample(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Downsample point cloud using voxel grid"""
        if len(points) == 0:
            return points

        # Compute voxel indices
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Use unique voxel indices to get representative points
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)

        return points[unique_idx]

    def query_radius(self, center: np.ndarray, radius: float) -> List[np.ndarray]:
        """
        Query all points within radius of center.

        Args:
            center: Query center [3]
            radius: Search radius (meters)

        Returns:
            List of points within radius
        """
        indices = self.kdtree.query_ball_point(center, radius)
        return self.points[indices] if len(indices) > 0 else []

    def nearest_distance(self, point: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Find nearest point and distance.

        Args:
            point: Query point [3]

        Returns:
            Tuple of (distance, nearest_point)
        """
        dist, idx = self.kdtree.query(point)
        return dist, self.points[idx]

    def update(self, new_points: np.ndarray):
        """Update point cloud with new data"""
        self.raw_points = new_points
        self.points = self._voxel_downsample(new_points, self.voxel_size)
        self.kdtree = cKDTree(self.points)


class PointCloudCollisionChecker:
    """
    Collision checker that uses point cloud data from 3D LiDAR.
    Integrates with existing collision_lib.py system.
    """

    def __init__(self, safety_margin: float = 0.05, voxel_size: float = 0.01):
        """
        Initialize point cloud collision checker.

        Args:
            safety_margin: Safety distance threshold (meters)
            voxel_size: Voxel size for point cloud downsampling
        """
        self.safety_margin = safety_margin
        self.voxel_size = voxel_size
        self.octree: Optional[OctreePointCloud] = None

    def set_point_cloud(self, points: np.ndarray):
        """
        Set or update the point cloud from LiDAR.

        Args:
            points: Point cloud array [N x 3] in robot base frame
        """
        if len(points) == 0:
            print("[PointCloud] Warning: Empty point cloud received")
            return

        if self.octree is None:
            self.octree = OctreePointCloud(points, self.voxel_size)
        else:
            self.octree.update(points)

    def check_sphere_collision(
        self,
        center: np.ndarray,
        radius: float
    ) -> PointCloudCollisionResult:
        """
        Check if a sphere collides with point cloud.

        Args:
            center: Sphere center [3]
            radius: Sphere radius

        Returns:
            Collision result
        """
        if self.octree is None:
            return PointCloudCollisionResult(False, float('inf'))

        # Find nearest point
        dist, nearest_pt = self.octree.nearest_distance(center)

        # Check collision with safety margin
        collision_threshold = radius + self.safety_margin
        in_collision = dist < collision_threshold

        return PointCloudCollisionResult(
            in_collision=in_collision,
            min_distance=dist - radius,
            closest_point=nearest_pt
        )

    def check_cylinder_collision(
        self,
        base: np.ndarray,
        axis: np.ndarray,
        radius: float,
        height: float
    ) -> PointCloudCollisionResult:
        """
        Check if a cylinder collides with point cloud.

        Args:
            base: Cylinder base center [3]
            axis: Cylinder axis direction (normalized) [3]
            radius: Cylinder radius
            height: Cylinder height

        Returns:
            Collision result
        """
        if self.octree is None:
            return PointCloudCollisionResult(False, float('inf'))

        # Sample points along cylinder axis
        num_samples = max(3, int(height / 0.05))
        t_values = np.linspace(0, height, num_samples)

        min_dist = float('inf')
        closest_pt = None

        for t in t_values:
            # Point on cylinder axis
            axis_point = base + t * axis

            # Find nearest point cloud point
            dist, nearest_pt = self.octree.nearest_distance(axis_point)

            # Distance from axis to point
            vec_to_point = nearest_pt - axis_point
            dist_to_axis = np.linalg.norm(vec_to_point)

            # Actual distance from cylinder surface
            surface_dist = dist_to_axis - radius

            if surface_dist < min_dist:
                min_dist = surface_dist
                closest_pt = nearest_pt

        collision_threshold = self.safety_margin
        in_collision = min_dist < collision_threshold

        return PointCloudCollisionResult(
            in_collision=in_collision,
            min_distance=min_dist,
            closest_point=closest_pt
        )

    def check_link_collision(
        self,
        link_position: np.ndarray,
        link_geometry: Tuple[str, np.ndarray],
        link_name: str = ""
    ) -> PointCloudCollisionResult:
        """
        Check collision for a robot link.

        Args:
            link_position: Link position [3]
            link_geometry: Tuple of (type, dimensions)
                - "sphere": dimensions = [radius]
                - "cylinder": dimensions = [radius, height]
            link_name: Name of the link

        Returns:
            Collision result
        """
        geom_type, dimensions = link_geometry

        if geom_type == "sphere":
            radius = dimensions[0]
            result = self.check_sphere_collision(link_position, radius)
        elif geom_type == "cylinder":
            radius, height = dimensions[0], dimensions[1]
            # Assume cylinder aligned with Z-axis
            axis = np.array([0, 0, 1])
            base = link_position - axis * height / 2
            result = self.check_cylinder_collision(base, axis, radius, height)
        else:
            return PointCloudCollisionResult(False, float('inf'))

        result.link_name = link_name
        return result

    def check_robot_collision(
        self,
        link_transforms: dict,
        collision_geometry: List[Tuple[str, List[float]]]
    ) -> Tuple[bool, Optional[PointCloudCollisionResult]]:
        """
        Check collision for entire robot against point cloud.

        Args:
            link_transforms: Dictionary mapping link names to positions
            collision_geometry: List of (type, dimensions) for each link

        Returns:
            Tuple of (in_collision, collision_result)
        """
        if self.octree is None:
            return False, None

        min_result = None
        min_distance = float('inf')

        for i, (link_name, link_pos) in enumerate(link_transforms.items()):
            if i >= len(collision_geometry):
                break

            geom = collision_geometry[i]
            result = self.check_link_collision(link_pos, geom, link_name)

            if result.in_collision:
                return True, result

            if result.min_distance < min_distance:
                min_distance = result.min_distance
                min_result = result

        return False, min_result


def transform_pointcloud(
    points: np.ndarray,
    translation: np.ndarray,
    rotation: np.ndarray
) -> np.ndarray:
    """
    Transform point cloud from sensor frame to robot base frame.

    Args:
        points: Point cloud in sensor frame [N x 3]
        translation: Translation vector [3]
        rotation: Rotation matrix [3 x 3] or quaternion [4]

    Returns:
        Transformed points [N x 3]
    """
    if rotation.shape == (4,):
        # Convert quaternion to rotation matrix
        qw, qx, qy, qz = rotation
        rotation = np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
            [2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
            [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)]
        ])

    # Apply rotation and translation
    transformed = (rotation @ points.T).T + translation
    return transformed
