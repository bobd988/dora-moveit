#!/usr/bin/env python3
"""
GEN72 Robot Configuration
Extracted from URDF: GEN72/urdf/gen_72_b_description/urdf/GEN72.urdf
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class JointConfig:
    """Single joint configuration"""
    name: str
    lower_limit: float  # rad
    upper_limit: float  # rad
    velocity_limit: float  # rad/s
    effort_limit: float  # Nm

@dataclass
class DHParams:
    """DH parameters for a link"""
    a: float  # link length
    alpha: float  # link twist
    d: float  # link offset
    theta_offset: float  # joint angle offset

class GEN72Config:
    """GEN72 7-DOF Robot Arm Configuration"""

    # Number of joints
    NUM_JOINTS = 7

    # Joint limits (from URDF)
    JOINT_CONFIGS = [
        JointConfig("joint1", -3.0014, 3.0014, 3.141, 25.0),
        JointConfig("joint2", -1.8323, 1.8323, 3.141, 25.0),
        JointConfig("joint3", -3.0014, 3.0014, 3.141, 25.0),
        JointConfig("joint4", -2.8792, 0.9597, 3.141, 25.0),
        JointConfig("joint5", -3.0014, 3.0014, 3.926, 5.0),
        JointConfig("joint6", -1.707, 1.783, 3.926, 5.0),
        JointConfig("joint7", -3.0014, 3.0014, 3.926, 5.0),
    ]

    # Extract arrays for easy access
    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms from URDF (xyz, rpy)
    LINK_TRANSFORMS = [
        # joint1: base_link -> Link1
        {"xyz": [0, 0, 0.218], "rpy": [0, 0, 0]},
        # joint2: Link1 -> Link2
        {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]},
        # joint3: Link2 -> Link3
        {"xyz": [0, -0.28, 0], "rpy": [1.5708, 0, 0]},
        # joint4: Link3 -> Link4
        {"xyz": [0.04, 0, 0], "rpy": [1.5708, 0, 0]},
        # joint5: Link4 -> Link5
        {"xyz": [-0.019, 0.2525, 0], "rpy": [-1.5708, 0, 0]},
        # joint6: Link5 -> Link6
        {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]},
        # joint7: Link6 -> Link7
        {"xyz": [0.0905, 0.067, 0], "rpy": [1.5708, 0, 1.5708]},
    ]

    # Collision geometry (simplified spheres for each link)
    # Format: (type, dimensions) where dimensions = [radius] for sphere
    # Using sphere approximations to avoid orientation issues with cylinders
    # Radii based on URDF inertia parameters and link geometry
    COLLISION_GEOMETRY = [
        ("sphere", [0.055]),  # base_link - mass=0.726kg
        ("sphere", [0.045]),  # Link1 - mass=0.511kg
        ("sphere", [0.045]),  # Link2 - mass=0.552kg
        ("sphere", [0.045]),  # Link3 - mass=0.774kg
        ("sphere", [0.04]),   # Link4 - mass=0.437kg
        ("sphere", [0.035]),  # Link5 - mass=0.424kg
        ("sphere", [0.03]),   # Link6 - mass=0.303kg
        ("sphere", [0.025]),  # Link7 - mass=0.177kg, end effector
    ]

    # Safety parameters
    COLLISION_MARGIN = 0.015  # 1cm safety margin
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 5.0, 8.0, 8.0, 8.0])  # rad/s^2 (estimated)

    # Default home configuration (safe, no self-collision)
    # Joint2 raised to avoid link2-base collision, joint4 bent to keep arm compact
    HOME_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])

    # Safe initial configuration (within limits, no self-collision)
    SAFE_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        """Get joint position limits"""
        return GEN72Config.JOINT_LOWER_LIMITS, GEN72Config.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        """Get joint velocity limits"""
        return GEN72Config.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        """Check if configuration is within joint limits"""
        return np.all(q >= GEN72Config.JOINT_LOWER_LIMITS) and \
               np.all(q <= GEN72Config.JOINT_UPPER_LIMITS)

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        """Clip configuration to joint limits"""
        return np.clip(q, GEN72Config.JOINT_LOWER_LIMITS, GEN72Config.JOINT_UPPER_LIMITS)
