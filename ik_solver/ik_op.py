#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Inverse Kinematics Operator for Dora-MoveIt
============================================

Converts end-effector pose to joint positions.
This is a pluggable IK solver - can be swapped with any IK implementation:
- Numerical IK (Jacobian-based)
- Analytical IK (robot-specific)
- Machine Learning IK
- External IK service

Input: target_pose (6D: x, y, z, roll, pitch, yaw) or (7D: x, y, z, qw, qx, qy, qz)
Output: joint_positions (7D for GEN72 robot)

Dora Integration:
- Receives: ik_request (pose), joint_state (current joints for seeding)
- Sends: ik_solution (joint positions), ik_status (success/failure)
"""

import json
import numpy as np
import pyarrow as pa
from dataclasses import dataclass
from typing import Optional, Tuple, List
from dora import Node
from config.robot_config import GEN72Config
from ik_solver.advanced_ik_solver import TracIKSolver, DifferentialEvolutionIKSolver, IKRequest, IKResult


class NumericalIKSolver:
    """
    Numerical IK solver using Jacobian pseudo-inverse method.
    
    This is a simplified IK solver for demonstration.
    In production, use specialized libraries like:
    - PyKDL
    - ikfast
    - pytorch-kinematics
    - pinocchio
    """
    
    def __init__(self, num_joints: int = 7):
        """
        Initialize IK solver.
        
        Args:
            num_joints: Number of robot joints
        """
        self.num_joints = num_joints
        self.max_iterations = 500
        self.position_tolerance = 0.01
        self.orientation_tolerance = 1e-1
        self.step_size = 0.5
        
        # Joint limits (GEN72 robot)
        self.joint_limits_lower = GEN72Config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = GEN72Config.JOINT_UPPER_LIMITS
        
        # Link transforms from GEN72 URDF
        self.link_transforms = GEN72Config.LINK_TRANSFORMS
        
    def forward_kinematics(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics using GEN72 link transforms.

        Args:
            joint_positions: Joint angles [7]

        Returns:
            Tuple of (position [3], rotation_matrix [3x3])
        """
        q = joint_positions

        # Helper function for rotation matrices
        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        # Start from base
        T = np.eye(4)

        # Apply each joint transform from GEN72Config.LINK_TRANSFORMS
        transforms = GEN72Config.LINK_TRANSFORMS

        for i, (joint_angle, link_tf) in enumerate(zip(q, transforms)):
            # Extract xyz and rpy from link transform
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]

            # Build transform matrix for this link
            R_link = rot_z(rpy[2]) @ rot_x(rpy[0])  # Simplified: only use roll and yaw
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            # Apply link transform
            T = T @ T_link

            # Apply joint rotation (all joints rotate around Z-axis)
            T_joint = np.eye(4)
            T_joint[:3, :3] = rot_z(joint_angle)
            T = T @ T_joint

        position = T[:3, 3]
        rotation = T[:3, :3]

        return position, rotation
    
    def compute_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix numerically.
        
        Args:
            joint_positions: Current joint positions [7]
            
        Returns:
            Jacobian matrix [6 x 7]
        """
        jacobian = np.zeros((6, self.num_joints))
        delta = 1e-6
        
        pos0, rot0 = self.forward_kinematics(joint_positions)
        
        for i in range(self.num_joints):
            q_delta = joint_positions.copy()
            q_delta[i] += delta
            
            pos1, rot1 = self.forward_kinematics(q_delta)
            
            # Position Jacobian
            jacobian[:3, i] = (pos1 - pos0) / delta
            
            # Orientation Jacobian (simplified)
            jacobian[3:, i] = 0.0  # Placeholder for angular velocity
        
        return jacobian
    
    def solve(self, request: IKRequest) -> IKResult:
        """
        Solve inverse kinematics.
        
        Args:
            request: IK request with target pose
            
        Returns:
            IKResult with solution or failure info
        """
        target_pos = request.target_position
        
        # Initialize with seed or safe config
        if request.seed_joints is not None:
            q = request.seed_joints.copy()
        else:
            q = GEN72Config.SAFE_CONFIG.copy()
        
        for iteration in range(self.max_iterations):
            # Ensure q is 1D with correct size
            q = np.asarray(q).flatten()[:self.num_joints]

            # Forward kinematics
            current_pos, current_rot = self.forward_kinematics(q)

            # Position error
            pos_error = target_pos - current_pos
            error_norm = np.linalg.norm(pos_error)

            # Check convergence
            if error_norm < self.position_tolerance:
                return IKResult(
                    success=True,
                    joint_positions=q,
                    error=error_norm,
                    iterations=iteration + 1,
                    message="IK converged successfully"
                )

            # Compute Jacobian
            J = self.compute_jacobian(q)[:3, :]  # Position only (3x7)

            # Damped least squares (Levenberg-Marquardt)
            damping = 0.001
            JJT = J @ J.T
            J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(3))

            # Update joints
            dq = J_pinv @ pos_error
            q = q + self.step_size * dq

            # Apply joint limits
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)
        
        # Failed to converge
        current_pos, _ = self.forward_kinematics(q)
        final_error = np.linalg.norm(target_pos - current_pos)
        
        return IKResult(
            success=False,
            joint_positions=q,
            error=final_error,
            iterations=self.max_iterations,
            message=f"IK failed to converge. Error: {final_error:.6f}"
        )


class IKOperator:
    """
    Dora operator for Inverse Kinematics.

    Inputs:
        - ik_request: Target pose [x, y, z, roll, pitch, yaw] or [x, y, z, qw, qx, qy, qz]
        - joint_state: Current joint positions (for seeding)

    Outputs:
        - ik_solution: Joint positions if successful
        - ik_status: JSON with success/error info
    """

    def __init__(self, num_joints: int = 7, solver_type: str = "tracik"):
        """
        Initialize IK operator.

        Args:
            num_joints: Number of joints
            solver_type: "tracik" (default), "de" (differential evolution), or "simple"
        """
        if solver_type == "tracik":
            self.solver = TracIKSolver()
            print("[IK] Using TracIK solver (advanced)")
        elif solver_type == "de":
            self.solver = DifferentialEvolutionIKSolver()
            print("[IK] Using Differential Evolution solver")
        else:
            self.solver = NumericalIKSolver(num_joints)
            print("[IK] Using simple numerical solver")

        self.solver_type = solver_type
        self.current_joints: Optional[np.ndarray] = None
        self.request_count = 0
        
    def process_joint_state(self, joint_positions: np.ndarray):
        """Update current joint state for seeding"""
        self.current_joints = joint_positions.copy()
        
    def process_ik_request(self, pose_data: np.ndarray) -> Tuple[Optional[np.ndarray], dict]:
        """
        Process an IK request.
        
        Args:
            pose_data: Target pose array (6D or 7D)
            
        Returns:
            Tuple of (joint_solution or None, status_dict)
        """
        self.request_count += 1
        
        # Parse pose
        if len(pose_data) == 6:
            # [x, y, z, roll, pitch, yaw]
            position = pose_data[:3]
            orientation = pose_data[3:6]
            orientation_type = "rpy"
        elif len(pose_data) == 7:
            # [x, y, z, qw, qx, qy, qz]
            position = pose_data[:3]
            orientation = pose_data[3:7]
            orientation_type = "quaternion"
        else:
            return None, {
                "success": False,
                "error": f"Invalid pose length: {len(pose_data)}. Expected 6 (xyzrpy) or 7 (xyzquat)",
                "request_id": self.request_count
            }
        
        # Create IK request
        request = IKRequest(
            target_position=position,
            target_orientation=orientation,
            seed_joints=self.current_joints,
            orientation_type=orientation_type
        )
        
        # Solve IK
        result = self.solver.solve(request)
        
        status = {
            "success": result.success,
            "error": float(result.error),
            "iterations": result.iterations,
            "message": result.message,
            "request_id": self.request_count,
            "target_position": position.tolist()
        }
        
        if result.success:
            return result.joint_positions, status
        else:
            return None, status


def main():
    """Main entry point for Dora IK operator"""
    print("=== Dora-MoveIt IK Operator ===")

    node = Node()
    # Use TracIK solver by default (can be changed to "de" or "simple")
    ik_op = IKOperator(num_joints=7, solver_type="tracik")

    print("IK operator started, waiting for requests...")
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "joint_state":
                # Update current joint state for seeding
                joints = event["value"].to_numpy()
                ik_op.process_joint_state(joints)
                
            elif input_id == "ik_request":
                # Process IK request
                pose = event["value"].to_numpy()
                
                print(f"[IK] Request #{ik_op.request_count + 1}: target={pose[:3]}")
                
                solution, status = ik_op.process_ik_request(pose)
                
                # Send status
                status_bytes = json.dumps(status).encode('utf-8')
                node.send_output("ik_status", pa.array(list(status_bytes), type=pa.uint8()))
                
                # Send solution if successful
                if solution is not None:
                    node.send_output(
                        "ik_solution",
                        pa.array(solution, type=pa.float32()),
                        metadata={"encoding": "jointstate", "success": True}
                    )
                    print(f"[IK] SUCCESS: Solution found, error={status['error']:.6f}")
                else:
                    print(f"[IK] FAILED: {status['message']}")
                    
        elif event_type == "STOP":
            print("IK operator stopping...")
            break


if __name__ == "__main__":
    main()

