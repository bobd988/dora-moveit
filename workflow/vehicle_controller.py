#!/usr/bin/env python3
"""
Simple vehicle controller to drive Hunter SE forward 1.5m
"""
from dora import Node
import pyarrow as pa
import numpy as np

node = Node()

target_distance = 0.3  # meters
wheel_speed = 20.0  # rad/s for rear wheels
initial_pos = None
moved = False
first_message = True

for event in node:
    if event["type"] == "INPUT":
        event_id = event["id"]

        if event_id == "joint_positions":
            # Send initial wheel command on first message
            if first_message:
                wheel_cmd = np.array([wheel_speed, wheel_speed], dtype=np.float32)
                node.send_output("wheel_commands", pa.array(wheel_cmd))
                first_message = False

            # Get current joint positions (includes freejoint position)
            joint_pos = event["value"].to_numpy()

            # freejoint gives [x, y, z, qw, qx, qy, qz] as first 7 values
            if len(joint_pos) >= 7:
                current_x = joint_pos[0]

                if initial_pos is None:
                    initial_pos = current_x

                distance_moved = current_x - initial_pos

                if distance_moved < target_distance and not moved:
                    # Send wheel commands: both wheels same speed
                    wheel_cmd = np.array([wheel_speed, wheel_speed], dtype=np.float32)
                    node.send_output("wheel_commands", pa.array(wheel_cmd))
                else:
                    if not moved:
                        # Stop the vehicle
                        wheel_cmd = np.array([0.0, 0.0], dtype=np.float32)
                        node.send_output("wheel_commands", pa.array(wheel_cmd))
                        print(f"Vehicle stopped after moving {distance_moved:.3f}m")
                        # Send completion signal
                        print("[DEBUG] Sending movement_complete signal")
                        node.send_output("movement_complete", pa.array([True]))
                        moved = True
