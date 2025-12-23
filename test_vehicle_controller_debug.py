#!/usr/bin/env python3
"""Debug vehicle controller"""
from dora import Node
import pyarrow as pa
import numpy as np

node = Node()

target_distance = 1.5
wheel_speed = 2.0
initial_pos = None
moved = False

print("Vehicle controller started (DEBUG MODE)")

# Send initial wheel command
wheel_cmd = np.array([wheel_speed, wheel_speed], dtype=np.float32)
node.send_output("wheel_commands", pa.array(wheel_cmd))
print(f"Sent initial wheel command: {wheel_cmd}")

for event in node:
    if event["type"] == "INPUT":
        event_id = event["id"]

        if event_id == "joint_positions":
            joint_pos = event["value"].to_numpy()
            print(f"Received joint_positions: len={len(joint_pos)}, first 7: {joint_pos[:7]}")

            if len(joint_pos) >= 7:
                current_x = joint_pos[0]

                if initial_pos is None:
                    initial_pos = current_x
                    print(f"Initial position set: {initial_pos}")

                distance_moved = current_x - initial_pos
                print(f"Distance moved: {distance_moved:.3f}m / {target_distance}m")

                if distance_moved < target_distance and not moved:
                    wheel_cmd = np.array([wheel_speed, wheel_speed], dtype=np.float32)
                    node.send_output("wheel_commands", pa.array(wheel_cmd))
                else:
                    if not moved:
                        wheel_cmd = np.array([0.0, 0.0], dtype=np.float32)
                        node.send_output("wheel_commands", pa.array(wheel_cmd))
                        print(f"Vehicle stopped after moving {distance_moved:.3f}m")
                        node.send_output("movement_complete", pa.array([True]))
                        print("Sent movement_complete signal")
                        moved = True
                        break
