#!/usr/bin/env python3
"""Monitor vehicle movement"""
from dora import Node
import time

node = Node()
start_time = time.time()
joint_count = 0
movement_complete = False

print("Vehicle monitor started...")

for event in node:
    elapsed = time.time() - start_time

    if event["type"] == "INPUT":
        event_id = event["id"]

        if event_id == "joint_positions":
            joint_count += 1
            if joint_count % 10 == 0:
                print(f"[{elapsed:.2f}s] Received {joint_count} joint_positions")

        elif event_id == "movement_complete":
            movement_complete = True
            print(f"[{elapsed:.2f}s] Vehicle movement complete!")
            break

    if elapsed > 15.0:
        print(f"\nTimeout after {elapsed:.2f}s")
        break

print(f"\nResults:")
print(f"  Joint positions received: {joint_count}")
print(f"  Movement complete: {movement_complete}")

if joint_count == 0:
    print("\nERROR: No joint_positions received from MuJoCo!")
