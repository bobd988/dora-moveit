#!/usr/bin/env python3
"""Monitor MuJoCo simulator outputs"""
from dora import Node
import time

node = Node()
start_time = time.time()
received_data = {
    "joint_positions": 0,
    "joint_velocities": 0,
    "sensor_data": 0
}

print("MuJoCo monitor started, listening for outputs...")
print("Waiting for data from mujoco_sim...")

timeout = 5.0
for event in node:
    elapsed = time.time() - start_time

    if event["type"] == "INPUT":
        event_id = event["id"]
        if event_id in received_data:
            received_data[event_id] += 1
            print(f"[{elapsed:.2f}s] Received {event_id} (count: {received_data[event_id]})")

    if elapsed > timeout:
        print(f"\nTimeout after {timeout}s")
        break

    if sum(received_data.values()) >= 10:
        print("\nReceived enough data, test passed!")
        break

print(f"\nResults after {elapsed:.2f}s:")
for key, count in received_data.items():
    print(f"  {key}: {count} messages")

if sum(received_data.values()) == 0:
    print("\nERROR: No data received from mujoco_sim!")
    print("MuJoCo node is not publishing data.")
