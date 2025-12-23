#!/usr/bin/env python3
"""Test if dora timer is working"""
from dora import Node
import time

node = Node()
tick_count = 0
start_time = time.time()

print("Timer test node started, waiting for ticks...")

for event in node:
    if event["type"] == "INPUT":
        tick_count += 1
        elapsed = time.time() - start_time
        print(f"Tick #{tick_count} received at {elapsed:.2f}s")

        if tick_count >= 10:
            print(f"\nTimer working! Received {tick_count} ticks in {elapsed:.2f}s")
            break
    elif event["type"] == "STOP":
        break

print(f"Test complete. Total ticks: {tick_count}")
