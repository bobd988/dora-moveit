#!/usr/bin/env python3
"""
Standalone MuJoCo visualization script to debug collision issues.
Shows robot initial position and environment objects.
"""

import numpy as np
import mujoco
import mujoco.viewer

# Create MuJoCo XML model
xml = """
<mujoco model="gen72_scene">
  <option gravity="0 0 -9.81"/>

  <worldbody>
    <!-- Ground plane -->
    <body name="ground" pos="0 0 -0.2">
      <geom type="box" size="1 1 0.01" rgba="0.3 0.3 0.3 1"/>
    </body>

    <!-- Table -->
    <body name="table" pos="0.6 0 0.3">
      <geom type="box" size="0.2 0.3 0.01" rgba="0.6 0.4 0.2 1"/>
    </body>

    <!-- Test obstacle -->
    <body name="obstacle" pos="0.4 0.2 0.6">
      <geom type="sphere" size="0.1" rgba="1 0 0 0.5"/>
    </body>

    <!-- Robot base -->
    <body name="link0" pos="0 0 0">
      <geom type="cylinder" size="0.06 0.075" rgba="0.8 0.8 0.8 1"/>
      <joint name="joint0" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>

      <body name="link1" pos="0 0 0.15">
        <geom type="cylinder" size="0.06 0.06" rgba="0.7 0.7 0.7 1"/>
        <joint name="joint1" type="hinge" axis="0 1 0" range="-1.7628 1.7628"/>

        <body name="link2" pos="0 0 0.12">
          <geom type="cylinder" size="0.05 0.075" rgba="0.8 0.8 0.8 1"/>
          <joint name="joint2" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>

          <body name="link3" pos="0 0 0.15">
            <geom type="cylinder" size="0.05 0.06" rgba="0.7 0.7 0.7 1"/>
            <joint name="joint3" type="hinge" axis="0 1 0" range="-3.0718 -0.0698"/>

            <body name="link4" pos="0 0 0.12">
              <geom type="cylinder" size="0.045 0.075" rgba="0.8 0.8 0.8 1"/>
              <joint name="joint4" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>

              <body name="link5" pos="0 0 0.15">
                <geom type="cylinder" size="0.04 0.04" rgba="0.7 0.7 0.7 1"/>
                <joint name="joint5" type="hinge" axis="0 1 0" range="-0.0175 3.7525"/>

                <body name="link6" pos="0 0 0.08">
                  <geom type="sphere" size="0.04" rgba="0.8 0.8 0.8 1"/>
                  <joint name="joint6" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>

                  <body name="link7" pos="0 0 0.05">
                    <geom type="sphere" size="0.05" rgba="0.5 0.5 0.9 1"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="act0" joint="joint0" kp="100"/>
    <position name="act1" joint="joint1" kp="100"/>
    <position name="act2" joint="joint2" kp="100"/>
    <position name="act3" joint="joint3" kp="100"/>
    <position name="act4" joint="joint4" kp="100"/>
    <position name="act5" joint="joint5" kp="100"/>
    <position name="act6" joint="joint6" kp="100"/>
  </actuator>
</mujoco>
"""

# Load model
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Set initial configuration from demo_node.py
initial_config = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
data.qpos[:7] = initial_config
data.ctrl[:7] = initial_config

# Forward kinematics
mujoco.mj_forward(model, data)

print("=== MuJoCo Scene Visualization ===")
print(f"Initial configuration: {initial_config}")
print(f"Ground position: [0, 0, -0.2]")
print(f"Table position: [0.6, 0, 0.3]")
print(f"Obstacle position: [0.4, 0.2, 0.6]")
print("\nOpening MuJoCo viewer...")
print("Use mouse to rotate view, scroll to zoom")
print("Press ESC to close")

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
