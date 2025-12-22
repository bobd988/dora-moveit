#!/usr/bin/env python3
"""
将 GEN72 URDF 转换为 MuJoCo XML 并添加障碍物
"""
import mujoco

# 加载 URDF
urdf_path = "GEN72/urdf/gen_72_b_description/meshes/GEN72.urdf"
model = mujoco.MjModel.from_xml_path(urdf_path)

# 保存为 MuJoCo XML
xml_path = "GEN72_base.xml"
mujoco.mj_saveLastXML(xml_path, model)

print(f"已将 URDF 转换为 MuJoCo XML: {xml_path}")
print("现在手动编辑该文件添加光源、地面和障碍物")
