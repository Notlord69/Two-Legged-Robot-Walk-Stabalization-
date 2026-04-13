import pybullet as p
import pybullet_data
import time
import os

pc = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

urdf_path = "/home/notlord/ros2_ws/Siclo1_V1/Siclo1.urdf"
robot_id = p.loadURDF(urdf_path, basePosition=[0.0, 0.0, 0.8806])

print(f"Initial Z: {p.getBasePositionAndOrientation(robot_id)[0][2]}")

for _ in range(100):
    p.stepSimulation()

print(f"Final Z after 100 steps: {p.getBasePositionAndOrientation(robot_id)[0][2]}")
p.disconnect()
