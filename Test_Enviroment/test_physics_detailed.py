import pybullet as p
import pybullet_data
import os
import numpy as np

pc = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81, physicsClientId=pc)
p.setTimeStep(0.01, physicsClientId=pc)
p.loadURDF("plane.urdf", physicsClientId=pc)

urdf_path = "/home/notlord/ros2_ws/Siclo1_V1/Siclo1.urdf"
p.setAdditionalSearchPath(os.path.dirname(urdf_path))
robot_id = p.loadURDF(
    urdf_path,
    basePosition=[0.0, 0.0, 0.8806],
    physicsClientId=pc,
    flags=p.URDF_USE_INERTIA_FROM_FILE
)

print(f"Robot ID: {robot_id}")
print(f"Num joints: {p.getNumJoints(robot_id, physicsClientId=pc)}")

pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=pc)
print(f"Start Z: {pos[2]:.6f}")

for i in range(10):
    p.stepSimulation(physicsClientId=pc)
    pos, _ = p.getBasePositionAndOrientation(robot_id, physicsClientId=pc)
    print(f"Cycle {i} Z: {pos[2]:.6f}")

p.disconnect()
