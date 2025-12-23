import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF("single_servo.urdf", [0, 0, 0.05])

def move_servo(target_angle, steps=200, delay=1):
    joint_index = 0
    for step in range(steps):
        current_angle = p.getJointState(robot, joint_index)[0]
        angle = current_angle + (target_angle - current_angle) * (step + 1) / steps
        p.setJointMotorControl2(
            robot,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=angle,
            force=5,
            positionGain=0.1,
            velocityGain=0.1
        )
        p.stepSimulation()
        time.sleep(delay)

# Move smoothly
move_servo(1.57)  # 90 deg


input("Press Enter to exit...")
p.disconnect()
