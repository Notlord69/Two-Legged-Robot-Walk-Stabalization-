# day3_servo_pybullet.py
import pybullet as p
import pybullet_data
import time

# --- Setup PyBullet ---
p.connect(p.GUI)  # Use GUI to see movement
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Ground plane

# Load a simple robotic arm (1 joint as servo)
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.1])  # Use R2D2 model as a simple joint example

# Logging setup
log_file = "log.txt"

def log(message):
    print(message)
    with open(log_file, "a") as f:
        f.write(message + "\n")

# --- Servo simulation function ---
def move_servo(target_angle_rad, steps=100, delay=0.01):
    """Move joint 0 to target_angle_rad gradually."""
    joint_index = 0
    current_angle = p.getJointState(robot, joint_index)[0]
    log(f"Starting servo move: current={current_angle:.2f} rad, target={target_angle_rad:.2f} rad")

    for step in range(1, steps + 1):
        # Linear interpolation
        angle = current_angle + (target_angle_rad - current_angle) * (step / steps)
        p.setJointMotorControl2(robot, joint_index, p.POSITION_CONTROL, targetPosition=angle)
        p.stepSimulation()
        time.sleep(delay)
        log(f"Step {step}: angle={angle:.2f} rad")

    log(f"Reached target angle: {target_angle_rad:.2f} rad\n")

# --- Example movements ---
move_servo(1.57)  # ~90 degrees in radians
move_servo(3.14)  # ~180 degrees in radians
move_servo(0)     # back to 0 degrees

# --- Cleanup ---
time.sleep(2)
p.disconnect()
