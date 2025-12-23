# day3_servo_pybullet_smooth.py
import pybullet as p
import pybullet_data
import time

# --- Setup PyBullet ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF("r2d2.urdf", [0, 0, 0.1])

log_file = "log.txt"

def log(message):
    print(message)
    with open(log_file, "a") as f:
        f.write(message + "\n")

# --- Servo simulation function using VELOCITY_CONTROL ---
def move_servo(target_angle_rad, steps=500, delay=0.02, max_velocity=0.5):
    joint_index = 0
    current_angle = p.getJointState(robot, joint_index)[0]
    log(f"Starting servo move: current={current_angle:.2f} rad, target={target_angle_rad:.2f} rad")

    for step in range(1, steps + 1):
        # Linear interpolation for desired position
        desired_angle = current_angle + (target_angle_rad - current_angle) * (step / steps)
        # Current joint state
        joint_state = p.getJointState(robot, joint_index)[0]
        # Compute velocity toward desired angle
        velocity = (desired_angle - joint_state) / delay
        # Limit max velocity
        velocity = max(-max_velocity, min(max_velocity, velocity))
        # Apply velocity control
        p.setJointMotorControl2(
            robot,
            joint_index,
            p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=10
        )
        p.stepSimulation()
        time.sleep(delay)
        log(f"Step {step}: angle={joint_state:.2f} rad, velocity={velocity:.2f}")

    log(f"Reached target angle: {target_angle_rad:.2f} rad\n")

# --- Example movements ---
move_servo(1.57)
move_servo(3.14)
move_servo(0)

input("Press Enter to exit...")
p.disconnect()
