import time
from motor import Motor

motor = Motor()

dt = 0.1  # seconds
steps = 50

motor.set_speed(90)

for step in range(steps):
    t = step * dt

    if step == 20:
        motor.set_speed(30)
    if step == 35:
        motor.set_speed(-60)

    motor.update(dt)

    print(
        f"t={t:.1f}s | "
        f"speed={motor.speed:.1f} deg/s | "
        f"angle={motor.angle:.2f} deg"
    )

    time.sleep(dt)
