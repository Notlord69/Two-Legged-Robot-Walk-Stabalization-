Motor Simulation – Speed Control

- Motor has state: angle and speed
- Speed controls how fast angle changes
- Core equation: angle += speed * dt
- dt defines simulation resolution
- Speed can change during runtime (open-loop control)
- Angle clamping simulates servo limits
- No feedback, no correction, no PID
