# Idle Stance Controller + Left Hip Torque Fix

**Date:** 2026-05-02
**Problem:** Robot flips over within 2 seconds of spawning. Legs point upward.

## Root Cause Analysis (from session 2026-05-02_11-29-37)

Five cascading failures:

1. **No standing pose** — IK targets (`ik_left_angles`, `ik_right_angles`) are `(0,0,0,0)` for the entire run. The gait planner is the only module that writes these, but it exits immediately when `mission_state == IDLE` (`gait_planner.py:145`). Without `--walk`, mission never leaves IDLE.
2. **Left hip pitch torque dead** — `tau_L_hip_fwd = 0.0` across all 1000 cycles despite non-zero WBC tracking errors (up to 0.22 rad). Cause: PyBullet default velocity motors are never disabled after URDF load, interfering with TORQUE_CONTROL.
3. **Contact forces below threshold** — measured 3.0/0.0 N vs threshold 5.0 N. An 8 kg robot should produce ~39 N/foot. Consequence of no stance pose.
4. **Knee torque saturation** — both knees hit 100 N·m limit instantly, fighting gravity with wrong targets (0 rad = straight legs).
5. **COM free-fall** — 0.886 m → 0.147 m in 2 seconds.

## Design

### Change 1: Disable Default PyBullet Motors

**File:** `HeartBeat.py`, `PyBulletRobot._build_joint_map()`

After building the joint map, disable the default velocity motor on every joint:
```
p.setJointMotorControl2(robot_id, jid, p.VELOCITY_CONTROL, force=0)
```

One-time init step. Joints later set to POSITION_CONTROL (hip roll, hip twist) override this. Joints using TORQUE_CONTROL (hip pitch, knee, ankle) require this to function.

### Change 2: Idle Stance Pose in Gait Planner

**File:** `gait_planner.py`

Remove the early return on `mission_state == IDLE` at line 145. Add `_handle_idle_stance()`:

1. Compute target foot z-offset from desired COM height (80% of full leg extension) using URDF-derived segment lengths from `kinematics.py`.
2. Solve IK via `kinematics.solve_ik()` for both legs — feet directly below hips.
3. Write to `ik_left_angles` and `ik_right_angles`.
4. Fallback: if IK raises ValueError, use hardcoded angles computed from the 80% geometry (approx hip_pitch ~0.15 rad, knee ~0.30 rad, ankle ~-0.15 rad — exact values derived from URDF at implementation time).

Flow:
- `mission == IDLE` → `_handle_idle_stance()` (new)
- `mission == WALK` → existing gait logic (unchanged)

### Not Changed (deliberate)

- **WBC gains** (KP=100, KD=28) — evaluate after robot can stand
- **Balance controller** — already computes hip roll + sagittal offset; they had no effect with zero IK targets
- **Mission state machine** — IDLE→RAMP transition requires CONTACT_CONFIRMED, which should succeed once stance produces real contact forces
- **Contact threshold** (5.0 N) — symptom of no stance, not a threshold bug
- **URDF** — no modifications

## Verification

Run `python3 main.py --gui` and confirm:
- COM stays above 0.7 m for 10 seconds
- Both feet reach CONTACT_CONFIRMED
- `tau_L_hip_fwd` shows non-zero values in telemetry
- Regime monitor reports condition above CRITICAL
