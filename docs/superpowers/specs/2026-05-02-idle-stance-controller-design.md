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

## Implementation Status (2026-05-02)

### Completed (committed)

1. **Disable default PyBullet motors** — `HeartBeat.py:_build_joint_map()` now calls
   `setJointMotorControl2(VELOCITY_CONTROL, force=0)` on every joint after URDF load.
   Commit: `316c731`

2. **Idle stance IK in gait planner** — `gait_planner.py` no longer returns early on
   `MissionState.IDLE`. New `_handle_idle_stance()` computes standing IK at 95% R_MAX
   with hardcoded fallback. Commits: `ebc02ff`, `8d3e03d`

3. **Tests** — 3 new tests in `Test_Enviroment/test_idle_stance.py` (nonzero IK,
   within limits, fallback). Existing `test_no_update_when_idle` renamed to
   `test_no_gait_advance_when_idle`. All 377 tests pass. Commit: `831517b`

### Blocker: Robot Explodes After Motor Disable

Disabling default PyBullet velocity motors causes the robot to catapult to 40-72 m
altitude within 2 seconds. All joint torques read 0 N·m in telemetry despite the WBC
computing non-zero errors (1.3-3.7 rad, 5-8 joints saturated). Zero contact forces.

**Attempted fix (reverted):** `_set_initial_pose()` using `p.resetJointState()` to
spawn joints at stance angles. Did not help — robot still explodes, likely due to
link interpenetration causing explosive PyBullet constraint resolution.

**Root issue:** After `setJointMotorControl2(VELOCITY_CONTROL, force=0)`, subsequent
`setJointMotorControl2(TORQUE_CONTROL, force=X)` calls in `apply_control()` appear to
have no effect — `getJointState()[3]` (appliedJointMotorTorque) returns 0 on ALL
joints for the entire run. This means no motor torque is ever applied, joints are
completely free, and the robot is a ragdoll under gravity + constraint forces.

### Geometry Discovery

The "80% of full leg extension" target from the design is unreachable. The robot's
extreme proportions (L_THIGH=60.7mm, L_SHANK=687mm) create a narrow workspace:
- R_MIN = 0.6313 m (85% of R_MAX)
- R_MAX = 0.7426 m
- Usable range: 111 mm

Joint limits (±1.571 rad) constrain standing to d ≥ 0.6906 m (93% R_MAX). The
implementation uses 95% R_MAX (d=0.7055m), giving hip=-1.22 rad, knee=1.30 rad.

## Future Debugging

### Priority 1: Fix motor disable → torque application pipeline

The default motor disable (`VELOCITY_CONTROL, force=0`) breaks torque control entirely.
Investigate:

1. **PyBullet motor mode transition** — Does switching from `VELOCITY_CONTROL` to
   `TORQUE_CONTROL` via `setJointMotorControl2` work correctly? Write a minimal
   PyBullet test (single joint, no URDF complexity) to verify.

2. **Alternative disable patterns** — Try `p.setJointMotorControl2(...,
   p.VELOCITY_CONTROL, targetVelocity=0, force=0)` with explicit `targetVelocity=0`.
   Or try `p.setJointMotorControlArray` for batch disable.

3. **Per-joint debug** — Add a debug print inside `apply_control()` to confirm
   `setJointMotorControl2(TORQUE_CONTROL, force=X)` is actually called with X ≠ 0.
   If X is non-zero but readback is 0, the issue is in PyBullet's motor model.

4. **Consider reverting motor disable** — The original problem (left hip torque = 0)
   may have a different root cause. Before the disable, the right hip and all
   knee/ankle joints DID get torque. Investigate why Left_Hip_Forwards specifically
   fails with default motors active.

### Priority 2: Stance transition strategy

Even when motor control works, driving joints from 0 to ±1.22 rad with KP=100 will
saturate all joints at 100 N·m. Options:
- Ramp IK targets gradually from current position over ~50 cycles
- Use `p.resetJointState()` to pre-position joints (needs interpenetration testing)
- Spawn robot at a higher z to allow settling, then lower

## Verification (when blocker resolved)

Run `python3 main.py --gui` and confirm:
- COM stays above 0.7 m for 10 seconds
- Both feet reach CONTACT_CONFIRMED
- `tau_L_hip_fwd` shows non-zero values in telemetry
- Regime monitor reports condition above CRITICAL
