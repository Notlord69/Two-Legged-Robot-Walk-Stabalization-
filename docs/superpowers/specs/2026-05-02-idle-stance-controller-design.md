# Idle Stance Controller + Left Hip Torque Fix

**Date:** 2026-05-02 | **Updated:** 2026-05-02 (blocker resolved, ramp design added)
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

**Initial hypothesis (WRONG):** After `setJointMotorControl2(VELOCITY_CONTROL, force=0)`,
subsequent `setJointMotorControl2(TORQUE_CONTROL, force=X)` calls in `apply_control()`
appear to have no effect — `getJointState()[3]` (appliedJointMotorTorque) returns 0 on
ALL joints for the entire run.

### Blocker Resolved (2026-05-02, session 12-27-38)

**The torques ARE being applied.** The zero readback is a PyBullet API limitation, not a
control failure. Evidence:

1. **Angular velocity proves torque transfer.** From cycle 1 (post-warmup): base angular
   velocities reach 60 rad/s. An 8 kg ragdoll under gravity alone cannot reach 60 rad/s
   in 0.5s (50 warmup cycles). Only sustained 100 N·m joint torques produce this.

2. **`getJointState()[3]` semantics.** PyBullet's `TORQUE_CONTROL` mode works via
   `addJointTorque()` — an external force, not the internal motor constraint. The field
   `appliedJointMotorTorque` (index 3) reports motor constraint torque only. When the motor
   is disabled (`VELOCITY_CONTROL, force=0`), the motor torque is 0 — regardless of
   external torques applied via `TORQUE_CONTROL`. This is documented PyBullet behavior.

3. **Saturation confirms WBC output.** Telemetry shows 5-6 joints saturated at 100 N·m
   from the first cycle onward (columns 39-44). The WBC IS computing maximum torque.

**True root cause: instantaneous step command + no ramp.** The robot spawns with all
joints at 0 rad (PyBullet default). The idle stance controller immediately commands
hip_pitch=±1.22 rad, knee=±1.30 rad. The PD controller (KP=100 N·m/rad) sees
>1 rad error on every joint, saturates at 100 N·m, and catapults the robot.

Timeline:
- Cycle 0: joints at 0 rad, target at ±1.22 rad → error ~1.22 rad → τ = 100 N·m (saturated)
- Warmup (50 cycles, 0.5s): all joints slam toward targets at max torque, overshoot
- Cycle 1 (post-warmup): COM at y=1.977 m (should be 0), angular velocity 60 rad/s
- By t=10s: COM_z = 55 m (free flight)

**Telemetry bug:** Columns 45-50 (applied torques) read from `shared_state.joint_torques`,
which is populated by `getJointState()[3]` in `read_sensors()`. For TORQUE_CONTROL joints,
this is always 0. Fix: report `shared_state.target_torques` (WBC-computed) instead.

### Geometry Discovery

The "80% of full leg extension" target from the design is unreachable. The robot's
extreme proportions (L_THIGH=60.7mm, L_SHANK=687mm) create a narrow workspace:
- R_MIN = 0.6313 m (85% of R_MAX)
- R_MAX = 0.7426 m
- Usable range: 111 mm

Joint limits (±1.571 rad) constrain standing to d ≥ 0.6906 m (93% R_MAX). The
implementation uses 95% R_MAX (d=0.7055m), giving hip=-1.22 rad, knee=1.30 rad.

## Resolved Investigations

### Priority 1 (RESOLVED): Motor disable → torque pipeline is WORKING

The default motor disable pattern is correct and torques are being applied. See
"Blocker Resolved" section above. No code change needed for motor disable.

### Priority 2 (ACTIVE): Stance transition — WBC oscillation during ramp

Driving joints from 0 to ±1.22 rad with KP=100 saturates all joints at 100 N·m and
catapults the robot. Multiple approaches attempted; core problem identified but not
yet solved.

## Changes Implemented (committed this session)

### Change 3: Ramp-to-Stance Controller (IMPLEMENTED, NOT SUFFICIENT)

**File:** `gait_planner.py`, `_handle_idle_stance()`
**Status:** Code committed, tests passing. Ramp logic is correct but does not prevent
the explosion because the WBC oscillates at ±100 N·m during the ramp (see "Oscillation
Root Cause" below).

On first idle call, snapshots current joint positions. Over `STANCE_RAMP_CYCLES` (50)
cycles, linearly interpolates from snapshot to stance targets. Resets on
`reset_gait_planner()`. Three new tests added, all 380 tests pass.

```
α(t) = min(1.0, cycle_count / STANCE_RAMP_CYCLES)
target = (1 - α) * start_angles + α * stance_angles
```

### Change 4: Fix Telemetry Torque Readback (IMPLEMENTED, WORKING)

**File:** `HeartBeat.py` (telemetry row write, columns 45-50)
**Status:** Committed, working. Telemetry now shows actual WBC-commanded torques.

Replaced `shared_state.joint_torques` (PyBullet motor readback, always 0 for
TORQUE_CONTROL) with `shared_state.target_torques` (WBC-computed torques). This
confirmed torques ARE applied — the readback was the misleading signal.

## Failed Approaches (all reverted)

### Approach A: resetJointState to stance before warmup (REVERTED)

**What:** Added `_set_initial_stance()` in `_build_joint_map()` to call
`p.resetJointState()` on all joints to stance angles immediately after URDF load.

**Result:** No interpenetration (verified: pure gravity run with stance angles is
stable, angular velocity stays 0). But the WBC still oscillates at ±100 N·m during
warmup, launching the robot. Pre-positioning doesn't help because the oscillation is
a control-loop instability, not a position problem.

### Approach B: Lower spawn height to z=−0.14 (REVERTED)

**What:** Changed `URDF_SPAWN_Z` from 0.02 to −0.14 so feet touch ground at stance
angles. With stance angles, feet are at z=0.16 when spawned at z=0.02 — 16cm above
ground. At z=−0.14, feet are at z≈0 (2.5mm penetration, resolved gently by PyBullet).

**Result:** Robot still explodes during warmup. The 16cm freefall isn't the root cause —
the WBC oscillation happens even when feet start on the ground.

**Geometry learned:** Base spawns at (0.55, 0.18, 0.74) when URDF origin is at
(0, 0, −0.14). URDF base_link is ~0.88m above CAD floor origin. Feet at stance
angles are at x=0.87 (left) and x=0.23 (right), spread 0.64m laterally. COM at
(0.55, 0.18, 0.85).

### Approach C: Simplified warmup — no balance/GRF/perception (REVERTED)

**What:** Reduced warmup to only: sensors → gait_planner → WBC → apply_control → step.
Removed perception, stability, balance_controller, GRF, mission, recovery.

**Result:** Robot still explodes. The oscillation comes from the WBC itself, not from
balance controller corrections. Removing balance controller made no difference.

## Oscillation Root Cause (identified, not fixed)

**Discovered via manual 10-cycle trace** (session 2026-05-02):

With joints pre-positioned at stance (via resetJointState) and ramp starting from
stance, the first warmup cycle has zero WBC error (τ ≈ 0). But by cycle 1, the
left hip flips from 1.22 to 1.32 rad (+0.10 rad overshoot), and the WBC reacts
with −100 N·m. By cycle 2, the joint swings back to 0.64 rad, and the WBC hits
+100 N·m. The torque oscillates ±100 N·m every cycle:

```
cycle=0  L_hip=1.2191  tau_hip= 0.0   max_err=0.00
cycle=1  L_hip=1.3190  tau_hip=-100.0  max_err=0.10
cycle=2  L_hip=0.6446  tau_hip=+100.0  max_err=0.59
cycle=3  L_hip=1.6080  tau_hip=-100.0  max_err=0.39
cycle=4  L_hip=0.6273  tau_hip=+100.0  max_err=0.65
...
```

**Why this happens:** The joint swings past the target on the first step (0.10 rad
overshoot from ground contact bounce), then KP=100 produces a large corrective
torque that overshoots in the other direction. The KD=28 damping is insufficient
to prevent oscillation at these error magnitudes. Each cycle, the overshoot grows
because the torque is always at saturation (±100 N·m).

**This is a classic PD controller instability:** KP is too high for the joint
inertia + contact dynamics of this robot. The effective loop gain (KP × dt²/I)
exceeds the stability margin. The KD term should prevent this, but KD=28 at the
velocities involved (>2 rad/s by cycle 2) produces only ~56 N·m of damping — not
enough to counteract the 100 N·m proportional term.

## Next Steps (for next session)

### Option 1: Reduce WBC gains (most likely fix)

KP=100 is clearly too aggressive for an 8 kg robot. The critical damping ratio
ζ = KD / (2√(KP × I_eff)) needs to be ≥ 1. With I_eff ≈ 0.5 kg·m² (estimated
from link masses × shank length²), critical KD = 2√(100 × 0.5) = 14.1 N·m·s/rad.
KD=28 should be sufficient — but the effective inertia may be much lower due to
the extreme L_THIGH/L_SHANK ratio (60.7mm / 687mm), making the system underdamped.

Try: KP=30, KD=10 (conservative, 1/3 of current). Ramp-to-stance should then work
because the PD controller won't saturate on small errors.

### Option 2: POSITION_CONTROL for all joints during IDLE stance

Instead of using TORQUE_CONTROL with WBC PD during idle stance, switch all joints to
PyBullet `POSITION_CONTROL` with the stance angles as targets. Let PyBullet's internal
PD controller handle the settling. Only switch to TORQUE_CONTROL when transitioning
to WALK (when the WBC needs to track dynamic trajectories).

Pro: PyBullet's internal PD is tuned for stability.
Con: Need mode-switching logic in `apply_control()`, and the transition from
POSITION_CONTROL to TORQUE_CONTROL could be discontinuous.

### Option 3: Joint-level velocity clamping

Add per-joint velocity limits via `p.changeDynamics(maxJointVelocity=...)`. This
prevents the catastrophic overshoot — the joint physically can't move more than
a few degrees per step, so the PD error stays bounded even with KP=100.

## Verification (when explosion is resolved)

Run `python3 main.py --gui --duration 1000` and confirm:

1. **No explosion:** COM stays below 2 m altitude for the entire run
2. **Standing:** COM_z stabilizes between 0.65-0.90 m within the first 1.0s
3. **Torque telemetry:** Columns 45-50 show non-zero values (telemetry fix is working)
4. **No oscillation:** Hip torques don't alternate ±100 N·m every cycle
5. **Contact forces:** Both feet show > 5 N within first 2.0s
6. **Regime monitor:** Condition reaches DEGRADED or NOMINAL

### Smoke test commands
```bash
LATEST=$(ls -td sessions/2026-* | head -1)

# 1. COM altitude — should not exceed 2m
awk -F',' 'NR>1{z=$6+0; if(z>2.0) print "FAIL: COM_z="z" at t="$1}' "$LATEST/telemetry.csv" | head -3

# 2. Torque oscillation check — sign should NOT flip every cycle
awk -F',' 'NR>1 && NR<12{printf "t=%.2f tau_L_hip=%.1f tau_R_hip=%.1f\n", $1, $46, $49}' "$LATEST/telemetry.csv"

# 3. Joint angle convergence over first 0.5s
awk -F',' 'NR>1 && NR<=52{printf "t=%.3f L_hip=%.3f L_knee=%.3f R_hip=%.3f R_knee=%.3f\n", $1, $20, $21, $23, $24}' "$LATEST/telemetry.csv"

# 4. Contact forces after settling
awk -F',' 'NR>1 && $1+0>1.0 && $1+0<1.5{printf "t=%.2f L=%.1f R=%.1f\n", $1, $13, $14}' "$LATEST/telemetry.csv"
```
