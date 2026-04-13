# Vertical Support Force Fix — Design Spec
**Date:** 2026-04-13
**Status:** Approved

---

## Problem Statement

The robot collapses every run. Telemetry shows COM_Z rising from 0.859 m to 0.922 m
in 0.13 s, contact forces spiking to 178–197 N then dropping to zero, and
`Stab=UNSTABLE` from cycle 1. The GATE log shows `F=[0,0]` before the real-time
loop stabilises. The FSM never exits DOUBLE_SUPPORT.

Three confirmed root causes, in priority order:

### Root Cause 1 — GRF spring formula computes against wrong variable (PRIMARY)

`grf.py` computes:
```
F_z = K_SPRING * (Z_REST - z_foot) - B_DAMPER * z_dot_foot
```
where `z_foot` is the world-frame absolute z of the foot (≈ 0 when foot is on the
ground) and `Z_REST = 0.75 m` is the **nominal leg extension** (hip-to-foot distance).

At standing (`z_foot ≈ 0`): `F_z = 1589 × 0.75 = 1192 N` per leg. The Jacobian
transpose maps this to joint extension torques that actively push the body upward.
The legs extend until they lock out, the Jacobian determinant → 0, force collapses,
and the robot falls. This is what the COM_Z-rising signature confirms.

A correctly formulated spring needs:
1. The compression variable to be **leg extension** (`z_hip - z_foot`), not absolute foot z.
2. A **gravity feedforward** term so the spring produces half-body-weight at
   equilibrium instead of zero.

### Root Cause 2 — Default PyBullet velocity motors fight WBC torques (SECONDARY)

PyBullet loads every URDF joint with a default velocity motor. `apply_control` sends
`TORQUE_CONTROL` commands but never disables these default motors. They run
simultaneously, reducing effective torque delivery by an unpredictable fraction.

### Root Cause 3 — WBC `target_torques` accumulates across cycles (TERTIARY)

`_wbc_step` reads `target_torques`, adds the new PD value to the existing value,
then writes back — without clearing the dict first. After N cycles, the commanded
torque is N × single-cycle PD value. This causes the initial spike pattern visible
in the data (F=[178,197] at c=2–4).

---

## Fix Design (Approach A — Minimal Targeted Changes)

### Fix 1: GRF spring formula (`grf.py`)

**New constants:**
```python
ROBOT_MASS:   float = 8.0                      # kg, from URDF total mass
GRAVITY_COMP: float = ROBOT_MASS * 9.81 / 2.0  # N·per·leg, feedforward at standing
# Z_REST = 0.75 m unchanged — still nominal leg extension
```

**`_spring_damper_fz` signature change:**
```python
# Before: receives z_foot (world-frame absolute)
def _spring_damper_fz(z_foot, z_dot_foot, k_spring):
    return k_spring * (Z_REST - z_foot) - B_DAMPER * z_dot_foot

# After: receives leg_ext = z_hip - z_foot (true compression variable)
def _spring_damper_fz(leg_ext, z_dot_foot, k_spring):
    return GRAVITY_COMP + k_spring * (Z_REST - leg_ext) - B_DAMPER * z_dot_foot
```

At equilibrium (`leg_ext = Z_REST = 0.75 m`):
`F_z = GRAVITY_COMP + 0 - 0 = 39.2 N` — exactly half body weight. ✓

**`_compute_leg_correction` signature change:**
- Add `z_hip: float` parameter.
- Compute `leg_ext = z_hip - z_foot` internally.
- Pass `leg_ext` to `_spring_damper_fz` instead of `z_foot`.

**`GRFController.update` change:**
- Look up `z_hip` from `shared_state.link_positions` for each leg before calling
  `_compute_leg_correction`. Use link keys `'Left_Upper_Leg_1'` and
  `'Right_Upper_Leg_1'` (same keys used in `gait_planner.py`).
- Fall back to `Z_REST` (0.75 m) if the link is not yet in `link_positions`
  (first cycle before `update_link_positions` runs).

### Fix 2: Disable default PyBullet velocity motors (`HeartBeat.py`)

Append to `PyBulletInterface._build_joint_map`, after the joint-map loop:
```python
# Disable default velocity motors so TORQUE_CONTROL has full authority.
# PyBullet loads every joint with a default velocity motor; without this,
# it fights every WBC torque command.
for jid in self.joint_ids.values():
    p.setJointMotorControl2(
        self.robot_id, jid,
        controlMode=p.VELOCITY_CONTROL,
        force=0,
        physicsClientId=self.pc,
    )
```

Called once at init, zero runtime cost.

### Fix 3: Reset `target_torques` each cycle (`HeartBeat.py`)

At the top of `_wbc_step`, replace:
```python
torques = getattr(shared_state, 'target_torques', {})
```
with:
```python
torques = {}   # fresh each cycle; GRF corrections merged in apply_control
```

GRF corrections are already merged separately in `apply_control` and are unaffected.

---

## Files Changed

| File | Change |
|------|--------|
| `grf.py` | Add `ROBOT_MASS`, `GRAVITY_COMP`; fix `_spring_damper_fz`; add `z_hip` plumbing in `_compute_leg_correction` and `update` |
| `HeartBeat.py` | Motor-disable loop in `_build_joint_map`; torque-dict reset in `_wbc_step` |
| `test_grf.py` (new) | 3 unit tests for GRF equilibrium and spring behaviour |

**Out of scope:** gait FSM phases (COM_SHIFT, LIFT, SWING, PLACE). Those are only
reachable once DOUBLE_SUPPORT exits cleanly. Do not touch them.

---

## Success Criteria

Verified from the existing GATE log and telemetry — no new instrumentation required:

| Signal | Before | After |
|--------|--------|-------|
| `COM_Z` | Rising +6 cm in 0.13 s | Stable ±2 cm of spawn height |
| `F=[L,R]` | Spike to ~190 N then 0 | Steady ~39 N per leg |
| `Stab=` | 3 (UNSTABLE) from c=1 | 1 (STABLE) within 5 cycles |
| GATE `both=True` | Never | Fires at `timer ≥ DS_MIN_TIME` |
| DS → COM_SHIFT | Never | Transitions within 0.5 s |

---

## Unit Tests (`test_grf.py`)

1. `test_grf_equilibrium` — `z_foot=0`, `z_hip=0.75`, `z_dot=0` → `F_z ≈ 39.2 N ± 1 N`
2. `test_grf_compression` — `z_foot=0.03` (foot above ground, leg compressed) → `F_z > GRAVITY_COMP`
3. `test_grf_extension` — `z_hip=0.72` (leg over-extended) → `F_z < GRAVITY_COMP`
