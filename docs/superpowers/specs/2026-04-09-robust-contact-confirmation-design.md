# Robust Contact Confirmation Gate
**Date:** 2026-04-09
**Status:** Approved for implementation

---

## Problem

`FootContactFSM.update()` requires `ticks >= 3 AND is_flat` to reach `CONTACT_CONFIRMED`.
`is_flat` is computed in `HeartBeat.py:read_sensors()` as:

```python
ss.left_foot_flat = (max(pts_x) - min(pts_x)) > 0.01 if len(pts_x) > 1 else False
```

PyBullet's narrow-phase solver for a **box vs. flat plane** collapses to a single contact
point at stable rest (degenerate case). `len(pts_x) == 1` → `is_flat = False` every cycle.
Contact is permanently stuck at `CONTACT_TENTATIVE`. Downstream consequences:

- `MissionController._handle_idle()` demands both feet `CONTACT_CONFIRMED` → mission
  stays `IDLE` forever.
- `GaitPlanner.update_gait_planner()` early-returns on `MissionState.IDLE` → `ik_angles`
  stay at zero → WBC outputs zero torque → **robot is motionless ("statue")**.
- `RecoveryController` timeout (`3.0 s`, `both_not_confirmed`) fires at cycle 302 →
  `EMERGENCY_STOP` → simulation terminates.

Root cause: the flat gate assumes multiple contact points for a stable foot. PyBullet
violates this for a rectangular box on a flat plane at rest.

---

## Design

### Approach: OR Gate with Pitch Fallback

The flat gate is correct when PyBullet provides multi-point data. Orientation takes over
when physics collapses to a single point.

```
is_confirmed = (ticks >= 3) AND (multi_point OR single_flat)

multi_point = (len(pts_x) > 1) AND (spread_x > 0.01 m)   ← original, unchanged
single_flat = (len(pts_x) == 1) AND (|foot_pitch| < 7°)   ← new fallback
```

The two conditions are independent physical sensors. Neither can spoof the other.

### Pitch Computation

PyBullet `getLinkState(rid, link_id)[1]` returns the link orientation as a quaternion
`(x, y, z, w)` in world frame. This call is **already made** in `read_sensors()` — the
quaternion is fetched but not stored.

Foot pitch = rotation about the world Y-axis (forward tilt). Standard ZYX extraction:

```python
qx, qy, qz, qw = link_state[1]
foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
```

The `max(-1, min(1, …))` clamp guards against floating-point overshoot at ±90°.

**Why 7° is the correct threshold:**

| Condition                         | Typical pitch  |
|-----------------------------------|----------------|
| Settled flat foot (post-bounce)   | 0–3°           |
| Micro-bounce oscillation          | 3–6°           |
| Heel-strike (transient)           | 8–15°          |
| Genuine tiptoe stance             | 20–35°         |
| **7° threshold**                  | Accepts rows 1–2, rejects rows 3–4 |

The tick counter (`>= 3` cycles above 5 N) is the temporal gate. Heel-strike never
accumulates 3 ticks, so the pitch threshold only needs to discriminate stable postures.

At `q = 0` the URDF foot sole is parallel to the floor (no plantar-flexion offset), so
`foot_pitch ≈ 0°` at rest. The 7° ceiling is a strict absolute check with no calibration
offset required.

### Companion Fix: Recovery Timeout Guard

`RecoveryController.evaluate()` fires its 3-second timeout whenever
`both_not_confirmed AND step_duration > 3.0 s`. With the contact gate fixed, this
condition will resolve naturally — but it remains a latent bug if `both_not_confirmed`
persists for any other reason while the robot is standing idle (not commanded to walk).

Add a mission-state guard:

```python
if (both_not_confirmed and
        step_duration > self.config.timeout_threshold and
        shared_state.mission_state != MissionState.IDLE):
```

A robot in IDLE has not been commanded to step. Timing out on unconfirmed contact in
IDLE is incorrect behaviour. The timeout is only meaningful during an active step
(RAMP / WALK / DECEL / STOP).

---

## Affected Files

| File | Change | Scope |
|------|--------|-------|
| `shared_state.py` | Add `left_foot_pitch`, `right_foot_pitch` fields | `__init__`, `reset()` |
| `HeartBeat.py` | Add `FLAT_PITCH_THRESHOLD` constant; extract quaternion pitch in `read_sensors()`; update `foot_flat` OR gate | `PyBulletInterface` |
| `recovery.py` | Add `MissionState` import; add `mission_state != IDLE` guard to timeout check | `RecoveryController.evaluate()` |
| `perception.py` | **No changes** — reads `left_foot_flat` as before |  |
| `mission.py` | **No changes** |  |

---

## Constants

```python
# HeartBeat.py — alongside other module-level constants
FLAT_PITCH_THRESHOLD: float = math.radians(7.0)
# rad — foot-flat gate for single-contact confirmation.
# At q=0 the URDF foot is parallel to the floor (zero plantar offset).
# 7° accepts post-bounce settling (0–6°); rejects tiptoe (>20°) and
# transient heel-strike (8–15°, never accumulates 3 ticks anyway).
```

---

## Data Flow

```
getLinkState(foot_link)   [already called — no new PyBullet call]
    ├── [0] pos  → ss.left_foot_position        (unchanged)
    ├── [1] quat → foot_pitch → ss.left_foot_pitch   (NEW, stored for diagnostics)
    └── [6] vel  → ss.left_foot_velocity        (unchanged)

read_sensors() per foot:
    # Pitch computed unconditionally (kinematic, not force-dependent)
    qx, qy, qz, qw = link_state[1]
    foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    ss.left_foot_pitch = foot_pitch

    if force > 5 N:
        pts_x       = [c[5][0] for c in contacts]
        multi_point = len(pts_x) > 1 and spread > 0.01
        single_flat = len(pts_x) == 1 and abs(foot_pitch) < FLAT_PITCH_THRESHOLD
        ss.left_foot_flat = multi_point or single_flat
    else:
        ss.left_contact_ticks = 0
        ss.left_foot_flat     = False   ← unchanged

ss.left_foot_flat ──► FootContactFSM.update()   [unchanged]
                       ticks >= 3 AND is_flat → CONTACT_CONFIRMED
                                    │
                       MissionController._handle_idle()
                       → IDLE → RAMP (unblocked)
                                    │
                       GaitPlanner writes ik_angles
                                    │
                       WBC generates torques → robot walks
```

---

## Tests

New tests added to `test_transitions.py` (existing pattern: manually set shared_state
fields, call `update_perception()`, assert state):

| # | Setup | Expected |
|---|-------|----------|
| T1 | `ticks=3`, `len(pts_x)=1`, `foot_pitch=4°` | `left_foot_flat=True`, state → `CONTACT_CONFIRMED` |
| T2 | `ticks=3`, `len(pts_x)=1`, `foot_pitch=15°` | `left_foot_flat=False`, state stays `CONTACT_TENTATIVE` |
| T3 | `ticks=3`, `len(pts_x)=4`, spread=3cm, any pitch | `left_foot_flat=True` (multi-point path, regression) |
| T4 | `ticks=3`, `len(pts_x)=1`, `foot_pitch=7.0°` exactly | `left_foot_flat=False` (strictly less than) |
| T5 | `ticks=2`, `len(pts_x)=1`, `foot_pitch=4°` | state stays `CONTACT_TENTATIVE` (tick gate still required) |

Recovery companion fix tests (new or extended in `test_mission.py`):

| # | Setup | Expected |
|---|-------|----------|
| R1 | `mission_state=IDLE`, `both_not_confirmed`, `step_duration=4.0s` | No recovery action (IDLE guard blocks timeout) |
| R2 | `mission_state=WALK`, `both_not_confirmed`, `step_duration=4.0s` | `REPOSITION` fires (original behaviour preserved) |

---

## Out of Scope

- Changing `FLAT_PITCH_THRESHOLD` dynamically (fixed constant is sufficient)
- Modifying `perception.py`'s private state handler methods (`_handle_touch_expected`,
  `_detect_slip`) — dead code removal is a separate cleanup task
- Changes to `gait_planner.py`, `wbc`, or any control module
