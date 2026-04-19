# Swing Oscillation Fix — Design Spec

**Date:** 2026-04-19  
**Status:** Draft  
**Problem:** Robot trips during early/mid-swing due to WBC oscillation causing self-collision

---

## Problem Analysis

### Symptoms
- Swing leg "swings crazily" at liftoff
- Leg slows down, then tangles with stance leg
- Tripping occurs during early swing (φ < 0.2) and mid-swing (φ ≈ 0.3–0.5)

### Root Cause
1. **WBC gain saturation:** WBC_KP=200 N·m/rad saturates at 100 N·m effort limit with only 0.5 rad error. Creates bang-bang control → overshoot.
2. **Underdamped response:** WBC_KD=15 N·m·s/rad insufficient for leg inertia (~0.5 kg·m²). ζ ≈ 0.3 (underdamped) → oscillation.
3. **Tight clearance:** SWING_HEIGHT=0.04m leaves no margin for tracking error.
4. **Fast trajectory:** SWING_DURATION=0.40s requires high acceleration, exacerbating saturation.

### Leg Geometry Context
- L_THIGH = 0.06m, L_SHANK = 0.69m (ratio 1:11)
- High shank inertia about hip → large torque needed for acceleration
- Small hip errors create large foot position errors (long lever arm)

---

## Solution

### 2.1 Retune WBC Gains (HeartBeat.py)

| Parameter | Current | New | Rationale |
|-----------|---------|-----|-----------|
| WBC_KP | 200.0 N·m/rad | 100.0 N·m/rad | Halved to avoid torque saturation at small errors |
| WBC_KD | 15.0 N·m·s/rad | 28.0 N·m·s/rad | Increased for near-critical damping (ζ ≈ 0.9) |

**Critical damping calculation:**  
ζ = KD / (2·√(KP·I)), with I ≈ 0.5 kg·m²  
ζ = 28 / (2·√(100·0.5)) = 28 / 14.1 ≈ 0.9

### 2.2 Increase Swing Clearance (gait_planner.py)

| Parameter | Current | New | Rationale |
|-----------|---------|-----|-----------|
| SWING_HEIGHT | 0.04 m | 0.06 m | +2cm margin for tracking error |
| SWING_DURATION | 0.40 s | 0.50 s | +100ms reduces peak acceleration by 36% |

**Acceleration reduction:**  
Peak accel ∝ H/T². Ratio = (0.06/0.50²) / (0.04/0.40²) = 0.24 / 0.25 = 0.96  
But velocity ∝ H/T: (0.06/0.50) / (0.04/0.40) = 0.12 / 0.10 = 1.2 (20% faster peak velocity)  
Net effect: smoother profile, more time to track.

### 2.3 Add Tracking Telemetry

**shared_state.py** — new fields in `Siclo1State.__init__`:
```python
self.wbc_tracking_error: Dict[str, float] = {}      # θ_cmd - θ_actual per joint
self.wbc_torque_saturated: Dict[str, bool] = {}     # True when |τ| ≥ effort limit
```

**HeartBeat.py** — in `_wbc_step()`, after computing τ for each joint:
```python
error = theta_target - theta_now
shared_state.wbc_tracking_error[jname] = error
lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
shared_state.wbc_torque_saturated[jname] = (abs(tau) >= lim - 0.1)
```

Existing TelemetryThread will pick up these fields for session logging.

---

## Files Changed

| File | Changes |
|------|---------|
| `HeartBeat.py` | WBC_KP: 200→100, WBC_KD: 15→28, tracking error logging in `_wbc_step()` |
| `gait_planner.py` | SWING_HEIGHT: 0.04→0.06, SWING_DURATION: 0.40→0.50 |
| `shared_state.py` | Add `wbc_tracking_error: Dict[str, float]`, `wbc_torque_saturated: Dict[str, bool]` |

---

## Testing

### Unit Tests

**test_wbc_tracking.py** (new):
- `test_tracking_error_field_exists` — shared_state has wbc_tracking_error dict
- `test_saturation_flag_triggers` — flag sets when torque hits limit
- `test_tracking_error_populated` — error dict filled after WBC cycle

**test_gait_planner.py** (extend):
- `test_swing_height_increased` — SWING_HEIGHT == 0.06
- `test_swing_duration_increased` — SWING_DURATION == 0.50

### Integration Validation

Run `python3 main.py --walk 1.0` with GUI:
1. Swing leg moves smoothly (no oscillation)
2. Legs don't collide (no tangling)
3. Robot completes walk without tripping

### Telemetry Validation

Post-run checks:
- `max(|wbc_tracking_error|) < 0.3 rad` — no large deviations
- `wbc_torque_saturated` count is low — gains not constantly saturating

### Regression

All existing tests must pass:
- `test_gait_planner.py` (10 tests)
- `test_heartbeat_gait_wiring.py` (5 tests)
- `test_grf.py` (8 tests)

---

## Out of Scope

- Self-collision avoidance logic (treats symptom, not cause)
- Feedforward torque / trajectory replanning (complexity not justified)
- Hardware changes

---

## Success Criteria

1. Robot completes `--walk 1.0` without tripping
2. No visible oscillation during swing phase
3. Tracking error stays below 0.3 rad throughout gait cycle
4. All existing tests pass
