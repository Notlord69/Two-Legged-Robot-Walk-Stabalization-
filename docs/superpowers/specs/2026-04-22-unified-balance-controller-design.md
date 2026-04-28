# Unified Balance Controller — Design Spec

**Date:** 2026-04-22
**Status:** Draft
**Replaces:** `active_balance.py` (lateral-only, wrong joint axis)

## Problem Statement

The robot falls forward (Y-axis) during COM_SHIFT because no sagittal balance controller exists. COM_Y drifts from 0.25m to 0.97m (0.72 m/s) with zero correction. Both feet lose contact and the robot collapses.

Investigation revealed a deeper structural issue: the existing `active_balance.py` applies "lateral" correction torques to Hip_Forwards (sagittal-plane axis ±X) and Ankle (yaw axis ≈ -Z). Neither joint produces the lateral restoring force intended. The robot stands despite this, not because of it — passive ground reaction forces from the 0.75m lateral foot separation provide the actual lateral stability.

## Root Cause Summary

| Axis | Joint Used by active_balance | Joint Axis (URDF) | Actual Plane of Rotation | Correct Joint |
|---|---|---|---|---|
| Lateral (X) | Hip_Forwards | ±X | Sagittal (Y-Z) | **Hip_Inwards** (axis -Y, frontal X-Z) |
| Lateral (X) | Ankle | ≈ -Z | Yaw | **None useful on this URDF** |
| Sagittal (Y) | *Nothing* | — | — | **Hip_Forwards** (axis ±X, sagittal Y-Z) |

## Design Principle: Compose Position Targets, Not Torques

The current architecture has three modules (active_balance, WBC, GRF) blindly summing torques on Hip_Forwards. Adding a fourth (sagittal balance) would make this worse.

Instead: **balance controllers modify what WBC tracks, not add competing forces.** WBC remains the sole torque authority on every joint. This eliminates torque fighting.

- Lateral balance → position target on Hip_Inwards (same POSITION_CONTROL mode already in use)
- Sagittal balance → position offset on Hip_Forwards IK target (WBC tracks the adjusted setpoint)
- Emergency sagittal torque → direct injection only when CP_y error exceeds threshold (fall imminent)

## Architecture

```
100 Hz Pipeline
│
├─[1] Sensors + State Estimation (stability.py, enhanced)
│      ├── 2D COM position, velocity
│      ├── 2D capture point: ξ = com_xy + v_xy / ω₀
│      └── 2D support polygon margins (lateral + sagittal)
│
├─[2] Balance Controller (NEW balance_controller.py)
│      │
│      ├── Lateral axis (X):
│      │    Input:  ξ_x, stance_foot_x, support polygon x-bounds
│      │    Output: desired_hip_roll (rad) per leg → position target
│      │    Method: LIPM capture point PD → hip roll angle
│      │    Note:   Absorbs gait_planner._compute_hip_roll() — unifies
│      │            balance correction and weight transfer
│      │
│      └── Sagittal axis (Y):
│           Input:  ξ_y, stance_foot_y, support polygon y-bounds
│           Output: sagittal_pitch_offset (rad) → added to IK hip_pitch before WBC
│                   emergency_sagittal_torque (N·m) → direct injection when falling
│           Method: LIPM capture point PD → pitch correction angle
│
├─[3] GRF (existing grf.py, unchanged)
│      └── Feedforward: spring-damper Fz → Jᵀ → Δτ_hip_pitch + Δτ_knee
│
├─[4] Gait Planner (gait_planner.py, simplified)
│      ├── _compute_hip_roll() removed (moved to balance_controller)
│      ├── Phase advance gates on BOTH stability_margin_lateral AND stability_margin_sagittal
│      ├── Generates IK targets: (hip_pitch, knee, ankle) per leg
│      └── Foot placement reads CP_y for future sagittal landing adjustment
│
├─[5] WBC (HeartBeat.py, updated)
│      ├── Hip_Inwards:  POSITION_CONTROL to balance_hip_roll target (same mode as now)
│      ├── Hip_Forwards: TORQUE_CONTROL tracking (IK_pitch + sagittal_pitch_offset)
│      ├── Knee:         TORQUE_CONTROL tracking IK_knee
│      └── Ankle:        TORQUE_CONTROL tracking IK_ankle
│
└─[6] apply_control (HeartBeat.py, updated)
       ├── Hip_Inwards:  POSITION_CONTROL (balance target) — single authority
       ├── Hip_Forwards: WBC + GRF + emergency_sagittal — saturation-aware
       ├── Knee:         WBC + GRF
       └── Ankle:        WBC only (drop fake ankle "lateral" torques)
```

## Joint Ownership Table

| Joint | Control Mode | Authorities | Notes |
|---|---|---|---|
| Hip_Inwards | POSITION | balance_controller (lateral) | Sole owner. Absorbs weight transfer. |
| Hip_Forwards | TORQUE | WBC(IK + sagittal_offset) + GRF + emergency | WBC primary; GRF feedforward; emergency threshold-gated |
| Knee | TORQUE | WBC + GRF | Unchanged |
| Ankle | TORQUE | WBC | Drop active_balance ankle torques (yaw axis, not useful) |
| Hip_Twist | POSITION (default) | Unused | Locked at 0 by PyBullet default motor |

## Module Specifications

### 1. balance_controller.py (NEW — replaces active_balance.py)

#### Class: `BalanceController`

**Constructor state:**
```python
_prev_com_vel_x: float = 0.0          # for lateral accel estimation
_prev_com_vel_y: float = 0.0          # for sagittal accel estimation
_prev_hip_roll_left: float = 0.0      # rate-limiting state
_prev_hip_roll_right: float = 0.0
_prev_pitch_offset: float = 0.0       # rate-limiting state
```

#### Lateral Balance (X-axis)

LIPM capture point in X:
```
ξ_x = x_com + ẋ_com / ω₀
ω₀ = sqrt(g / z_com)
```

Support center during walking phases: `stance_foot_world_pos[0]`
Support center during standing: midpoint of feet in X (both contact confirmed)

Lateral error: `e_x = ξ_x - x_support_center`

Hip roll target (combines balance + weight transfer):
```
target_roll = -(LATERAL_ROLL_GAIN * e_x + LATERAL_KD * ẋ_com)
target_roll = clip(target_roll, -HIP_ROLL_MAX, +HIP_ROLL_MAX)
```

Rate-limited per cycle: `|Δroll| ≤ HIP_ROLL_RATE_LIMIT` (rad/cycle)

Output per leg:
- Stance leg: `balance_hip_roll = target_roll`
- Swing leg: `balance_hip_roll = -target_roll * 0.5` (counter-tilt, attenuated)

During DOUBLE_SUPPORT (no walking): both legs get `target_roll * 0.5` (symmetric correction).

**Constants (tuning, not URDF-derived):**
```python
LATERAL_ROLL_GAIN:   float = 0.8    # rad/m, CP error → hip roll angle
LATERAL_KD:          float = 0.1    # rad·s/m, derivative damping on ẋ_com
HIP_ROLL_MAX:        float = 0.25   # rad (~14°), URDF limit is ±0.698 rad
HIP_ROLL_RATE_LIMIT: float = 0.03   # rad/cycle, smooth transitions
```

#### Sagittal Balance (Y-axis)

LIPM capture point in Y:
```
ξ_y = y_com + ẏ_com / ω₀
```

Support center: `stance_foot_world_pos[1]` during walking, midpoint of feet in Y during standing.

Sagittal error: `e_y = ξ_y - y_support_center`

**Normal mode** (|e_y| < EMERGENCY_THRESHOLD):

Pitch offset modifies IK target:
```
raw_offset = -SAGITTAL_PITCH_GAIN * e_y - SAGITTAL_KD * ẏ_com
sagittal_pitch_offset = clip(raw_offset, -PITCH_OFFSET_MAX, +PITCH_OFFSET_MAX)
```

Rate-limited: `|Δoffset| ≤ PITCH_RATE_LIMIT` rad/cycle.

Written to `shared_state.sagittal_pitch_offset`. WBC reads `IK_hip_pitch + sagittal_pitch_offset` as its target.

**Emergency mode** (|e_y| ≥ EMERGENCY_THRESHOLD):

Direct torque injection in addition to pitch offset:
```
T_emergency = -EMERGENCY_KP * e_y - EMERGENCY_KD * ẏ_com
T_emergency = clip(T_emergency, -EMERGENCY_TORQUE_MAX, +EMERGENCY_TORQUE_MAX)
```

Written to `shared_state.emergency_sagittal_torque` as a dict keyed by URDF joint names.

Emergency torque split logic:
- Both feet in contact: 50/50 split between Left_Hip_Forwards and Right_Hip_Fowards
- Single foot in contact: 100% to the contact-confirmed leg's hip pitch joint
- No contact: 50/50 (best-effort recovery)

Emergency torque ramps down as |e_y| decreases below threshold (hysteresis band to prevent chatter).
Exit condition: `|e_y| < EMERGENCY_THRESHOLD - EMERGENCY_HYSTERESIS` (0.06 m).

**Constants (tuning, not URDF-derived):**
```python
SAGITTAL_PITCH_GAIN:    float = 1.2    # rad/m, CP_y error → pitch offset
SAGITTAL_KD:            float = 0.3    # rad·s/m, velocity damping
PITCH_OFFSET_MAX:       float = 0.15   # rad (~8.6°), prevents IK workspace violation
PITCH_RATE_LIMIT:       float = 0.02   # rad/cycle, smooth transitions

EMERGENCY_THRESHOLD:    float = 0.08   # m, CP_y error triggers direct torque
EMERGENCY_KP:           float = 40.0   # N·m/m, proportional emergency torque
EMERGENCY_KD:           float = 10.0   # N·m·s/m, derivative emergency damping
EMERGENCY_TORQUE_MAX:   float = 50.0   # N·m, below URDF effort limit of 100 N·m
EMERGENCY_HYSTERESIS:   float = 0.02   # m, must drop below threshold - hysteresis to exit
```

#### Public API

```python
def update_balance() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle.
    Writes to shared_state: balance_hip_roll_left, balance_hip_roll_right,
    sagittal_pitch_offset, emergency_sagittal_torque, and diagnostic fields."""

def reset_balance() -> None:
    """Reset controller state after warmup."""

def get_balance_diagnostics() -> dict:
    """Return dict with both axes' state for telemetry."""
```

### 2. stability.py Changes

Current: computes 2D capture point, but stability margin is derived from `polygon.distance(cp_point)` — a single scalar that blends both axes.

Change: decompose the margin into lateral and sagittal components.

```python
# After computing cp_point and polygon:
if polygon.contains(cp_point):
    # Project CP onto polygon edges to get per-axis margin
    cp_x, cp_y = cp_xy
    # Lateral margin: distance from CP_x to nearest X-edge of polygon
    xs = [pt[0] for pt in polygon.exterior.coords]
    stability_margin_lateral = min(cp_x - min(xs), max(xs) - cp_x)
    # Sagittal margin: distance from CP_y to nearest Y-edge of polygon
    ys = [pt[1] for pt in polygon.exterior.coords]
    stability_margin_sagittal = min(cp_y - min(ys), max(ys) - cp_y)
else:
    stability_margin_lateral = 0.0
    stability_margin_sagittal = 0.0
```

Both written to `shared_state.stability_margin_lateral` and `shared_state.stability_margin_sagittal`. The existing `stability_margin` (2D scalar) is kept for backward compatibility.

### 3. gait_planner.py Changes

**Removed:**
- `_compute_hip_roll()` method — logic moves to balance_controller
- `_prev_stance_roll`, `_prev_swing_roll` instance state
- `HIP_ROLL_GAIN`, `HIP_ROLL_MAX`, `HIP_ROLL_RATE_LIMIT` constants

**Modified — `_compute_stance_ik()`:**
```python
def _compute_stance_ik(self) -> None:
    # Hip roll now comes from balance_controller via shared_state
    stance_side = shared_state.stance_side
    if stance_side == "left":
        stance_roll = shared_state.balance_hip_roll_left
    else:
        stance_roll = shared_state.balance_hip_roll_right

    # IK solve unchanged...
    angles = (stance_roll, ik_angles[0], ik_angles[1], ik_angles[2])
```

**Modified — `_update_non_stance_ik_from_joints()`:**
```python
def _update_non_stance_ik_from_joints(self) -> None:
    # Swing roll from balance_controller
    stance_side = shared_state.stance_side
    if stance_side == "right":
        swing_roll = shared_state.balance_hip_roll_left
        shared_state.ik_left_angles = (
            swing_roll,
            jp.get('Left_Hip_Forwards', 0.0),
            # ...
        )
    else:
        swing_roll = shared_state.balance_hip_roll_right
        # ...
```

**Modified — phase gating:**

COM_SHIFT exit condition adds sagittal check:
```python
# Before (lateral only):
cp_close = np.linalg.norm(
    shared_state.capture_point - shared_state.stance_foot_world_pos[:2]
) < COM_SHIFT_THRESHOLD

# After (2D):
cp_close_lateral = abs(
    shared_state.capture_point[0] - shared_state.stance_foot_world_pos[0]
) < COM_SHIFT_THRESHOLD
cp_close_sagittal = abs(
    shared_state.capture_point[1] - shared_state.stance_foot_world_pos[1]
) < COM_SHIFT_SAGITTAL_THRESHOLD
cp_close = cp_close_lateral and cp_close_sagittal
```

New constant:
```python
COM_SHIFT_SAGITTAL_THRESHOLD: float = 0.05  # m, wider than lateral because sagittal dynamics are faster
```

### 4. HeartBeat.py Changes

**target_torques initialisation:**

`active_balance.py` previously initialised `shared_state.target_torques` with zero-valued ankle/hip entries each cycle. With active_balance removed, WBC must initialise the dict itself at the start of `_wbc_step()`:
```python
shared_state.target_torques = {}  # WBC now owns initialisation
```

**WBC (`_wbc_step`):**

Hip_Forwards target incorporates sagittal pitch offset.
The offset is NOT baked into `ik_left_angles` / `ik_right_angles` — those remain pure gait IK outputs. The offset is added at WBC read time so that gait_planner's IK targets stay clean and debuggable:
```python
for idx, jname, sign in _WBC_LEFT_JOINTS:
    theta_target = shared_state.ik_left_angles[idx]
    # Sagittal pitch offset for hip_pitch joint only
    if idx == 1:  # hip_pitch index in 4-tuple
        theta_target += shared_state.sagittal_pitch_offset
    # ... PD computation unchanged
```

Same for `_WBC_RIGHT_JOINTS`.

**`apply_control()`:**

Hip_Inwards position target reads from balance_controller:
```python
hip_roll_joints = {
    'Left_Hip_Inwards':  shared_state.balance_hip_roll_left,
    'Right_Hip_Inwards': shared_state.balance_hip_roll_right,
}
```

Emergency sagittal torque merged into Hip_Forwards:
```python
emergency = getattr(shared_state, 'emergency_sagittal_torque', {})
for jname, raw_torque in torques.items():
    merged = raw_torque + grf_corr.get(jname, 0.0) + emergency.get(jname, 0.0)
    # ... clip and apply
```

**Pipeline order update:**

Replace `active_balance.update_active_balance()` call with `balance_controller.update_balance()`:
```python
# Stage 5: was active_balance, now balance_controller
import balance_controller
balance_controller.update_balance()
```

**Saturation-aware clipping:**

When total torque on Hip_Forwards exceeds URDF effort limit (100 N·m):
1. Preserve emergency_sagittal_torque (safety-critical)
2. Preserve GRF (gravity compensation)
3. Scale WBC component to fit remaining budget

```python
def _saturate_hip_pitch(wbc_tau, grf_tau, emergency_tau, effort_limit):
    """Scale WBC down if total exceeds limit. Preserve safety + feedforward."""
    total = wbc_tau + grf_tau + emergency_tau
    if abs(total) <= effort_limit:
        return total
    # Budget after preserving safety + feedforward
    protected = grf_tau + emergency_tau
    remaining = effort_limit * np.sign(total) - protected
    # Scale WBC to fit
    if abs(wbc_tau) > 1e-6:
        scale = min(1.0, abs(remaining / wbc_tau))
        return protected + wbc_tau * scale
    return np.clip(protected, -effort_limit, effort_limit)
```

### 5. shared_state.py Changes

New fields added to `Siclo1State.__init__()`:
```python
# Balance controller outputs (written by balance_controller.py)
self.balance_hip_roll_left:  float = 0.0   # rad, position target for Left_Hip_Inwards
self.balance_hip_roll_right: float = 0.0   # rad, position target for Right_Hip_Inwards
self.sagittal_pitch_offset:  float = 0.0   # rad, added to IK hip_pitch before WBC
self.emergency_sagittal_torque: Dict[str, float] = {}  # N·m, direct injection

# 2D stability margins (written by stability.py)
self.stability_margin_lateral:  float = 0.0  # m, CP-to-polygon-edge in X
self.stability_margin_sagittal: float = 0.0  # m, CP-to-polygon-edge in Y

# Balance diagnostics
self.capture_point_error_lateral:  float = 0.0  # m, ξ_x - support_center_x
self.capture_point_error_sagittal: float = 0.0  # m, ξ_y - support_center_y
self.balance_mode_lateral:  str = "INACTIVE"    # ACTIVE/INACTIVE
self.balance_mode_sagittal: str = "INACTIVE"    # NORMAL/EMERGENCY/INACTIVE
```

Fields removed:
- `active_balance_mode` — replaced by `balance_mode_lateral` + `balance_mode_sagittal`
- `lateral_error` — replaced by `capture_point_error_lateral`

`reset()` updated to clear new fields.

### 6. Import and Module Registry Changes

**HeartBeat.py imports:**
```python
# Remove:
import active_balance
# Add:
import balance_controller
```

**Pipeline stage name update in shared_state.py:**
```python
STAGE_NAMES: tuple = (
    'sensors', 'link_positions', 'perception', 'stability',
    'balance',          # was 'active_balance'
    'grf', 'gait_planner', 'mission',
    'wbc', 'recovery', 'apply_control', 'step_sim',
)
```

## Files Changed

| File | Action | Scope |
|---|---|---|
| `balance_controller.py` | CREATE | New module, ~250 lines |
| `active_balance.py` | DELETE | Fully replaced |
| `shared_state.py` | MODIFY | Add 9 fields, remove 2, update STAGE_NAMES |
| `stability.py` | MODIFY | Add 2D margin decomposition (~15 lines) |
| `gait_planner.py` | MODIFY | Remove hip_roll logic, add sagittal phase gate |
| `HeartBeat.py` | MODIFY | WBC pitch offset, apply_control balance reads, import swap, saturation clipper |

## Files NOT Changed

| File | Reason |
|---|---|
| `telemetry.py` | Reads existing shared_state columns only |
| `TelemetryRingBuffer` | Fixed 72-column layout, no schema change |
| `grf.py` | Unchanged — feedforward, additive, no conflict |
| `recovery.py` | Reads stability_status, unchanged |
| `mission.py` | Reads/writes mission_state and ramp_gain, unchanged |
| `kinematics.py` | Pure IK solver, no balance awareness |
| `VizBridge` / recorder | Read-only consumers of joint state |
| `Siclo1.urdf` | No structural changes |
| `perception.py` | Contact FSM, unchanged |

## Test Strategy

**Unit tests for balance_controller.py:**
- Lateral: zero error → zero hip roll; positive CP_x error → negative roll (abduct toward error)
- Sagittal: zero error → zero pitch offset; positive CP_y error → negative pitch offset (lean back)
- Emergency: |e_y| > 0.08m → non-zero emergency torque; |e_y| < 0.06m → emergency decays to zero
- Rate limiting: large step input → output changes by at most RATE_LIMIT per cycle
- Freeze/emergency_stop → zero outputs

**Integration tests:**
- Standing: both balance axes keep CP within support polygon
- COM_SHIFT: lateral balance shifts COM toward stance foot (replaces old active_balance test)
- Phase gating: COM_SHIFT does not exit until BOTH lateral and sagittal CP are within threshold

**Regression tests (existing tests that must still pass):**
- `test_gait_planner.py` — IK angle format is still 4-tuple; hip_roll now comes from shared_state instead of internal method
- `test_stance_anchor.py` — stance foot locking unchanged
- `test_wbc_tracking.py` — WBC PD logic unchanged, just reads offset-adjusted target

## Physical Validation Criteria

Before declaring this work complete, verify in simulation:

1. **Standing stability:** Robot stands for 10s with COM drift < 0.02m in both X and Y
2. **Sagittal hold during COM_SHIFT:** COM_Y velocity stays below 0.1 m/s (was 0.72 m/s)
3. **No forward collapse:** COM_Z remains above 0.70m throughout walking attempt
4. **Emergency recovery:** Robot recovers from 0.05m sagittal perturbation without freezing
5. **Weight transfer still works:** Robot reaches SWING phase within 2s of walk command
