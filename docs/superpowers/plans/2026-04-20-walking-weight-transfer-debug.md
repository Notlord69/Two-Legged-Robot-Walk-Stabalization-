# Walking Weight Transfer Debug Session

**Date:** 2026-04-20
**Status:** Root cause identified, fix partially implemented

## Problem Statement

Robot raises left leg slightly but no forward motion. Simulation is stable but stuck.

## Root Cause Analysis

### Primary Issue: Conflicting Control Goals

1. **gait_planner.py (COM_SHIFT):** Waits for capture point within 0.03m of stance foot
2. **active_balance.py:** Targets midpoint between feet when both are in contact

These goals directly conflict. During COM_SHIFT:
- Both feet CONTACT_CONFIRMED → active_balance targets midpoint
- gait_planner needs CP at stance foot (0.26m away from midpoint)
- active_balance fights against weight shift
- Swing foot never unloads (stays ~40N), times out, aborts to DOUBLE_SUPPORT

### Secondary Issue: Missing Hip Roll Control

The URDF has `Left_Hip_Inwards` / `Right_Hip_Inwards` joints (hip roll/abduction), but they are NOT controlled by WBC. Only 3 joints per leg are controlled:
- Hip_Forwards (pitch)
- Knee
- Ankle

Without hip roll control, the robot cannot actively tilt its pelvis to shift weight laterally.

### Tertiary Issue: Unusual Geometry

The robot's stance has ~0.75m lateral foot spread (in X-axis, which is lateral in this URDF):
- Left foot: x ≈ 0.92
- Right foot: x ≈ 0.19
- COM: x ≈ 0.48

This extreme spread requires large hip roll motion to transfer weight.

## Fixes Applied

### Fix 1: Active Balance Target (IMPLEMENTED)
Modified `active_balance.py:_lateral_support_center()` to return stance foot position during walking phases instead of midpoint:

```python
def _lateral_support_center(self) -> float:
    walking_phases = (StepPhase.COM_SHIFT, StepPhase.LIFT,
                      StepPhase.SWING, StepPhase.PLACE)
    if shared_state.step_phase in walking_phases:
        return float(shared_state.stance_foot_world_pos[0])
    # Standing: use midpoint...
```

### Fix 2: Reduced WBC During COM_SHIFT (IMPLEMENTED)
Modified `HeartBeat.py:_wbc_step()` to scale down WBC gains to 30% during COM_SHIFT:

```python
if phase == StepPhase.COM_SHIFT:
    kp = WBC_KP * 0.3
    kd = WBC_KD * 0.3
```

### Fix 3: Increased Active Balance Gains (IMPLEMENTED)
Modified `active_balance.py`:
- `kp_lateral: 40 → 60`
- `kd_lateral: 8 → 12`
- `hip_torque_limit: 60 → 70`

## Fixes Attempted But Reverted

### Attempt: Hip Roll Torque Control (FAILED)
Tried applying torques to Hip_Inwards joints during COM_SHIFT. Issues:
1. PyBullet's default position control fights torques
2. Disabling position control on hip joints causes instability
3. Hip rolls flop to limits (±40°) without proper control

## Hip Roll Position Control (IMPLEMENTED 2026-04-21)

### Changes Made

1. **shared_state.py**: Expanded `ik_left_angles` and `ik_right_angles` from 3-tuple to 4-tuple:
   - New order: `(hip_roll, hip_pitch, knee, ankle)`
   - Hip roll = Hip_Inwards joint (lateral tilt for weight transfer)

2. **HeartBeat.py**:
   - Added Hip_Inwards joints to `_WBC_LEFT_JOINTS` and `_WBC_RIGHT_JOINTS` at index 0
   - WBC keeps full gains on hip roll during COM_SHIFT (reduced gains only for other joints)
   - `apply_control()` uses POSITION_CONTROL for Hip_Inwards joints (overrides PyBullet default motor)
   - Position control gains: positionGain=1.0, velocityGain=0.5, force=100N·m

3. **gait_planner.py**:
   - Added `_compute_hip_roll()` method: returns (stance_roll, swing_roll)
   - During COM_SHIFT: stance hip ABDUCTS (negative), swing hip ADDUCTS (positive) to tilt pelvis toward stance
   - Constants: `HIP_ROLL_GAIN = 0.8 rad/m`, `HIP_ROLL_MAX = 0.25 rad (~14°)`
   - All IK angle assignments updated to 4-tuple format

4. **Tests**: Updated test assertions for 4-tuple IK angles (278 tests passing)

### Current Status

Hip roll position control is working - joints track commanded angles during COM_SHIFT:
```
[c=75] COM_SHIFT stance=right roll_cmd=-0.20 roll_act=-0.20 F=(0,31)
[c=100] COM_SHIFT stance=right roll_cmd=-0.20 roll_act=-0.20 F=(0,82)
```

Robot reaches SWING/PLACE phases, demonstrating weight transfer success:
```
[c=160] LIFT stance=right steps=0
[c=180] SWING stance=right steps=0 F=(0,128) <-- stance foot bearing weight
```

### Remaining Issues

1. **Intermittent contact loss**: Robot bounces/loses contact during transition phases
2. **Dynamic instability**: Hip roll commands may be too abrupt, causing oscillation
3. **Timing**: Robot loses stance contact during SWING (should maintain throughout)

## Next Steps

1. Tune hip roll ramp rate (smooth transition instead of step change)
2. Add damping to hip roll response
3. Reduce position control gains if oscillating
4. Test with GUI to visualize dynamics
5. Consider adding ankle/hip pitch compensation during weight transfer
