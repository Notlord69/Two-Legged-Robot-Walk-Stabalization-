# Drift & Yaw Fix — Design Spec

**Date:** 2026-04-28
**Status:** Approved
**Problem:** Robot exhibits fast translational drift and ~90° yaw rotation (west → south) within 5 seconds of standing. Root causes: URDF leg asymmetry producing consistent torque bias, uncontrolled Hip_Twist joints, and PD-only balance loops with no steady-state error rejection.

---

## 1. URDF Symmetry Fix

### 1.1 Problem

The left leg has spurious X/Y offsets on Knee and Ankle joint origins from a Fusion 360 export artifact. The right leg's geometry is clean (purely vertical Knee/Ankle origins). Additionally, `Right_Hip_Twist` has an asymmetric lower limit (-2° instead of -20°).

### 1.2 Joint Origin Changes

| Joint | Field | Before | After | Rationale |
|-------|-------|--------|-------|-----------|
| `Left_Knee` | origin xyz | `0.031 0.000969 -0.052133` | `0.0 0.0 -0.060661` | Match right leg's clean vertical geometry |
| `Left_Ankle` | origin xyz | `0.1 0.024043 -0.679218` | `0.0 0.0 -0.686961` | Match right leg's clean vertical geometry |
| `Right_Hip_Twist` | limit lower | `-0.034907` | `-0.349066` | Match left hip twist symmetric ±20° range |
| `Left_Hip_Twist` | origin xyz | `0.925 0.2 0.98` | `0.925 0.2 1.018094` | Compensate for left foot height shift (+0.038094 m) |

### 1.3 Height Compensation

Changing the Left Knee/Ankle Z-components shifts the left foot down at neutral pose. To keep both feet at the same height, `Left_Hip_Twist` origin Z is adjusted from 0.98 to 1.018094:

```
Right ankle world Z = 0.82 + 0.08 + 0.017797 - 0.060661 - 0.686961 = 0.170175 m
Left intermediates  = -0.08125 - 0.019047 - 0.060661 - 0.686961 = -0.847919
New Left_Hip_Z      = 0.170175 - (-0.847919) = 1.018094 m
```

Both Hip_Forwards frames land at Z = 0.917797 m. Both ankles at Z = 0.170175 m.

### 1.4 Derived Data Updates

- **`DEFAULT_LINK_DATA`** in `shared_state.py`: Both sides get identical segment lengths:
  - `l_thigh` / `r_thigh`: 0.060661 m
  - `l_shank` / `r_shank`: 0.686961 m
  - `com_local` fractions recomputed from URDF inertial origins
- **`URDF_JOINT_LIMITS`** in `shared_state.py`: `Right_Hip_Twist` lower limit updated to `-0.349066`

### 1.5 What We Do NOT Change

- Joint axes, link masses, link inertia tensors
- `Hip_Inwards` / `Hip_Forwards` origins (mirrored signs are correct for left/right convention; Z magnitude differences of ~1.25 mm and ~1.25 mm are within tolerance)
- Ankle axis Y-component (±0.005365, already mirrored correctly)

### 1.6 Documentation

All changes recorded in `urdf_changes.md` at project root.

---

## 2. Yaw Stabilizer

### 2.1 Problem

`Left_Hip_Twist` and `Right_Hip_Twist` are never commanded by any module. PyBullet's default motor provides negligible resistance, so any net yaw moment spins the robot freely.

### 2.2 Solution

Add `POSITION_CONTROL` on both Hip_Twist joints in `HeartBeat.py:PyBulletInterface.apply_control()`, using the same pattern as the existing Hip_Inwards balance roll control.

### 2.3 Control Parameters

```python
target_position = 0.0   # rad, hold at neutral
force           = 50.0  # N·m, half of URDF effort limit
positionGain    = 0.8   # dimensionless, P gain
velocityGain    = 0.4   # dimensionless, D gain
```

**Gain rationale:**
- `force=50.0 N·m`: Standing yaw moments are small (< 5 N·m after URDF fix). 50 N·m provides large margin while leaving headroom for future intentional yaw steering during walking.
- `positionGain=0.8` / `velocityGain=0.4`: Slightly softer than hip roll control (1.0/0.5) since yaw compliance is more forgiving than lateral balance.

### 2.4 Integration with Existing Control

- Hip_Twist joints added to the skip set in the WBC torque loop (alongside Hip_Inwards) to prevent conflicting control modes.
- Position-controlled joints are grouped: hip_roll (from balance) + yaw_hold (new), both using `POSITION_CONTROL`; remaining joints use `TORQUE_CONTROL` from WBC.

### 2.5 Future Extensibility

The `0.0` target is a constant. When walking with intentional turning is implemented, a yaw planner could write `shared_state.yaw_target` and the stabilizer would track it. No structural changes needed — just replace the constant with a shared_state read.

---

## 3. Balance Controller I-Term

### 3.1 Problem

PD-only balance loops have zero steady-state error rejection. Any persistent perturbation (CoM estimation drift, asymmetric friction, slight ground slope) creates a constant offset that the proportional term can never fully correct.

### 3.2 New Constants

```python
# Lateral integral
LATERAL_KI:  float = 0.05   # rad/(m·s), integral gain on lateral CP error
LATERAL_I_MAX: float = 0.04 # rad, anti-windup clamp on integrator output

# Sagittal integral
SAGITTAL_KI:  float = 0.08  # rad/(m·s), integral gain on sagittal CP error
SAGITTAL_I_MAX: float = 0.03 # rad, anti-windup clamp on integrator output
```

**Gain rationale:**
- KI values are ~6-7% of corresponding KP (lateral KP=0.8, sagittal KP=1.2). Deliberately slow — the integrator creeps toward zero offset over 1-2 seconds, not reacting to transients. Fast integral gains on a balancing robot cause limit-cycle oscillations.
- I_MAX clamps are ~1/6 of the corresponding P-term clip (HIP_ROLL_MAX=0.25, PITCH_OFFSET_MAX=0.15). The integral is a trim correction, never the primary actuator.

### 3.3 New Internal State

```python
self._integral_lateral:  float = 0.0  # m·s, accumulated lateral CP error
self._integral_sagittal: float = 0.0  # m·s, accumulated sagittal CP error
```

### 3.4 Integration Formula (Lateral)

```python
DT = 0.01  # s, 100 Hz fixed timestep
self._integral_lateral += e_x * DT
self._integral_lateral = max(-LATERAL_I_MAX / LATERAL_KI,
                             min(LATERAL_I_MAX / LATERAL_KI,
                                 self._integral_lateral))

target_roll = -(LATERAL_ROLL_GAIN * e_x
                + LATERAL_KD * vx
                + LATERAL_KI * self._integral_lateral)
```

The clamp is on the accumulated integral (`I_MAX / KI`), so `KI * integral` never exceeds `I_MAX` regardless of gain retuning. Same pattern applies to the sagittal axis.

### 3.5 Anti-Windup

Back-calculation anti-windup: the integrator accumulator is clamped each cycle so the integral output term stays within `[-I_MAX, +I_MAX]`. This prevents wind-up during saturation, emergency mode, or prolonged disturbance.

### 3.6 Reset Behavior

`_write_zero_outputs()` and `reset()` zero both integrators. Prevents stale integral from causing a kick after freeze/emergency recovery. The sagittal integrator also resets when emergency mode activates (emergency torque takes over; a wound-up integrator would fight recovery).

### 3.7 What We Do NOT Add

No derivative-on-measurement change. The existing KD terms already use COM velocity (not error derivative), which avoids derivative kick.

---

## 4. Testing Strategy

### 4.1 URDF Symmetry (Static Verification)

- Parse modified URDF: verify left/right Knee and Ankle origins are identical
- Verify both Hip_Twist limits are symmetric (±0.349066 rad)
- Spawn robot at neutral pose, step physics once, read both foot Z positions — equal within 1 mm
- Verify `DEFAULT_LINK_DATA` entries match new URDF geometry
- Verify `URDF_JOINT_LIMITS` dict matches new limit values

### 4.2 Yaw Stabilizer (Unit)

- Apply 5 N·m external yaw torque for 100 cycles. Final base yaw must stay under 2°
- Verify Hip_Twist joints are excluded from WBC torque loop
- Verify stabilizer respects freeze_robot / emergency_stop (zero output when frozen)

### 4.3 I-Term Balance (Unit + Integration)

- Feed constant CP error of 0.01 m for 200 cycles. Verify integrator accumulates and total output exceeds PD-only output
- Feed error that would cause wind-up beyond I_MAX. Verify clamp holds and integrator unwinds on error reversal
- Call `reset()`, verify both integrators return to 0.0
- 30-second standing sim: final base position drift < 5 mm from spawn in X and Y; final base yaw < 1° from spawn heading

### 4.4 Regression

Run all existing tests. Update assertions broken by URDF geometry changes (segment lengths, foot positions) to new symmetric values.

---

## 5. Files Modified

| File | Change |
|------|--------|
| `Siclo1.urdf` | Left_Knee origin, Left_Ankle origin, Right_Hip_Twist limit, Left_Hip_Twist Z |
| `shared_state.py` | `DEFAULT_LINK_DATA` segment lengths + com_local; `URDF_JOINT_LIMITS` Hip_Twist |
| `balance_controller.py` | Add KI/I_MAX constants, integrator state, PID formula, reset logic |
| `HeartBeat.py` | Add Hip_Twist position control in `apply_control()`, skip in torque loop |
| `urdf_changes.md` | New file documenting all URDF modifications |
| Existing tests | Update assertions for new symmetric geometry |

## 6. Scope Exclusions

- No changes to gait_planner.py, stability.py, perception.py, or mission.py
- No gain tuning optimization — initial values are starting points for sim testing
- No yaw steering during walking — yaw target stays at 0.0
- No changes to Hip_Inwards/Hip_Forwards origins (within tolerance)
