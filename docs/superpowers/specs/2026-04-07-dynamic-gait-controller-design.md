# Dynamic Gait Controller — Design Spec
**Date:** 2026-04-07  
**Project:** Siclo1 Bipedal Robot Simulation  
**Status:** Approved

---

## Overview

A three-module Dynamic Gait Controller that enables the robot to walk a commanded distance using physics-based Ground Reaction Force management, Capture-Point-aware foot placement, and a CLI-driven state machine.

**CLI entry point:**
```
python3 main.py --walk 2.0
```

**Module decomposition:**

| File | Responsibility | Analogous to |
|---|---|---|
| `grf.py` | Virtual spring-damper Fz + Jacobian torque correction | `active_balance.py` |
| `gait_planner.py` | Foot target, swing trajectory, IK call | `kinematics.py` |
| `mission.py` | State machine, step counter, CLI distance | new |

**Module call order in HeartBeat (100 Hz loop):**
```
Safety → Stability → ActiveBalance → GRF → GaitPlanner → Mission → WBC
```

---

## Section 1: `grf.py` — Ground Reaction Force Controller

### Purpose
Compute a desired vertical support force Fz using a Virtual Spring-Damper model, then map it to hip/knee torque corrections via the sagittal-plane Jacobian Transpose. Does not replace the IK solver — outputs additive torque corrections on top of position targets.

### Spring-Damper Model

```
F_z_desired = k_spring * (z_rest - z_foot) - b_damper * z_dot_foot
```

Constants (all with unit comments required):
```python
Z_REST      = 0.75    # m, nominal standing leg length (hip to foot vertical)
K_SPRING    = 1589.0  # N/m, supports 8.1 kg with max 5 cm compression: m*g/δ_max
B_DAMPER    = 94.0    # N·s/m, critical damping: 2*sqrt(K*m)*ζ, ζ=0.7
```

Impact spike suppression: `b_damper` is sized for ζ=0.7 (under-critically-damped) to absorb touch-down impulses without triggering the 15 N force threshold in `ContactConfig.force_threshold_confirmed`.

### Jacobian Transpose Mapping

Sagittal-plane 2-link Jacobian (hip pitch + knee, no ankle in sagittal):

```
J = [[∂x/∂θ_hip,  ∂x/∂θ_knee ],
     [∂z/∂θ_hip,  ∂z/∂θ_knee ]]

∂x/∂θ_hip  =  L_thigh*cos(θ_hip) + L_shank*cos(θ_hip + θ_knee)
∂x/∂θ_knee =  L_shank*cos(θ_hip + θ_knee)
∂z/∂θ_hip  = -(L_thigh*sin(θ_hip) + L_shank*sin(θ_hip + θ_knee))
∂z/∂θ_knee = -L_shank*sin(θ_hip + θ_knee)

τ = Jᵀ * [0, F_z]ᵀ   (F_x = 0 for pure vertical support)
```

Output torques use URDF joint name keys (same format as `active_balance.py`) and are clipped to `URDF_JOINT_LIMITS` before writing.

### Gain Ramping

A scalar `ramp_gain ∈ [0.0, 1.0]` from `shared_state.ramp_gain` multiplies all output torques:

```python
τ_output = τ_raw * ramp_gain
```

`ramp_gain` is managed by `mission.py`, not `grf.py`. `grf.py` only reads it.

### Shared State

**Reads:** `left/right_foot_position`, `left/right_foot_velocity`, `joint_positions`, `left/right_foot_contact_state`, `ramp_gain`, `freeze_robot`, `emergency_stop_triggered`

**Writes:** `grf_torque_correction` (new dict field, URDF joint name keys)

### New `shared_state` field
```python
grf_torque_correction: Dict[str, float] = {}  # N·m, per URDF joint name
```

---

## Section 2: `gait_planner.py` — Foot Target + Swing Trajectory

### Purpose
Compute where each foot should land (Capture-Point-adjusted target), generate the swing arc trajectory each cycle, call the existing `kinematics.solve_ik()`, and update `shared_state` IK angles and foot targets.

### Foot Target — CoM Tracking

At toe-off, swing foot target is computed from the Capture Point:

```
x_target = capture_point_x * STEP_TIMING_SCALE + STEP_LENGTH
```

Constants:
```python
STEP_LENGTH        = 0.12   # m, fixed sagittal advance per step
STEP_TIMING_SCALE  = 0.5    # dimensionless, blend factor for CP correction
```

`capture_point` is read from `shared_state.capture_point` (already written by `active_balance.py`). If CoM leans forward, the capture point is ahead of centre, and the foot lands further forward — the Jacobian torque in `grf.py` compensates the resulting load shift.

### Swing Trajectory

Foot height follows a symmetric parabolic arc during swing:

```python
z_swing = SWING_HEIGHT * 4.0 * φ * (1.0 - φ)   # peaks at φ=0.5
x_swing = x_stance + (x_target - x_stance) * φ   # linear forward advance
```

Constants:
```python
SWING_HEIGHT   = 0.04   # m, peak foot clearance above ground
SWING_DURATION = 0.40   # s, full swing phase (40 cycles at 100 Hz)
```

Phase advance each cycle:
```python
shared_state.swing_phase += dt / SWING_DURATION   # φ ∈ [0, 1]
```

At `φ ≥ 1.0`: foot placed, `step_count` incremented, swing side swaps, phase resets to 0.

### IK Call

```python
foot_xyz_rel = (x_swing - hip_x, 0.0, z_swing - hip_z)
angles = kinematics.solve_ik(foot_xyz_rel, side)   # existing solver, no changes
shared_state.ik_left_angles = angles   # or ik_right_angles
```

Reuses `kinematics.solve_ik()` with no modifications. `clamp_foot_target()` inside the solver protects against workspace violations.

### Shared State

**Reads:** `com_position`, `com_velocity`, `capture_point`, `swing_phase`, `left/right_foot_position`, `active_swing_side`, `mission_state`, `ramp_gain`, `freeze_robot`

**Writes:** `swing_phase`, `swing_foot_target`, `left/right_foot_target`, `ik_left_angles`, `ik_right_angles`, `step_count`, `active_swing_side`

### New `shared_state` fields
```python
active_swing_side: str  = "left"   # which leg is currently in swing phase
step_count:        int  = 0        # total steps completed this mission
```

---

## Section 3: `mission.py` — State Machine + CLI

### Purpose
Parse `--walk <distance>`, compute required step count, run the gait state machine, and control `ramp_gain` / `mission_state` to engage and disengage the gait modules.

### State Machine

```
IDLE ──(walk commanded, both feet CONFIRMED)──► RAMP
RAMP ──(ramp_gain == 1.0)──────────────────────► WALK
WALK ──(steps_remaining == 1)──────────────────► DECEL
DECEL ──(steps_remaining == 0)─────────────────► STOP
STOP ──(ramp_gain == 0.0)──────────────────────► IDLE
```

| State | Behaviour |
|---|---|
| `IDLE` | GRF and gait output zero. `active_balance.py` still runs. |
| `RAMP` | `ramp_gain += 1/50` per cycle (0 → 1 over 0.5 s). Entry requires both feet `CONTACT_CONFIRMED`. |
| `WALK` | Full gait active. Step counter increments each touchdown. |
| `DECEL` | `STEP_LENGTH` halved (0.06 m), `K_SPRING` increased 20% to absorb stopping impulse. |
| `STOP` | Swing cancelled. `ramp_gain -= 1/20` per cycle (1 → 0 over 0.2 s). |

### Step Count

```python
STEP_LENGTH   = 0.12        # m — must match gait_planner.py constant
steps_total   = math.ceil(distance_m / STEP_LENGTH)
steps_remaining = steps_total - shared_state.step_count
```

### CLI Integration

Addition to `main.py` argument parser (minimal change):
```python
parser.add_argument("--walk", type=float, default=None, metavar="METRES",
                    help="Walk forward D metres then stop (e.g. --walk 2.0)")
```

Passed into `Siclo1Controller.__init__(walk_distance=args.walk)` → `MissionController.__init__(walk_distance)`. If `--walk` is not provided, `MissionController` stays in `IDLE` indefinitely — no impact on existing behaviour.

### Shared State

**Reads:** `step_count`, `left/right_foot_contact_state`, `freeze_robot`, `emergency_stop_triggered`

**Writes:** `mission_state`, `steps_remaining`, `ramp_gain`

### New `shared_state` fields
```python
mission_state:   MissionState = MissionState.IDLE   # gait state machine
steps_remaining: int          = 0                   # steps until stop
ramp_gain:       float        = 0.0                 # [0,1] torque scale factor
```

### New enum (in `shared_state.py`)
```python
class MissionState(Enum):
    IDLE  = auto()
    RAMP  = auto()
    WALK  = auto()
    DECEL = auto()
    STOP  = auto()
```

---

## Summary of New `shared_state` Fields

| Field | Type | Default | Written by |
|---|---|---|---|
| `grf_torque_correction` | `Dict[str, float]` | `{}` | `grf.py` |
| `active_swing_side` | `str` | `"left"` | `gait_planner.py` |
| `step_count` | `int` | `0` | `gait_planner.py` |
| `mission_state` | `MissionState` | `IDLE` | `mission.py` |
| `steps_remaining` | `int` | `0` | `mission.py` |
| `ramp_gain` | `float` | `0.0` | `mission.py` |

---

## Physical Validity Notes

1. **Workspace constraint:** L_THIGH=60mm, L_SHANK=687mm → R_MIN=0.631m, R_MAX=0.743m. `STEP_LENGTH=0.12m` keeps foot targets within the reachable annulus at typical COM height. Do not increase step length beyond 0.18m without re-checking IK clamping.
2. **Foot separation:** URDF ankle Y separation = 28.6mm. The lateral GRF controller in `grf.py` applies symmetric force only — it does not correct lateral drift. Lateral balance remains `active_balance.py`'s job.
3. **Stop distance error:** `math.ceil` means the robot may overshoot by up to one step length (max 0.12m). This is acceptable for simulation. A future improvement could use fractional step targeting in DECEL.
4. **Impact spikes:** B_DAMPER at ζ=0.7 is intentionally under-critically-damped. If safety shutdowns still trigger on touchdown, increase ζ toward 1.0 (critical) by scaling `B_DAMPER = 2*sqrt(K_SPRING * 8.1)`.

---

## Files to Create / Modify

| Action | File | Notes |
|---|---|---|
| Create | `grf.py` | New module |
| Create | `gait_planner.py` | New module |
| Create | `mission.py` | New module |
| Modify | `shared_state.py` | Add 6 fields + `MissionState` enum |
| Modify | `HeartBeat.py` | Import + call 3 new modules in loop |
| Modify | `main.py` | Add `--walk` argparse argument |
