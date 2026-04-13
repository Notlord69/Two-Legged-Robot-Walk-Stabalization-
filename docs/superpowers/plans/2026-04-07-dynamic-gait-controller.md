# Dynamic Gait Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a three-module Dynamic Gait Controller (`grf.py`, `gait_planner.py`, `mission.py`) that enables `python3 main.py --walk 2.0` to walk a commanded distance using physics-based GRF management and Capture-Point-aware foot placement.

**Architecture:** `grf.py` computes a virtual spring-damper Fz and maps it to hip+knee torque corrections via the sagittal Jacobian transpose. `gait_planner.py` computes Capture-Point-adjusted foot targets, runs the parabolic swing arc, and calls the existing `kinematics.solve_ik()`. `mission.py` runs a five-state machine (IDLE→RAMP→WALK→DECEL→STOP) that controls `ramp_gain` and step counting. A minimal joint-space PD "WBC" step inside `HeartBeat.py` converts IK angle targets to torques and merges GRF corrections before `apply_control()`.

**Tech Stack:** Python 3.10+, PyBullet (via `sim/interface.py`), `numpy`, existing `kinematics.solve_ik()`, `shared_state.py` singleton, `python3 -m pytest`

---

## File Structure

| Action | File | Responsibility |
|---|---|---|
| Modify | `shared_state.py` | Add `MissionState` enum + 7 new fields (6 from spec + `swing_foot_x_stance`) |
| Create | `grf.py` | Virtual spring-damper Fz + Jacobian transpose torque corrections |
| Create | `gait_planner.py` | Capture-Point foot target, parabolic swing arc, `solve_ik()` call |
| Create | `mission.py` | Five-state machine, step counter, `walk_distance` from CLI |
| Modify | `HeartBeat.py` | Import three new modules, WBC PD step, GRF merge, `walk_distance` passthrough |
| Modify | `main.py` | Add `--walk METRES` arg, pass to `Siclo1Controller` |
| Create | `test_gait_shared_state.py` | New shared_state fields + enum |
| Create | `test_grf.py` | Spring-damper math, Jacobian, safety gates |
| Create | `test_gait_planner.py` | Swing trajectory math, IK, step count |
| Create | `test_mission.py` | State machine transitions, ramp_gain, step counting |

---

## Task 1: Add `MissionState` Enum + New `shared_state` Fields

**Files:**
- Modify: `shared_state.py:94-128` (enum section, then `Siclo1State.__init__`)
- Create: `test_gait_shared_state.py`

### Step 1.1 — Write the failing test

- [ ] Create `test_gait_shared_state.py`:

```python
"""Tests for new shared_state fields added for the Dynamic Gait Controller."""
import pytest
from shared_state import shared_state, Siclo1State


def test_mission_state_enum_values():
    from shared_state import MissionState
    assert hasattr(MissionState, 'IDLE')
    assert hasattr(MissionState, 'RAMP')
    assert hasattr(MissionState, 'WALK')
    assert hasattr(MissionState, 'DECEL')
    assert hasattr(MissionState, 'STOP')


def test_new_fields_exist_with_defaults():
    from shared_state import MissionState
    s = Siclo1State()
    assert s.grf_torque_correction == {}
    assert s.active_swing_side == "left"
    assert s.step_count == 0
    assert s.mission_state == MissionState.IDLE
    assert s.steps_remaining == 0
    assert s.ramp_gain == 0.0
    assert s.swing_foot_x_stance == 0.0


def test_grf_torque_correction_is_dict():
    s = Siclo1State()
    s.grf_torque_correction['Left_Hip_Forwards'] = 3.5
    assert s.grf_torque_correction['Left_Hip_Forwards'] == 3.5


def test_ramp_gain_bounds():
    s = Siclo1State()
    assert 0.0 <= s.ramp_gain <= 1.0
```

### Step 1.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_gait_shared_state.py -v`
- Expected: FAIL with `ImportError: cannot import name 'MissionState'`

### Step 1.3 — Add `MissionState` enum to `shared_state.py`

- [ ] In `shared_state.py`, after the `SystemStatus` enum (around line 124), add:

```python
class MissionState(Enum):
    """Gait state machine states for the Dynamic Gait Controller."""
    IDLE  = auto()
    RAMP  = auto()
    WALK  = auto()
    DECEL = auto()
    STOP  = auto()
```

### Step 1.4 — Add new fields to `Siclo1State.__init__`

- [ ] In `Siclo1State.__init__`, after the `# KINEMATICS STATE` block (after line 265), add a new section:

```python
        # ====================================================================
        # DYNAMIC GAIT CONTROLLER STATE
        # ====================================================================

        # Ground Reaction Force correction torques (N·m, per URDF joint name).
        # Written by grf.py; read by HeartBeat.py apply_control merge.
        self.grf_torque_correction: Dict[str, float] = {}

        # Which leg is currently in swing phase.
        # Written by gait_planner.py.
        self.active_swing_side: str = "left"

        # Total steps completed in the current mission.
        # Written by gait_planner.py; read by mission.py.
        self.step_count: int = 0

        # X-position of swing foot at toe-off (m, world frame).
        # Written by gait_planner.py at swing start; persists across loop iterations.
        self.swing_foot_x_stance: float = 0.0

        # Gait state machine state.
        # Written by mission.py; read by grf.py and gait_planner.py.
        self.mission_state: MissionState = MissionState.IDLE

        # Steps remaining until stop command is satisfied.
        # Written by mission.py.
        self.steps_remaining: int = 0

        # Torque scale factor ∈ [0, 1]. Ramped up by RAMP, down by STOP.
        # Written by mission.py; read by grf.py and gait_planner.py.
        self.ramp_gain: float = 0.0
```

### Step 1.5 — Update `MissionState` import in `Siclo1State.__init__` default

- [ ] At the top of `shared_state.py` the `from enum import Enum, auto` import already exists — no change needed.

- [ ] In the `Siclo1State.__init__` new block, the `MissionState.IDLE` reference needs `MissionState` to be defined above the class. Verify the enum was inserted **before** the `class Siclo1State` definition (it was added after `SystemStatus`, which is before `class Siclo1State`). ✓

### Step 1.6 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_gait_shared_state.py -v`
- Expected: 4 tests PASS

### Step 1.7 — Commit

```bash
git add shared_state.py test_gait_shared_state.py
git commit -m "feat: add MissionState enum + 7 gait controller fields to shared_state"
```

---

## Task 2: Create `grf.py` — Ground Reaction Force Controller

**Files:**
- Create: `grf.py`
- Create: `test_grf.py`

### Step 2.1 — Write the failing test

- [ ] Create `test_grf.py`:

```python
"""Tests for grf.py — Ground Reaction Force controller.

Pure-math tests: no PyBullet required. All shared_state fields are set
manually before each call to update_grf().
"""
import math
import pytest
import numpy as np
from shared_state import (
    shared_state, Siclo1State, ContactState, MissionState, URDF_JOINT_NAMES
)


def _reset_for_grf():
    """Set shared_state to a stable standing pose for GRF tests."""
    shared_state.reset()
    # Both feet on ground, confirmed contact
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_foot_position  = np.array([0.0, 0.1, 0.0])   # m, on ground
    shared_state.right_foot_position = np.array([0.0, -0.1, 0.0])  # m, on ground
    shared_state.left_foot_velocity  = np.zeros(3)
    shared_state.right_foot_velocity = np.zeros(3)
    shared_state.ramp_gain   = 1.0
    shared_state.freeze_robot = False
    shared_state.emergency_stop_triggered = False
    # Nominal standing joint angles (geometric ≈ 0 → URDF ≈ 0)
    shared_state.joint_positions = {
        'Left_Hip_Forwards':  0.0,
        'Left_Knee':          0.0,
        'Right_Hip_Fowards':  0.0,
        'Right_Knee':         0.0,
    }
    shared_state.mission_state = MissionState.WALK


def test_zero_force_at_rest_height():
    """F_z = 0 when foot is exactly at Z_REST with zero velocity."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST
    shared_state.right_foot_position[2] = Z_REST
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert abs(v) < 1e-9, f"Expected 0, got {v}"


def test_positive_fz_when_foot_below_rest():
    """Compressed leg (z_foot < Z_REST) → positive Fz → non-zero support torques."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    # Compress both feet 2 cm below rest
    shared_state.left_foot_position[2]  = Z_REST - 0.02
    shared_state.right_foot_position[2] = Z_REST - 0.02
    update_grf()
    corr = shared_state.grf_torque_correction
    assert len(corr) == 4, f"Expected 4 torque keys, got {list(corr.keys())}"
    # All keys present
    assert 'Left_Hip_Forwards' in corr
    assert 'Left_Knee'         in corr
    assert 'Right_Hip_Fowards' in corr
    assert 'Right_Knee'        in corr


def test_zero_output_on_freeze():
    """freeze_robot = True → all corrections zero."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.freeze_robot = True
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_zero_output_on_emergency_stop():
    """emergency_stop_triggered = True → all corrections zero."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.emergency_stop_triggered = True
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_zero_output_on_idle():
    """mission_state == IDLE → all corrections zero (gait not yet active)."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.mission_state = MissionState.IDLE
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_ramp_gain_scales_output():
    """Torque at ramp_gain=0.5 is exactly half of torque at ramp_gain=1.0."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST - 0.03
    shared_state.right_foot_position[2] = Z_REST - 0.03

    shared_state.ramp_gain = 1.0
    update_grf()
    full = dict(shared_state.grf_torque_correction)

    shared_state.ramp_gain = 0.5
    update_grf()
    half = dict(shared_state.grf_torque_correction)

    for k in full:
        assert abs(half[k] - full[k] * 0.5) < 1e-9, (
            f"Key {k}: expected {full[k]*0.5:.6f}, got {half[k]:.6f}"
        )


def test_swing_foot_gets_zero_grf():
    """Swing foot (NO_CONTACT) must not receive GRF torques."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    # Left foot in swing = no contact
    shared_state.set_contact_state('left', ContactState.NO_CONTACT)
    shared_state.left_foot_position[2] = Z_REST - 0.05  # would produce Fz if not gated
    shared_state.right_foot_position[2] = Z_REST - 0.02

    update_grf()
    corr = shared_state.grf_torque_correction
    # Left leg corrections must be zero
    assert corr.get('Left_Hip_Forwards', 0.0) == 0.0
    assert corr.get('Left_Knee',         0.0) == 0.0
    # Right leg corrections non-zero (stance foot compressed)
    assert corr.get('Right_Hip_Fowards', 0.0) != 0.0 or corr.get('Right_Knee', 0.0) != 0.0


def test_decel_increases_k_spring():
    """DECEL state uses K_SPRING * 1.2 → larger correction for same compression."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST - 0.02
    shared_state.right_foot_position[2] = Z_REST - 0.02

    shared_state.mission_state = MissionState.WALK
    update_grf()
    walk_corr = dict(shared_state.grf_torque_correction)

    shared_state.mission_state = MissionState.DECEL
    update_grf()
    decel_corr = dict(shared_state.grf_torque_correction)

    for k in walk_corr:
        if abs(walk_corr[k]) > 1e-9:
            ratio = decel_corr[k] / walk_corr[k]
            assert abs(ratio - 1.2) < 1e-6, f"Key {k}: expected ratio 1.2, got {ratio:.6f}"
```

### Step 2.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_grf.py -v`
- Expected: FAIL with `ModuleNotFoundError: No module named 'grf'`

### Step 2.3 — Create `grf.py`

- [ ] Create `grf.py`:

```python
"""
================================================================================
PROJECT SICLO1 — GROUND REACTION FORCE CONTROLLER  (grf.py)
================================================================================

Virtual Spring-Damper Fz + Jacobian Transpose torque corrections.

Model:
    F_z = K_SPRING * (Z_REST - z_foot) - B_DAMPER * z_dot_foot

Sagittal 2-link Jacobian (hip-pitch + knee, no ankle):
    ∂z/∂θ_hip  = -(L_thigh*sin(θ_hip) + L_shank*sin(θ_hip + θ_knee))
    ∂z/∂θ_knee = -L_shank*sin(θ_hip + θ_knee)
    τ = Jᵀ * [0, F_z]ᵀ

URDF axis sign convention:
    Left  hip/knee  axis = -X  →  θ_geo = -q_urdf  →  τ_urdf = -τ_geo
    Right hip/knee  axis = +X  →  θ_geo =  q_urdf  →  τ_urdf =  τ_geo

Outputs are additive corrections layered on top of active_balance target_torques.
Scaled by shared_state.ramp_gain. Only applied to stance (CONTACT_CONFIRMED) feet.
Applied only when mission_state != IDLE.

INPUTS (shared_state):
    left/right_foot_position, left/right_foot_velocity,
    joint_positions, left/right_foot_contact_state,
    ramp_gain, mission_state, freeze_robot, emergency_stop_triggered

OUTPUTS (shared_state):
    grf_torque_correction  — Dict[str, float], URDF joint name keys, N·m

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
from typing import Dict

from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
)


# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

# Leg geometry — left leg is canonical per kinematics.py
L_THIGH: float = DEFAULT_LINK_DATA['l_thigh']['length']  # m, hip-pitch to knee
L_SHANK: float = DEFAULT_LINK_DATA['l_shank']['length']  # m, knee to ankle

# Spring-damper constants (sized for 8.1 kg robot):
#   K_SPRING = m*g / δ_max  =  8.1 * 9.81 / 0.05  ≈  1589 N/m
#   B_DAMPER = 2 * sqrt(K * m) * ζ, ζ=0.7
Z_REST:   float = 0.75     # m,     nominal standing leg length (hip to foot, vertical)
K_SPRING: float = 1589.0   # N/m,   supports 8.1 kg with max 5 cm compression
B_DAMPER: float = 94.0     # N·s/m, ζ=0.7 under-critical damping for impact absorption

# DECEL boost factor applied to K_SPRING in DECEL state to absorb stopping impulse
DECEL_SPRING_BOOST: float = 1.2   # dimensionless, 20% stiffness increase in DECEL


def _spring_damper_fz(z_foot: float, z_dot_foot: float,
                      k_spring: float) -> float:
    """Compute desired vertical support force.

    z_foot    : foot z position in world frame (m)
    z_dot_foot: foot z velocity in world frame (m/s)
    k_spring  : spring constant to use (N/m) — caller may override for DECEL
    Returns F_z (N), positive = upward support.
    """
    return k_spring * (Z_REST - z_foot) - B_DAMPER * z_dot_foot


def _jacobian_torques(fz: float,
                      theta_hip_geo: float,
                      theta_knee_geo: float) -> tuple:
    """Sagittal Jacobian transpose: map Fz to (τ_hip_geo, τ_knee_geo).

    Uses geometric angles (positive = forward flex, axis-sign-agnostic).
    Returns geometric torques; caller applies URDF sign convention.

    τ_hip  = (∂z/∂θ_hip)  * Fz = -(L_thigh*sin(θ_hip) + L_shank*sin(θ_hip+θ_knee)) * Fz
    τ_knee = (∂z/∂θ_knee) * Fz = -L_shank*sin(θ_hip+θ_knee) * Fz
    """
    sum_angle = theta_hip_geo + theta_knee_geo
    dz_dhip  = -(L_THIGH * math.sin(theta_hip_geo) +
                 L_SHANK * math.sin(sum_angle))
    dz_dknee = -L_SHANK * math.sin(sum_angle)
    return dz_dhip * fz, dz_dknee * fz


def _clip(joint_name: str, value: float) -> float:
    """Clip torque to URDF effort limit."""
    lim = URDF_JOINT_LIMITS.get(joint_name)
    if lim is None:
        return value
    e = lim['effort']
    return max(-e, min(e, value))


def _compute_leg_correction(
    z_foot: float,
    z_dot_foot: float,
    q_hip_urdf: float,
    q_knee_urdf: float,
    urdf_sign: float,
    hip_key: str,
    knee_key: str,
    k_spring: float,
    ramp_gain: float,
) -> Dict[str, float]:
    """Compute GRF torque corrections for one leg.

    urdf_sign: +1.0 for right (axis=+X), -1.0 for left (axis=-X).
    Returns {hip_key: τ, knee_key: τ} with URDF-signed, clipped, gain-scaled torques.
    """
    # Convert URDF joint angles to geometric angles
    # Left (urdf_sign=-1): θ_geo = -q_urdf   Right (urdf_sign=+1): θ_geo = q_urdf
    theta_hip_geo  = urdf_sign * q_hip_urdf
    theta_knee_geo = urdf_sign * q_knee_urdf

    fz = _spring_damper_fz(z_foot, z_dot_foot, k_spring)

    tau_hip_geo, tau_knee_geo = _jacobian_torques(fz, theta_hip_geo, theta_knee_geo)

    # Apply URDF axis sign (geometric → URDF): left negates, right keeps
    tau_hip_urdf  = urdf_sign * tau_hip_geo
    tau_knee_urdf = urdf_sign * tau_knee_geo

    # Scale by ramp_gain and clip to joint limits
    return {
        hip_key:  _clip(hip_key,  tau_hip_urdf  * ramp_gain),
        knee_key: _clip(knee_key, tau_knee_urdf * ramp_gain),
    }


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

class GRFController:
    """Stateless per-cycle GRF controller. Reads/writes shared_state."""

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        zero = {
            'Left_Hip_Forwards': 0.0,
            'Left_Knee':         0.0,
            'Right_Hip_Fowards': 0.0,
            'Right_Knee':        0.0,
        }

        # Safety gate
        if (shared_state.freeze_robot or
                shared_state.emergency_stop_triggered or
                shared_state.mission_state == MissionState.IDLE):
            shared_state.grf_torque_correction = zero
            return

        ramp_gain = shared_state.ramp_gain

        # DECEL: increase K_SPRING by 20% to absorb stopping impulse
        k_spring = (K_SPRING * DECEL_SPRING_BOOST
                    if shared_state.mission_state == MissionState.DECEL
                    else K_SPRING)

        jp = shared_state.joint_positions
        result: Dict[str, float] = {}

        # Left leg — axis = -X → urdf_sign = -1.0
        if shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED:
            result.update(_compute_leg_correction(
                z_foot      = float(shared_state.left_foot_position[2]),
                z_dot_foot  = float(shared_state.left_foot_velocity[2]),
                q_hip_urdf  = jp.get('Left_Hip_Forwards', 0.0),
                q_knee_urdf = jp.get('Left_Knee', 0.0),
                urdf_sign   = -1.0,
                hip_key     = 'Left_Hip_Forwards',
                knee_key    = 'Left_Knee',
                k_spring    = k_spring,
                ramp_gain   = ramp_gain,
            ))
        else:
            result['Left_Hip_Forwards'] = 0.0
            result['Left_Knee']         = 0.0

        # Right leg — axis = +X → urdf_sign = +1.0
        if shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED:
            result.update(_compute_leg_correction(
                z_foot      = float(shared_state.right_foot_position[2]),
                z_dot_foot  = float(shared_state.right_foot_velocity[2]),
                q_hip_urdf  = jp.get('Right_Hip_Fowards', 0.0),
                q_knee_urdf = jp.get('Right_Knee', 0.0),
                urdf_sign   = +1.0,
                hip_key     = 'Right_Hip_Fowards',
                knee_key    = 'Right_Knee',
                k_spring    = k_spring,
                ramp_gain   = ramp_gain,
            ))
        else:
            result['Right_Hip_Fowards'] = 0.0
            result['Right_Knee']        = 0.0

        shared_state.grf_torque_correction = result


_grf_controller = GRFController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_grf() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _grf_controller.update()
```

### Step 2.4 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_grf.py -v`
- Expected: 8 tests PASS

### Step 2.5 — Commit

```bash
git add grf.py test_grf.py
git commit -m "feat: add GRF controller with virtual spring-damper + Jacobian transpose"
```

---

## Task 3: Create `gait_planner.py` — Foot Target + Swing Trajectory

**Files:**
- Create: `gait_planner.py`
- Create: `test_gait_planner.py`

### Step 3.1 — Write the failing test

- [ ] Create `test_gait_planner.py`:

```python
"""Tests for gait_planner.py — pure-math, no PyBullet required.

shared_state fields populated manually before each test.
"""
import math
import numpy as np
import pytest
from shared_state import shared_state, Siclo1State, ContactState, MissionState


def _reset_for_planner(mission_state=MissionState.WALK):
    shared_state.reset()
    shared_state.mission_state      = mission_state
    shared_state.ramp_gain          = 1.0
    shared_state.freeze_robot       = False
    shared_state.swing_phase        = 0.0
    shared_state.active_swing_side  = "left"
    shared_state.step_count         = 0
    shared_state.swing_foot_x_stance = 0.0
    shared_state.last_dt            = 0.01  # 100 Hz
    # CoM: stable upright position
    shared_state.com_position = np.array([0.0, 0.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    # Capture point (written by active_balance; simulate here)
    shared_state.capture_point = np.array([0.05, 0.0])   # slight forward lean
    # Hip link positions (world frame — normally from PyBullet getLinkState)
    shared_state.link_positions = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.1, 0.75]),
        'Right_Upper_Leg_1': np.array([0.0, -0.1, 0.75]),
    }
    # Foot positions
    shared_state.left_foot_position  = np.array([0.0, 0.1, 0.0])
    shared_state.right_foot_position = np.array([0.0, -0.1, 0.0])
    # Both feet on ground
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)


def test_no_update_when_idle():
    """IDLE state: swing_phase stays 0, step_count stays 0."""
    from gait_planner import update_gait_planner
    _reset_for_planner(mission_state=MissionState.IDLE)
    update_gait_planner()
    assert shared_state.swing_phase == 0.0
    assert shared_state.step_count == 0


def test_no_update_on_freeze():
    """freeze_robot: swing_phase stays 0."""
    from gait_planner import update_gait_planner
    _reset_for_planner()
    shared_state.freeze_robot = True
    update_gait_planner()
    assert shared_state.swing_phase == 0.0


def test_swing_phase_advances():
    """Each call advances swing_phase by dt / SWING_DURATION."""
    from gait_planner import update_gait_planner, SWING_DURATION
    _reset_for_planner()
    update_gait_planner()
    expected_phi = 0.01 / SWING_DURATION
    assert abs(shared_state.swing_phase - expected_phi) < 1e-9


def test_swing_trajectory_x_at_start():
    """At φ just after 0, x_swing ≈ x_stance (no advance yet)."""
    from gait_planner import update_gait_planner, SWING_DURATION
    _reset_for_planner()
    # x_stance for left foot = 0.0
    shared_state.swing_foot_x_stance = 0.0
    shared_state.swing_phase = 0.001  # tiny non-zero so planner uses existing x_stance
    update_gait_planner()
    # At φ≈0, left IK should be near-zero x_rel (foot near directly below hip)
    # Just check IK angles were written (not nan)
    assert not any(math.isnan(a) for a in shared_state.ik_left_angles)


def test_swing_trajectory_peak_height():
    """At φ=0.5, z_swing == SWING_HEIGHT (parabolic peak)."""
    from gait_planner import _swing_z, SWING_HEIGHT
    assert abs(_swing_z(0.5) - SWING_HEIGHT) < 1e-9


def test_swing_trajectory_zero_at_endpoints():
    """At φ=0 and φ=1, z_swing == 0 (ground level)."""
    from gait_planner import _swing_z
    assert _swing_z(0.0) == 0.0
    assert _swing_z(1.0) == 0.0


def test_step_count_increments_at_phase_end():
    """When swing_phase reaches 1.0, step_count increments and phase resets."""
    from gait_planner import update_gait_planner, SWING_DURATION
    _reset_for_planner()
    # Set phase just below 1.0; one more dt will push it to >= 1.0
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) - 1e-9
    update_gait_planner()
    assert shared_state.step_count == 1
    assert shared_state.swing_phase == 0.0


def test_swing_side_swaps_after_step():
    """active_swing_side flips left↔right after each step completes."""
    from gait_planner import update_gait_planner, SWING_DURATION
    _reset_for_planner()
    shared_state.active_swing_side = "left"
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) - 1e-9
    update_gait_planner()
    assert shared_state.active_swing_side == "right"


def test_ik_angles_written_for_active_swing():
    """After one update in WALK, ik_left_angles is a 3-tuple of floats."""
    from gait_planner import update_gait_planner
    _reset_for_planner()
    shared_state.swing_phase = 0.0
    update_gait_planner()
    angles = shared_state.ik_left_angles
    assert len(angles) == 3
    assert all(isinstance(a, float) for a in angles)


def test_decel_halves_step_length():
    """In DECEL, x_target uses STEP_LENGTH * 0.5."""
    from gait_planner import _compute_x_target, STEP_LENGTH, STEP_TIMING_SCALE
    shared_state.capture_point = np.array([0.05, 0.0])
    cp_x = 0.05
    target_walk  = _compute_x_target(cp_x, decel=False)
    target_decel = _compute_x_target(cp_x, decel=True)
    expected_walk  = cp_x * STEP_TIMING_SCALE + STEP_LENGTH
    expected_decel = cp_x * STEP_TIMING_SCALE + STEP_LENGTH * 0.5
    assert abs(target_walk  - expected_walk)  < 1e-9
    assert abs(target_decel - expected_decel) < 1e-9
```

### Step 3.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_gait_planner.py -v`
- Expected: FAIL with `ModuleNotFoundError: No module named 'gait_planner'`

### Step 3.3 — Create `gait_planner.py`

- [ ] Create `gait_planner.py`:

```python
"""
================================================================================
PROJECT SICLO1 — GAIT PLANNER  (gait_planner.py)
================================================================================

Compute Capture-Point-adjusted foot targets, run the parabolic swing arc,
and call kinematics.solve_ik() to produce joint angle targets.

Foot target (at toe-off):
    x_target = capture_point_x * STEP_TIMING_SCALE + STEP_LENGTH
    (STEP_LENGTH halved in DECEL state to absorb stopping)

Swing trajectory (parabolic arc):
    z_swing = SWING_HEIGHT * 4 * φ * (1 - φ)    # peaks at φ=0.5
    x_swing = x_stance + (x_target - x_stance) * φ

Phase advance per cycle:
    shared_state.swing_phase += dt / SWING_DURATION

At φ ≥ 1.0: foot placed, step_count++, active_swing_side flips, phase resets.

IK call (no modifications to kinematics.py):
    foot_xyz_rel = (x_swing - hip_x, 0.0, z_swing - hip_z)
    angles = kinematics.solve_ik(foot_xyz_rel, side)

INPUTS (shared_state):
    com_position, com_velocity, capture_point, swing_phase, swing_foot_x_stance,
    left/right_foot_position, active_swing_side, mission_state, ramp_gain,
    freeze_robot, link_positions, last_dt

OUTPUTS (shared_state):
    swing_phase, swing_foot_x_stance, swing_foot_target, left/right_foot_target,
    ik_left_angles, ik_right_angles, step_count, active_swing_side

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import numpy as np

import kinematics
from shared_state import shared_state, MissionState


# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH:       float = 0.12   # m, fixed sagittal advance per step (nominal)
STEP_TIMING_SCALE: float = 0.5    # dimensionless, blend factor for CP correction

SWING_HEIGHT:   float = 0.04   # m, peak foot clearance above ground at φ=0.5
SWING_DURATION: float = 0.40   # s, full swing phase (40 cycles at 100 Hz)

# Hip link names in shared_state.link_positions (verified HeartBeat.py 2026-04-05)
_LEFT_HIP_LINK:  str = "Left_Upper_Leg_1"
_RIGHT_HIP_LINK: str = "Right_Upper_Leg_1"


# ============================================================================
# HELPERS
# ============================================================================

def _swing_z(phi: float) -> float:
    """Parabolic foot height during swing.

    phi: normalized swing phase ∈ [0, 1]
    Returns z_foot (m) above ground; 0 at start and end, SWING_HEIGHT at mid.
    """
    return SWING_HEIGHT * 4.0 * phi * (1.0 - phi)


def _compute_x_target(capture_point_x: float, decel: bool = False) -> float:
    """Compute foot landing x-position from Capture Point.

    capture_point_x: x-component of CP in world frame (m)
    decel: True in DECEL state → halve STEP_LENGTH to absorb stopping impulse
    Returns target x (m, world frame).
    """
    step = STEP_LENGTH * 0.5 if decel else STEP_LENGTH
    return capture_point_x * STEP_TIMING_SCALE + step


# ============================================================================
# GAIT PLANNER CONTROLLER
# ============================================================================

class GaitPlannerController:
    """Per-cycle gait planner. Reads/writes shared_state."""

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        # Safety / gate
        if (shared_state.freeze_robot or
                shared_state.mission_state == MissionState.IDLE):
            return

        dt = shared_state.last_dt
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01   # fallback to 100 Hz nominal

        # Which leg is swinging?
        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK

        # Hip position in world frame
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # Foot stance position (position at toe-off, stored when swing starts)
        # At phase == 0.0 (swing start), snapshot current swing foot position
        phi = shared_state.swing_phase
        if phi == 0.0:
            foot_pos = (shared_state.left_foot_position
                        if side == "left"
                        else shared_state.right_foot_position)
            shared_state.swing_foot_x_stance = float(foot_pos[0])

        x_stance = shared_state.swing_foot_x_stance

        # Compute foot target (at toe-off, captured once per swing cycle)
        # For simplicity, recompute each cycle — CP may update mid-swing
        cp_x = float(getattr(shared_state, 'capture_point', np.zeros(2))[0])
        decel = (shared_state.mission_state == MissionState.DECEL)
        x_target = _compute_x_target(cp_x, decel=decel)
        shared_state.swing_foot_target = (x_target, 0.0, 0.0)

        if side == "left":
            shared_state.left_foot_target  = (x_target, 0.0, 0.0)
        else:
            shared_state.right_foot_target = (x_target, 0.0, 0.0)

        # Advance swing phase
        phi += dt / SWING_DURATION
        shared_state.swing_phase = phi

        # Compute swing foot position along arc
        phi_clamped = min(phi, 1.0)
        x_swing     = x_stance + (x_target - x_stance) * phi_clamped
        z_swing     = _swing_z(phi_clamped)

        # IK: foot position relative to hip-pitch joint
        foot_xyz_rel = (x_swing - hip_x, 0.0, z_swing - hip_z)
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)

        if side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

        # Step completion: φ ≥ 1.0
        if phi >= 1.0:
            shared_state.step_count    += 1
            shared_state.swing_phase    = 0.0
            # Flip swing side
            shared_state.active_swing_side = (
                "right" if side == "left" else "left"
            )


_gait_planner = GaitPlannerController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_gait_planner() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _gait_planner.update()
```

### Step 3.4 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_gait_planner.py -v`
- Expected: 10 tests PASS

### Step 3.5 — Commit

```bash
git add gait_planner.py test_gait_planner.py
git commit -m "feat: add gait planner with CP foot targeting and parabolic swing arc"
```

---

## Task 4: Create `mission.py` — State Machine + CLI

**Files:**
- Create: `mission.py`
- Create: `test_mission.py`

### Step 4.1 — Write the failing test

- [ ] Create `test_mission.py`:

```python
"""Tests for mission.py state machine — no PyBullet required."""
import math
import pytest
from shared_state import shared_state, Siclo1State, ContactState, MissionState


def _reset_for_mission():
    shared_state.reset()
    shared_state.mission_state   = MissionState.IDLE
    shared_state.ramp_gain       = 0.0
    shared_state.step_count      = 0
    shared_state.steps_remaining = 0
    shared_state.freeze_robot    = False
    shared_state.emergency_stop_triggered = False
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)


def test_stays_idle_without_walk_distance():
    """No walk_distance → mission stays IDLE indefinitely."""
    from mission import MissionController
    mc = MissionController(walk_distance=None)
    _reset_for_mission()
    for _ in range(100):
        mc.update()
    assert shared_state.mission_state == MissionState.IDLE
    assert shared_state.ramp_gain == 0.0


def test_stays_idle_until_both_feet_confirmed():
    """With walk_distance set but feet not CONFIRMED → stay IDLE."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    # Only left foot confirmed
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    mc.update()
    assert shared_state.mission_state == MissionState.IDLE


def test_transitions_idle_to_ramp_when_both_feet_confirmed():
    """Both feet CONFIRMED + walk_distance set → IDLE transitions to RAMP."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()
    assert shared_state.mission_state == MissionState.RAMP


def test_ramp_increments_ramp_gain():
    """RAMP state increments ramp_gain by 1/50 per cycle."""
    from mission import MissionController, RAMP_RATE
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    assert shared_state.mission_state == MissionState.RAMP
    before = shared_state.ramp_gain
    mc.update()  # RAMP cycle 1
    after = shared_state.ramp_gain
    assert abs(after - before - RAMP_RATE) < 1e-9


def test_ramp_transitions_to_walk_at_full_gain():
    """After 50 RAMP cycles, ramp_gain reaches 1.0 and state → WALK."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()
    assert abs(shared_state.ramp_gain - 1.0) < 1e-9
    assert shared_state.mission_state == MissionState.WALK


def test_steps_remaining_computed_correctly():
    """steps_remaining = ceil(distance / STEP_LENGTH) - step_count."""
    from mission import MissionController, STEP_LENGTH
    mc = MissionController(walk_distance=0.5)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()  # RAMP → WALK
    assert shared_state.mission_state == MissionState.WALK
    expected_total = math.ceil(0.5 / STEP_LENGTH)
    assert shared_state.steps_remaining == expected_total


def test_walk_transitions_to_decel_at_one_step_remaining():
    """WALK → DECEL when steps_remaining == 1."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)  # exactly 1 step
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()  # reach WALK
    # steps_remaining should be 1 for 0.12 m / 0.12 m = 1 step
    assert shared_state.mission_state == MissionState.WALK
    assert shared_state.steps_remaining == 1
    mc.update()  # WALK → DECEL
    assert shared_state.mission_state == MissionState.DECEL


def test_decel_transitions_to_stop_at_zero_steps():
    """DECEL → STOP when steps_remaining == 0."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    # Force into DECEL with 0 steps remaining
    shared_state.mission_state   = MissionState.DECEL
    shared_state.steps_remaining = 0
    shared_state.ramp_gain       = 1.0
    mc.update()
    assert shared_state.mission_state == MissionState.STOP


def test_stop_decrements_ramp_gain():
    """STOP state decrements ramp_gain by 1/20 per cycle."""
    from mission import MissionController, STOP_RATE
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.mission_state   = MissionState.STOP
    shared_state.ramp_gain       = 1.0
    mc.update()
    assert abs(shared_state.ramp_gain - (1.0 - STOP_RATE)) < 1e-9


def test_stop_transitions_to_idle_at_zero_gain():
    """After 20 STOP cycles, ramp_gain → 0.0 and state → IDLE."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.mission_state = MissionState.STOP
    shared_state.ramp_gain     = 1.0
    for _ in range(20):
        mc.update()
    assert abs(shared_state.ramp_gain) < 1e-9
    assert shared_state.mission_state == MissionState.IDLE


def test_emergency_stop_exits_immediately():
    """emergency_stop_triggered = True → mission goes to IDLE, ramp_gain = 0."""
    from mission import MissionController
    mc = MissionController(walk_distance=2.0)
    _reset_for_mission()
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain     = 1.0
    shared_state.emergency_stop_triggered = True
    mc.update()
    assert shared_state.ramp_gain == 0.0
    assert shared_state.mission_state == MissionState.IDLE
```

### Step 4.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_mission.py -v`
- Expected: FAIL with `ModuleNotFoundError: No module named 'mission'`

### Step 4.3 — Create `mission.py`

- [ ] Create `mission.py`:

```python
"""
================================================================================
PROJECT SICLO1 — MISSION CONTROLLER  (mission.py)
================================================================================

Five-state gait state machine:

    IDLE ──(walk commanded, both feet CONFIRMED)──► RAMP
    RAMP ──(ramp_gain == 1.0)──────────────────────► WALK
    WALK ──(steps_remaining == 1)──────────────────► DECEL
    DECEL ──(steps_remaining == 0)─────────────────► STOP
    STOP ──(ramp_gain == 0.0)──────────────────────► IDLE

RAMP: ramp_gain += RAMP_RATE (1/50) per cycle → 0.5 s to full torque
STOP: ramp_gain -= STOP_RATE (1/20) per cycle → 0.2 s to zero torque

STEP_LENGTH must match gait_planner.py's constant (0.12 m).

INPUTS (shared_state):
    step_count, left/right_foot_contact_state, freeze_robot, emergency_stop_triggered

OUTPUTS (shared_state):
    mission_state, steps_remaining, ramp_gain

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
from typing import Optional

from shared_state import shared_state, ContactState, MissionState


# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH: float = 0.12   # m, must match gait_planner.STEP_LENGTH exactly

RAMP_RATE: float = 1.0 / 50.0   # dimensionless/cycle, 0→1 over 0.5 s at 100 Hz
STOP_RATE: float = 1.0 / 20.0   # dimensionless/cycle, 1→0 over 0.2 s at 100 Hz


# ============================================================================
# MISSION CONTROLLER
# ============================================================================

class MissionController:
    """Gait state machine. One instance per simulation run.

    walk_distance: metres to walk before stopping. None → stay IDLE forever.
    """

    def __init__(self, walk_distance: Optional[float] = None):
        self.walk_distance: Optional[float] = walk_distance
        self._steps_total: int = (
            math.ceil(walk_distance / STEP_LENGTH)
            if walk_distance is not None
            else 0
        )

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        # Emergency stop: collapse everything immediately
        if shared_state.emergency_stop_triggered:
            shared_state.ramp_gain     = 0.0
            shared_state.mission_state = MissionState.IDLE
            return

        if shared_state.freeze_robot:
            return

        state = shared_state.mission_state

        if state == MissionState.IDLE:
            self._handle_idle()

        elif state == MissionState.RAMP:
            self._handle_ramp()

        elif state == MissionState.WALK:
            self._handle_walk()

        elif state == MissionState.DECEL:
            self._handle_decel()

        elif state == MissionState.STOP:
            self._handle_stop()

    # ------------------------------------------------------------------ #
    # STATE HANDLERS
    # ------------------------------------------------------------------ #

    def _handle_idle(self) -> None:
        """Transition to RAMP only when walk_distance is set and both feet confirmed."""
        if (self.walk_distance is not None and
                shared_state.left_foot_contact_state  == ContactState.CONTACT_CONFIRMED and
                shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED):
            shared_state.mission_state   = MissionState.RAMP
            shared_state.ramp_gain       = 0.0
            shared_state.step_count      = 0
            shared_state.steps_remaining = self._steps_total

    def _handle_ramp(self) -> None:
        """Increment ramp_gain. Transition to WALK when gain reaches 1.0."""
        shared_state.ramp_gain = min(1.0, shared_state.ramp_gain + RAMP_RATE)
        if shared_state.ramp_gain >= 1.0:
            shared_state.ramp_gain     = 1.0
            shared_state.mission_state = MissionState.WALK

    def _handle_walk(self) -> None:
        """Update steps_remaining. Transition to DECEL when 1 step left."""
        shared_state.steps_remaining = (
            self._steps_total - shared_state.step_count
        )
        if shared_state.steps_remaining <= 1:
            shared_state.mission_state = MissionState.DECEL

    def _handle_decel(self) -> None:
        """Update steps_remaining. Transition to STOP when no steps left."""
        shared_state.steps_remaining = max(
            0, self._steps_total - shared_state.step_count
        )
        if shared_state.steps_remaining == 0:
            shared_state.mission_state = MissionState.STOP

    def _handle_stop(self) -> None:
        """Decrement ramp_gain. Transition to IDLE when gain reaches 0."""
        shared_state.ramp_gain = max(0.0, shared_state.ramp_gain - STOP_RATE)
        if shared_state.ramp_gain <= 0.0:
            shared_state.ramp_gain     = 0.0
            shared_state.mission_state = MissionState.IDLE
```

### Step 4.4 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_mission.py -v`
- Expected: 11 tests PASS

### Step 4.5 — Commit

```bash
git add mission.py test_mission.py
git commit -m "feat: add mission state machine (IDLE→RAMP→WALK→DECEL→STOP)"
```

---

## Task 5: Wire `HeartBeat.py` — Imports, Call Order, WBC Step, GRF Merge

**Files:**
- Modify: `HeartBeat.py`

The changes are:
1. Import `grf`, `gait_planner`, `mission`
2. `Siclo1Controller.__init__` accepts `walk_distance=None` and creates `MissionController`
3. `step()` inserts GRF → GaitPlanner → Mission after `active_balance`, then WBC step before `apply_control`
4. `_warmup()` includes the three new modules
5. `PyBulletInterface.apply_control()` merges `grf_torque_correction` into final torques
6. New `_wbc_step()` converts IK angles to PD torques and adds to `target_torques`

### Step 5.1 — Write the failing test

- [ ] Create `test_heartbeat_gait_wiring.py`:

```python
"""Integration smoke test: verify gait wiring in HeartBeat initialises cleanly.

Requires PyBullet. Does NOT test physics — only that:
1. Siclo1Controller accepts walk_distance kwarg without error.
2. After init, mission_state == IDLE and ramp_gain == 0.0.
3. step() runs without exception for 10 cycles with walk_distance=None.
"""
import pytest
from shared_state import shared_state, MissionState


@pytest.fixture(scope="module")
def controller():
    from HeartBeat import Siclo1Controller
    ctrl = Siclo1Controller(use_gui=False, walk_distance=None)
    yield ctrl
    ctrl.finalize_telemetry()
    ctrl.shutdown()


def test_controller_accepts_walk_distance_none(controller):
    assert controller is not None


def test_mission_state_starts_idle(controller):
    assert shared_state.mission_state == MissionState.IDLE


def test_ramp_gain_starts_zero(controller):
    assert shared_state.ramp_gain == 0.0


def test_grf_torque_correction_exists(controller):
    # After init + warmup, grf_torque_correction is a dict
    assert isinstance(shared_state.grf_torque_correction, dict)


def test_ten_steps_run_without_exception(controller):
    for _ in range(10):
        controller.step()
```

### Step 5.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_heartbeat_gait_wiring.py -v`
- Expected: FAIL — `Siclo1Controller.__init__() got an unexpected keyword argument 'walk_distance'`

### Step 5.3 — Add imports to `HeartBeat.py`

- [ ] In `HeartBeat.py`, add after `import active_balance` (around line 51):

```python
import grf
import gait_planner
import mission
from mission import MissionController
```

### Step 5.4 — Add WBC constants to `HeartBeat.py`

- [ ] In `HeartBeat.py`, add to the CONSTANTS section (after `LOG_BUFFER_SIZE`):

```python
# WBC joint-space PD gains — converts IK angle targets to additive torques.
# These are tuning parameters, not URDF-derived.
WBC_KP: float = 200.0   # N·m/rad, joint position proportional gain
WBC_KD: float = 15.0    # N·m·s/rad, joint velocity derivative gain

# WBC joint mapping: (IK angle index, URDF joint name, sign)
# sign: +1 for right (axis +X), -1 for left (axis -X)
_WBC_LEFT_JOINTS = [
    (0, 'Left_Hip_Forwards', -1.0),   # hip_pitch  (axis = -X)
    (1, 'Left_Knee',         -1.0),   # knee       (axis = -X)
    (2, 'Left_Ankle',        -1.0),   # ankle      (axis ≈ -Z, neutral = 0)
]
_WBC_RIGHT_JOINTS = [
    (0, 'Right_Hip_Fowards', +1.0),   # hip_pitch  (axis = +X)
    (1, 'Right_Knee',        +1.0),   # knee       (axis = +X)
    (2, 'Right_Ankle',       +1.0),   # ankle      (axis ≈ -Z, neutral = 0)
]
```

### Step 5.5 — Modify `Siclo1Controller.__init__` to accept `walk_distance`

- [ ] In `HeartBeat.py`, change the signature of `Siclo1Controller.__init__`:

Old:
```python
    def __init__(self, use_gui: bool = False, viz_decimation: int = 10):
```

New:
```python
    def __init__(self, use_gui: bool = False, viz_decimation: int = 10,
                 walk_distance: float = None):
```

- [ ] At the end of `Siclo1Controller.__init__`, after `self._warmup(warmup_cycles)`, add:

```python
        # Mission controller — manages gait state machine and ramp_gain.
        self._mission = MissionController(walk_distance=walk_distance)
```

### Step 5.6 — Add `_wbc_step()` to `Siclo1Controller`

- [ ] Add the following method to `Siclo1Controller` (before `step()`):

```python
    def _wbc_step(self) -> None:
        """Joint-space PD controller: IK angles → additive torques in target_torques.

        Reads ik_left_angles and ik_right_angles (URDF-signed rad).
        Computes PD torque for each leg joint and adds to shared_state.target_torques.
        GRF corrections are NOT merged here — handled in apply_control().
        """
        torques = getattr(shared_state, 'target_torques', {})

        jp = shared_state.joint_positions
        jv = shared_state.joint_velocities

        for idx, jname, _ in _WBC_LEFT_JOINTS:
            theta_target = shared_state.ik_left_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            tau = WBC_KP * (theta_target - theta_now) - WBC_KD * omega_now
            torques[jname] = torques.get(jname, 0.0) + _clip_effort(jname, tau)

        for idx, jname, _ in _WBC_RIGHT_JOINTS:
            theta_target = shared_state.ik_right_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            tau = WBC_KP * (theta_target - theta_now) - WBC_KD * omega_now
            torques[jname] = torques.get(jname, 0.0) + _clip_effort(jname, tau)

        shared_state.target_torques = torques
```

### Step 5.7 — Modify `PyBulletInterface.apply_control()` to merge GRF

- [ ] In `HeartBeat.py`, replace the body of `PyBulletInterface.apply_control()`:

Old:
```python
    def apply_control(self) -> None:
        if self.robot_id is None:
            return

        rid = self.robot_id
        pc  = self.pc
        torques = getattr(self.shared_state, 'target_torques', {})

        for jname, raw_torque in torques.items():
            jid = self.joint_ids.get(jname)
            if jid is None:
                continue
            clipped = _clip_effort(jname, raw_torque)
            p.setJointMotorControl2(
                rid, jid,
                controlMode=p.TORQUE_CONTROL,
                force=clipped,
                physicsClientId=pc,
            )
```

New:
```python
    def apply_control(self) -> None:
        if self.robot_id is None:
            return

        rid = self.robot_id
        pc  = self.pc
        torques = getattr(self.shared_state, 'target_torques', {})
        grf_corr = getattr(self.shared_state, 'grf_torque_correction', {})

        for jname, raw_torque in torques.items():
            jid = self.joint_ids.get(jname)
            if jid is None:
                continue
            # Merge GRF additive correction and clip to URDF effort limit
            merged  = raw_torque + grf_corr.get(jname, 0.0)
            clipped = _clip_effort(jname, merged)
            p.setJointMotorControl2(
                rid, jid,
                controlMode=p.TORQUE_CONTROL,
                force=clipped,
                physicsClientId=pc,
            )
```

### Step 5.8 — Update `_warmup()` to include new modules

- [ ] In `HeartBeat.py`, update `_warmup()` body to include the three new modules:

Old:
```python
        for _ in range(cycles):
            self.pybullet.read_sensors()
            self.pybullet.update_link_positions()
            perception.update_perception()
            stability.update_stability(dt=TARGET_DT)
            active_balance.update_active_balance()
            recovery.update_recovery()
            self.pybullet.apply_control()
            sim.interface.step_simulation(self.physics_client)
```

New:
```python
        for _ in range(cycles):
            self.pybullet.read_sensors()
            self.pybullet.update_link_positions()
            perception.update_perception()
            stability.update_stability(dt=TARGET_DT)
            active_balance.update_active_balance()
            grf.update_grf()
            gait_planner.update_gait_planner()
            self._mission.update()
            self._wbc_step()
            recovery.update_recovery()
            self.pybullet.apply_control()
            sim.interface.step_simulation(self.physics_client)
```

### Step 5.9 — Update `step()` to include new modules in the correct order

- [ ] In `HeartBeat.py`, in `Siclo1Controller.step()`, after step 6 (`active_balance.update_active_balance()`), add the three new module calls and the WBC step:

Old (after step 6):
```python
        # 7. Emergency gate
        if shared_state.emergency_stop_triggered:
            return False

        # 8. Recovery
        recovery.update_recovery()
```

New:
```python
        # 7. GRF — virtual spring-damper torque corrections
        grf.update_grf()

        # 8. Gait Planner — swing arc + IK angle targets
        gait_planner.update_gait_planner()

        # 9. Mission — state machine, ramp_gain, step counting
        self._mission.update()

        # 10. WBC — IK angles → additive joint PD torques
        self._wbc_step()

        # 11. Emergency gate
        if shared_state.emergency_stop_triggered:
            return False

        # 12. Recovery
        recovery.update_recovery()
```

- [ ] Update the existing step comments 7–15 to renumber to 13–21 accordingly (search for `# 9.` through `# 15.` and increment by 6 in each label).

### Step 5.10 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_heartbeat_gait_wiring.py -v`
- Expected: 5 tests PASS

### Step 5.11 — Run full test suite to check no regressions

- [ ] Run: `python3 -m pytest test_gait_shared_state.py test_grf.py test_gait_planner.py test_mission.py test_heartbeat_gait_wiring.py -v`
- Expected: all PASS

### Step 5.12 — Commit

```bash
git add HeartBeat.py test_heartbeat_gait_wiring.py
git commit -m "feat: wire GRF/GaitPlanner/Mission into HeartBeat with WBC PD step"
```

---

## Task 6: Wire `main.py` — `--walk` Argument

**Files:**
- Modify: `main.py`

### Step 6.1 — Write the failing test

- [ ] Create `test_main_walk_arg.py`:

```python
"""Tests for --walk CLI argument in main.py — no simulation required."""
import pytest


def test_walk_arg_accepted():
    """--walk 2.0 is accepted without error."""
    import main
    args = main._make_parser().parse_args(['--walk', '2.0'])
    assert args.walk == 2.0


def test_walk_arg_default_is_none():
    """Omitting --walk yields walk=None."""
    import main
    args = main._make_parser().parse_args([])
    assert args.walk is None


def test_walk_arg_combined_with_gui():
    """--gui --walk 1.5 both accepted."""
    import main
    args = main._make_parser().parse_args(['--gui', '--walk', '1.5'])
    assert args.gui is True
    assert args.walk == 1.5


def test_walk_arg_combined_with_duration():
    """--duration 500 --walk 1.0 both accepted."""
    import main
    args = main._make_parser().parse_args(['--duration', '500', '--walk', '1.0'])
    assert args.duration == 500
    assert args.walk == 1.0
```

### Step 6.2 — Run test to verify it fails

- [ ] Run: `python3 -m pytest test_main_walk_arg.py -v`
- Expected: FAIL — `error: unrecognized arguments: --walk 2.0`

### Step 6.3 — Add `--walk` to `main.py`

- [ ] In `main.py`, in `_make_parser()`, add after the `--hold` argument:

```python
    parser.add_argument("--walk", type=float, default=None, metavar="METRES",
                        help="Walk forward D metres then stop (e.g. --walk 2.0)")
```

- [ ] In `main.py`, in `main()`, pass `walk_distance` to the controller. Change:

Old:
```python
    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation)
```

New:
```python
    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation,
                                  walk_distance=args.walk)
```

- [ ] Update the docstring at the top of `main.py` to include the new usage:

Old first line of docstring:
```python
"""
Siclo1 bipedal robot simulation — CLI entry point.

Usage:
    python3 main.py                                # headless, 1000 cycles
    python3 main.py --gui                          # GUI at 10 Hz, 1000 cycles
    python3 main.py --gui --viz-hz 33              # GUI at 33 Hz
    python3 main.py --gui --duration 2000 --hold   # 2000 cycles, inspect final pose
"""
```

New:
```python
"""
Siclo1 bipedal robot simulation — CLI entry point.

Usage:
    python3 main.py                                # headless, 1000 cycles
    python3 main.py --gui                          # GUI at 10 Hz, 1000 cycles
    python3 main.py --gui --viz-hz 33              # GUI at 33 Hz
    python3 main.py --gui --duration 2000 --hold   # 2000 cycles, inspect final pose
    python3 main.py --walk 2.0                     # walk 2.0 m headless
    python3 main.py --gui --walk 2.0               # walk 2.0 m with GUI
"""
```

### Step 6.4 — Run test to verify it passes

- [ ] Run: `python3 -m pytest test_main_walk_arg.py -v`
- Expected: 4 tests PASS

### Step 6.5 — Run full test suite

- [ ] Run: `python3 -m pytest test_gait_shared_state.py test_grf.py test_gait_planner.py test_mission.py test_heartbeat_gait_wiring.py test_main_walk_arg.py -v`
- Expected: all PASS

### Step 6.6 — Commit

```bash
git add main.py test_main_walk_arg.py
git commit -m "feat: add --walk CLI argument; pass walk_distance to Siclo1Controller"
```

---

## Self-Review

**Spec coverage check:**

| Spec requirement | Covered by task |
|---|---|
| `grf.py` virtual spring-damper model with K_SPRING=1589, B_DAMPER=94, Z_REST=0.75 | Task 2 (`grf.py` constants) |
| Jacobian transpose mapping Fz → hip+knee torques | Task 2 (`_jacobian_torques`) |
| URDF axis sign conventions for left/right | Task 2 (`urdf_sign` parameter) |
| `ramp_gain` scales GRF output | Task 2 (`ramp_gain * tau`) |
| `grf_torque_correction` field in shared_state | Task 1 |
| `gait_planner.py` CP-adjusted foot target | Task 3 (`_compute_x_target`) |
| Parabolic swing arc with SWING_HEIGHT=0.04, SWING_DURATION=0.40 | Task 3 (`_swing_z`) |
| `kinematics.solve_ik()` call (no changes to kinematics.py) | Task 3 |
| `active_swing_side`, `step_count` fields | Task 1 |
| `swing_foot_x_stance` (not in spec but required for persistence) | Task 1 |
| `mission.py` IDLE→RAMP→WALK→DECEL→STOP machine | Task 4 |
| RAMP: 1/50 per cycle; STOP: 1/20 per cycle | Task 4 (`RAMP_RATE`, `STOP_RATE`) |
| DECEL: STEP_LENGTH halved, K_SPRING +20% | Task 3 (`_compute_x_target`), Task 2 (`DECEL_SPRING_BOOST`) |
| `mission_state`, `steps_remaining`, `ramp_gain` fields | Task 1 |
| `MissionState` enum in `shared_state.py` | Task 1 |
| HeartBeat call order: ActiveBalance→GRF→GaitPlanner→Mission→WBC | Task 5 (`step()`) |
| `--walk` CLI argument | Task 6 |
| Passes `walk_distance` to `Siclo1Controller` → `MissionController` | Task 5 + 6 |
| GRF merge in `apply_control()` | Task 5 |
| WBC PD step for IK angle → torque conversion | Task 5 (`_wbc_step`) |

**Placeholder scan:** No TBDs, no "similar to task N", all code blocks complete.

**Type consistency check:**
- `kinematics.solve_ik(foot_xyz_rel, side)` returns `tuple` of 3 floats — matches `ik_left_angles: tuple` in `shared_state.__init__`. ✓
- `grf_torque_correction: Dict[str, float]` — all write sites produce `{str: float}`. ✓
- `MissionState.IDLE` referenced in `grf.py`, `gait_planner.py` — both import `MissionState` from `shared_state`. ✓
- `STEP_LENGTH = 0.12` in both `gait_planner.py` and `mission.py` — identical. ✓
- `capture_point[0]` — `active_balance.py` writes `np.array([x_cp, y_cp])`; index 0 is x. ✓
- `_WBC_LEFT_JOINTS` / `_WBC_RIGHT_JOINTS` joint names match `URDF_JOINT_NAMES` registry (note `Right_Hip_Fowards` typo preserved). ✓

---

**Plan complete and saved to `docs/superpowers/plans/2026-04-07-dynamic-gait-controller.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — Fresh subagent per task, review between tasks

**2. Inline Execution** — Execute tasks in this session using executing-plans

**Which approach?**
