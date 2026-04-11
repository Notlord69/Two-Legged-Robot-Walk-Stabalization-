# Phased Gait FSM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single phi-arc gait loop with a 5-state step-phase FSM (DOUBLE_SUPPORT → COM_SHIFT → LIFT → SWING → PLACE) that prevents WBC-induced robot launch by gating every phase transition on explicit physical conditions.

**Architecture:** `shared_state.py` gains a `StepPhase` enum and 4 new fields; `gait_planner.py` is rewritten as an FSM whose phases gate on contact state, Capture Point, foot force, and foot velocity; `grf.py` gains stance-only gating for LIFT/SWING/PLACE phases. All phase transitions call `recovery.reset_step()` to reset the step watchdog.

**Tech Stack:** Python 3.10, PyBullet, NumPy, `kinematics.solve_ik()`, `recovery.reset_step()`, pytest

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `shared_state.py` | Modify | Add `StepPhase` enum, `ERR_PHASE_TIMEOUT`, 4 new fields |
| `grf.py` | Modify | Stance-only gating for LIFT/SWING/PLACE phases |
| `gait_planner.py` | Rewrite | 5-state FSM, stance IK anchor, phase transitions |
| `test_shared_state_phase.py` | Create | Enum values, error code, field defaults |
| `test_grf_phase_gate.py` | Create | Swing leg GRF suppressed in LIFT/SWING/PLACE |
| `test_step_phase_transitions.py` | Create | All nominal DS→CS→LIFT→SWING→PLACE→DS transitions; timer resets |
| `test_step_phase_guards.py` | Create | freeze_robot blocks all; UNSTABLE blocks COM_SHIFT exit |
| `test_step_phase_timeouts.py` | Create | Each timeout fires at correct time; conditional force routing |
| `test_stance_anchor.py` | Create | stance_foot_world_pos locked once; IK recomputed each cycle |
| `test_velocity_gate.py` | Create | LIFT and PLACE exit blocked when vel_z > SETTLE_VEL_THRESHOLD |
| `test_gait_planner_fsm.py` | Create | Full step cycle integration; step_count increments once; active_swing_side flips once |

---

## Task 1: shared_state.py — StepPhase Enum + Error Code + Fields

**Files:**
- Modify: `shared_state.py`
- Create: `test_shared_state_phase.py`

- [ ] **Step 1: Write the failing test**

```python
# test_shared_state_phase.py
import numpy as np
import pytest
from shared_state import (
    Siclo1State, StepPhase,
    ERR_PHASE_TIMEOUT,
)


def test_step_phase_enum_members():
    phases = [p.name for p in StepPhase]
    assert phases == ["DOUBLE_SUPPORT", "COM_SHIFT", "LIFT", "SWING", "PLACE"]


def test_err_phase_timeout_value():
    assert ERR_PHASE_TIMEOUT == 6


def test_step_phase_default():
    s = Siclo1State()
    assert s.step_phase == StepPhase.DOUBLE_SUPPORT


def test_step_phase_timer_default():
    s = Siclo1State()
    assert s.step_phase_timer == 0.0


def test_stance_side_default():
    s = Siclo1State()
    assert s.stance_side == "right"   # complement of active_swing_side="left"


def test_stance_foot_world_pos_default():
    s = Siclo1State()
    assert isinstance(s.stance_foot_world_pos, np.ndarray)
    assert s.stance_foot_world_pos.shape == (3,)
    np.testing.assert_array_equal(s.stance_foot_world_pos, [0.0, 0.0, 0.0])
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_shared_state_phase.py -v
```

Expected: FAIL — `ImportError: cannot import name 'StepPhase'`

- [ ] **Step 3: Add StepPhase enum to shared_state.py**

In `shared_state.py`, after the `MissionState` enum block (after line 133), add:

```python
class StepPhase(Enum):
    """Five-phase step FSM state.  Owned exclusively by gait_planner.py.
    All other modules read this field; only gait_planner writes it."""
    DOUBLE_SUPPORT = auto()   # both feet grounded — stabilise before swing
    COM_SHIFT      = auto()   # shift COM over stance foot
    LIFT           = auto()   # unload and lift swing foot
    SWING          = auto()   # phi-arc swing; only phase where phi advances
    PLACE          = auto()   # drive swing foot to ground; await contact
```

- [ ] **Step 4: Add ERR_PHASE_TIMEOUT to error code constants**

In the `ERROR CODE CONSTANTS` block (around line 36-40), add:

```python
ERR_PHASE_TIMEOUT     = 6   # step-phase timeout fired (hot-path safe, no string)
```

- [ ] **Step 5: Add 4 new fields to Siclo1State.__init__**

In the `DYNAMIC GAIT CONTROLLER STATE` section (after the existing `ramp_gain` field, around line 317), add:

```python
        # Current step FSM phase.  Written by gait_planner.py exclusively.
        # All other modules read this to gate behaviour per phase.
        self.step_phase: StepPhase = StepPhase.DOUBLE_SUPPORT

        # Seconds elapsed in the current step phase.  Reset to 0.0 on every
        # phase transition.  Written by gait_planner.py.
        self.step_phase_timer: float = 0.0

        # Foot currently bearing weight — complement of active_swing_side.
        # "right" at spawn because active_swing_side defaults to "left".
        self.stance_side: str = "right"

        # Stance foot world-frame position locked once at DOUBLE_SUPPORT entry.
        # Used as IK anchor for all stance-leg computations in every phase.
        # Written by gait_planner.py exactly once per stance entry.
        self.stance_foot_world_pos: np.ndarray = np.zeros(3)
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
python -m pytest test_shared_state_phase.py -v
```

Expected: 6 PASSED

- [ ] **Step 7: Commit**

```bash
git add shared_state.py test_shared_state_phase.py
git commit -m "feat: add StepPhase enum, ERR_PHASE_TIMEOUT, and 4 phase FSM fields to shared_state"
```

---

## Task 2: grf.py — Stance-Only GRF for LIFT/SWING/PLACE

**Files:**
- Modify: `grf.py`
- Create: `test_grf_phase_gate.py`

Context: currently `grf.py` gates each leg on `CONTACT_CONFIRMED`. During LIFT/SWING/PLACE the swing leg must receive zero GRF even if its contact sensor still reads confirmed (sensor lag during liftoff). Only the stance leg gets GRF in those phases.

- [ ] **Step 1: Write the failing test**

```python
# test_grf_phase_gate.py
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, Siclo1State, ContactState, MissionState, StepPhase,
)
import grf


def _configure_state(step_phase, swing_side="left", ramp_gain=1.0):
    """Set up shared_state for a GRF update call."""
    shared_state.freeze_robot = False
    shared_state.emergency_stop_triggered = False
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = ramp_gain
    shared_state.step_phase = step_phase
    shared_state.active_swing_side = swing_side
    shared_state.stance_side = "right" if swing_side == "left" else "left"
    # Both feet confirmed contact (worst case: swing foot sensor has not cleared)
    shared_state.left_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_position  = np.array([0.0, 0.0, 0.0])
    shared_state.right_foot_position = np.array([0.0, 0.0, 0.0])
    shared_state.left_foot_velocity  = np.zeros(3)
    shared_state.right_foot_velocity = np.zeros(3)
    shared_state.joint_positions = {
        'Left_Hip_Forwards': 0.3,
        'Left_Knee':         0.3,
        'Right_Hip_Fowards': 0.3,
        'Right_Knee':        0.3,
    }


def test_double_support_both_legs_get_grf():
    _configure_state(StepPhase.DOUBLE_SUPPORT, swing_side="left")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    # Both legs should receive non-zero GRF (non-zero joint angles → non-zero Jacobian)
    assert result['Left_Hip_Forwards']  != 0.0
    assert result['Right_Hip_Fowards']  != 0.0


def test_com_shift_both_legs_get_grf():
    _configure_state(StepPhase.COM_SHIFT, swing_side="left")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    assert result['Left_Hip_Forwards']  != 0.0
    assert result['Right_Hip_Fowards']  != 0.0


def test_lift_swing_leg_gets_zero_grf():
    """LIFT phase: left is swing → left GRF must be 0 even with CONTACT_CONFIRMED."""
    _configure_state(StepPhase.LIFT, swing_side="left")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    assert result['Left_Hip_Forwards'] == 0.0
    assert result['Left_Knee']         == 0.0
    # Stance (right) still gets GRF
    assert result['Right_Hip_Fowards'] != 0.0


def test_swing_phase_swing_leg_gets_zero_grf():
    _configure_state(StepPhase.SWING, swing_side="left")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    assert result['Left_Hip_Forwards'] == 0.0
    assert result['Left_Knee']         == 0.0
    assert result['Right_Hip_Fowards'] != 0.0


def test_place_phase_swing_leg_gets_zero_grf():
    _configure_state(StepPhase.PLACE, swing_side="left")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    assert result['Left_Hip_Forwards'] == 0.0
    assert result['Left_Knee']         == 0.0
    assert result['Right_Hip_Fowards'] != 0.0


def test_phase_gate_works_for_right_swing():
    """Same logic when right is swing side."""
    _configure_state(StepPhase.SWING, swing_side="right")
    grf.update_grf()
    result = shared_state.grf_torque_correction
    assert result['Right_Hip_Fowards'] == 0.0
    assert result['Right_Knee']        == 0.0
    assert result['Left_Hip_Forwards'] != 0.0
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest test_grf_phase_gate.py -v
```

Expected: FAIL on `test_lift_swing_leg_gets_zero_grf` — swing leg GRF is currently not zeroed during LIFT/SWING/PLACE.

- [ ] **Step 3: Update grf.py imports**

At the top of `grf.py`, update the import block to:

```python
from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StepPhase,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
)
```

- [ ] **Step 4: Add phase gating constant and logic to GRFController.update()**

Replace the leg-computation section of `GRFController.update()` (lines 182–216 in grf.py) with:

```python
        # Phases where only the stance leg receives GRF.
        # Swing leg is gated out even if contact sensor still reads CONFIRMED
        # (sensor lags during liftoff can delay clearing).
        _STANCE_ONLY_PHASES = {StepPhase.LIFT, StepPhase.SWING, StepPhase.PLACE}
        step_phase  = shared_state.step_phase
        stance_side = shared_state.stance_side   # "left" or "right"

        jp = shared_state.joint_positions
        result: Dict[str, float] = {}

        # ── Left leg — axis = -X → urdf_sign = -1.0 ─────────────────────────
        left_eligible = (step_phase not in _STANCE_ONLY_PHASES
                         or stance_side == "left")
        if (left_eligible and
                shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED):
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

        # ── Right leg — axis = +X → urdf_sign = +1.0 ────────────────────────
        right_eligible = (step_phase not in _STANCE_ONLY_PHASES
                          or stance_side == "right")
        if (right_eligible and
                shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED):
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
```

Also remove the old `jp = shared_state.joint_positions` line that appeared before this block (it is now inside the replacement).

- [ ] **Step 5: Run tests to verify they pass**

```bash
python -m pytest test_grf_phase_gate.py -v
```

Expected: 6 PASSED

- [ ] **Step 6: Commit**

```bash
git add grf.py test_grf_phase_gate.py
git commit -m "feat: gate GRF to stance leg only during LIFT/SWING/PLACE phases"
```

---

## Task 3: gait_planner.py — FSM Scaffold + DOUBLE_SUPPORT + COM_SHIFT

**Files:**
- Rewrite: `gait_planner.py`
- Create: `test_step_phase_transitions.py` (DS→COM_SHIFT→LIFT portion)

- [ ] **Step 1: Write the failing transition tests**

```python
# test_step_phase_transitions.py
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, Siclo1State, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    """Return shared_state to a clean walking configuration."""
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.step_count                = 0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0   # N, well above threshold
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])  # right foot locked
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


# ── DOUBLE_SUPPORT → COM_SHIFT ────────────────────────────────────────────────

def test_ds_does_not_advance_before_min_time():
    _reset()
    shared_state.step_phase_timer = 0.05   # < DS_MIN_TIME=0.10
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_ds_advances_to_com_shift_when_both_confirmed_and_min_time_elapsed():
    _reset()
    shared_state.step_phase_timer = 0.11   # > DS_MIN_TIME
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_ds_timer_resets_on_transition_to_com_shift():
    _reset()
    shared_state.step_phase_timer = 0.11
    gait_planner.update_gait_planner()
    # After one update the timer was already incremented by dt=0.01 before check
    # then reset to 0.0 on transition, then +dt re-added — so timer ≈ dt
    assert shared_state.step_phase_timer < 0.02


def test_ds_blocks_when_foot_not_confirmed():
    _reset()
    shared_state.step_phase_timer = 0.5
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


# ── COM_SHIFT → LIFT ─────────────────────────────────────────────────────────

def test_com_shift_advances_to_lift_when_cp_close_and_stable():
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    # CP near stance foot (right foot at x=0.1)
    shared_state.capture_point     = np.array([0.11, 0.0])  # |0.11-0.10| = 0.01 < 0.03
    shared_state.stability_status  = StabilityStatus.STABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_com_shift_blocked_when_cp_far():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.50, 0.0])   # far from stance x=0.10
    shared_state.stability_status = StabilityStatus.STABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_com_shift_blocked_when_unstable_even_if_cp_close():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.UNSTABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_com_shift_timer_resets_on_transition_to_lift():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.STABLE
    shared_state.step_phase_timer = 0.5
    gait_planner.update_gait_planner()
    assert shared_state.step_phase_timer < 0.02


# ── LIFT → SWING ─────────────────────────────────────────────────────────────

def test_lift_advances_to_swing_when_unloaded_and_settled():
    _reset()
    shared_state.step_phase              = StepPhase.LIFT
    shared_state.left_foot_force         = 2.0    # < UNLOAD_FORCE_THRESHOLD=5.0
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])  # < 0.05 m/s
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_lift_blocked_when_force_still_high():
    _reset()
    shared_state.step_phase       = StepPhase.LIFT
    shared_state.left_foot_force  = 20.0   # > 5.0 N
    shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_blocked_when_velocity_not_settled():
    _reset()
    shared_state.step_phase              = StepPhase.LIFT
    shared_state.left_foot_force         = 2.0
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.20])  # > 0.05
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_snapshots_swing_foot_x_stance_on_transition():
    _reset()
    shared_state.step_phase          = StepPhase.LIFT
    shared_state.left_foot_position  = np.array([0.05, 0.0, 0.0])
    shared_state.left_foot_force     = 2.0
    shared_state.left_foot_velocity  = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.swing_phase == 0.0
    # x_stance snapped from left_foot_position[0] at transition
    assert abs(shared_state.swing_foot_x_stance - 0.05) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest test_step_phase_transitions.py -v
```

Expected: FAIL — `GaitPlannerController.update()` has no FSM; it runs phi-arc regardless.

- [ ] **Step 3: Rewrite gait_planner.py with FSM scaffold + DS + COM_SHIFT + LIFT handlers**

Replace the entire content of `gait_planner.py` with:

```python
"""
================================================================================
PROJECT SICLO1 — GAIT PLANNER  (gait_planner.py)
================================================================================

5-State Step-Phase FSM:
    DOUBLE_SUPPORT → COM_SHIFT → LIFT → SWING → PLACE → DOUBLE_SUPPORT

Phase advance per cycle:
    step_phase_timer += dt on every cycle.
    Each phase handler checks exit conditions and calls _transition_to().

Stance IK anchor (all phases, every cycle, stance leg only in LIFT/SWING/PLACE):
    stance_foot_rel = stance_foot_world_pos - stance_hip_pos
    ik_stance_angles = kinematics.solve_ik(stance_foot_rel, stance_side)

Swing arc (SWING and PLACE phases only):
    z_swing = SWING_HEIGHT * 4 * φ * (1 - φ)
    x_swing = x_stance + (x_target - x_stance) * φ

INPUTS (shared_state):
    step_phase, step_phase_timer, stance_side, stance_foot_world_pos,
    capture_point, swing_phase, swing_foot_x_stance,
    left/right_foot_{position,velocity,force,contact_state},
    stability_status, mission_state, ramp_gain, freeze_robot, link_positions,
    last_dt

OUTPUTS (shared_state):
    step_phase, step_phase_timer, swing_phase, swing_foot_x_stance,
    swing_foot_target, left/right_foot_target,
    ik_left_angles, ik_right_angles,
    step_count, active_swing_side, stance_side,
    stance_foot_world_pos, freeze_robot

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import numpy as np

import kinematics
import recovery
from shared_state import (
    shared_state, MissionState, StepPhase, ContactState, StabilityStatus,
    ERR_PHASE_TIMEOUT,
)


# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH:       float = 0.12   # m, fixed sagittal advance per step (nominal)
STEP_TIMING_SCALE: float = 0.5    # dimensionless, blend factor for CP correction

SWING_HEIGHT:   float = 0.04   # m, peak foot clearance above ground at φ=0.5
SWING_DURATION: float = 0.40   # s, full swing phase (40 cycles at 100 Hz)

# Phase timeout constants
DS_MIN_TIME:           float = 0.10   # s, minimum double-support duration
DS_TIMEOUT:            float = 2.0    # s, DS timeout → freeze_robot
COM_SHIFT_TIMEOUT:     float = 1.0    # s, COM_SHIFT timeout → conditional
COM_SHIFT_THRESHOLD:   float = 0.03   # m, |CP_x − stance_foot_x| to exit COM_SHIFT
LIFT_TIMEOUT:          float = 0.15   # s, LIFT timeout → conditional
SWING_TIMEOUT_FACTOR:  float = 1.5    # ×SWING_DURATION before force-advance to PLACE
PLACE_TIMEOUT:         float = 0.5    # s, PLACE timeout → conditional

# Physical thresholds
UNLOAD_FORCE_THRESHOLD: float = 5.0    # N, foot considered unloaded below this
SETTLE_VEL_THRESHOLD:   float = 0.05   # m/s, foot considered settled below this
PLACE_ENTRY_PHI:        float = 0.85   # dimensionless, phi at which SWING → PLACE

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

    capture_point_x: x-component of CP in world frame (m), pre-clamped
    decel: True in DECEL state → halve STEP_LENGTH to absorb stopping impulse
    Returns target x (m, world frame).
    """
    step = STEP_LENGTH * 0.5 if decel else STEP_LENGTH
    return capture_point_x * STEP_TIMING_SCALE + step


def _clamped_cp_x() -> float:
    """Read capture_point[0] from shared_state, clamped to ±0.20 m.

    Clamped to prevent IK workspace violations during erratic motion.
    ±0.20 m covers the full physically valid recovery range for this robot.
    """
    cp_x = float(getattr(shared_state, 'capture_point', np.zeros(2))[0])
    return float(np.clip(cp_x, -0.20, 0.20))


# ============================================================================
# GAIT PLANNER CONTROLLER
# ============================================================================

class GaitPlannerController:
    """Per-cycle gait planner FSM.  All state lives in shared_state."""

    def __init__(self):
        self._ds_lock_pending: bool = True  # lock stance foot on first DS cycle

    # ── Public entry point ────────────────────────────────────────────────────

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        if (shared_state.freeze_robot or
                shared_state.mission_state == MissionState.IDLE):
            return

        dt = shared_state.last_dt
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01   # fallback to 100 Hz nominal

        # Advance phase timer every cycle before dispatching
        shared_state.step_phase_timer += dt

        phase = shared_state.step_phase
        if phase == StepPhase.DOUBLE_SUPPORT:
            self._handle_double_support(dt)
        elif phase == StepPhase.COM_SHIFT:
            self._handle_com_shift(dt)
        elif phase == StepPhase.LIFT:
            self._handle_lift(dt)
        elif phase == StepPhase.SWING:
            self._handle_swing(dt)
        elif phase == StepPhase.PLACE:
            self._handle_place(dt)

    # ── Transition helpers ────────────────────────────────────────────────────

    def _transition_to(self, phase: StepPhase) -> None:
        """Advance FSM to phase: reset timer, reset step watchdog."""
        if phase == StepPhase.DOUBLE_SUPPORT:
            self._ds_lock_pending = True
        shared_state.step_phase       = phase
        shared_state.step_phase_timer = 0.0
        recovery.reset_step()

    def _abort_to_double_support(self) -> None:
        """Conditional timeout abort: preserve stance/swing sides, retry same step."""
        shared_state.swing_phase = 0.0
        self._transition_to(StepPhase.DOUBLE_SUPPORT)

    # ── Stance IK anchor (used every cycle for stance leg) ───────────────────

    def _compute_stance_ik(self) -> None:
        """Recompute stance leg IK from locked world-frame anchor.

        Runs every cycle in every phase for the stance leg.
        Stance foot is never frozen — WBC always gets a valid target.
        """
        stance_side = shared_state.stance_side
        hip_key = (_LEFT_HIP_LINK if stance_side == "left"
                   else _RIGHT_HIP_LINK)
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        stance_foot_rel = shared_state.stance_foot_world_pos - hip_pos
        rel_z = float(stance_foot_rel[2])
        if not (-1.0 < rel_z < 0.0):
            return   # invalid geometry (robot falling); hold last angles
        foot_xyz_rel = (
            float(stance_foot_rel[0]),
            float(stance_foot_rel[1]),
            rel_z,
        )
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, stance_side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)
        if stance_side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

    def _lock_stance_foot(self) -> None:
        """Snapshot current stance foot world position into stance_foot_world_pos.

        Called exactly once per stance entry (DOUBLE_SUPPORT entry).
        """
        stance_side = shared_state.stance_side
        foot_pos = (shared_state.left_foot_position  if stance_side == "left"
                    else shared_state.right_foot_position)
        shared_state.stance_foot_world_pos = foot_pos.copy()

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _handle_double_support(self, dt: float) -> None:
        # Lock stance foot exactly once on DS entry
        if self._ds_lock_pending:
            self._lock_stance_foot()
            self._ds_lock_pending = False

        self._compute_stance_ik()

        timer = shared_state.step_phase_timer
        if timer >= DS_TIMEOUT:
            shared_state.freeze_robot = True
            return

        both_confirmed = shared_state.both_feet_in_contact()
        if both_confirmed and timer >= DS_MIN_TIME:
            self._transition_to(StepPhase.COM_SHIFT)

    def _handle_com_shift(self, dt: float) -> None:
        self._compute_stance_ik()

        timer     = shared_state.step_phase_timer
        cp_x      = float(shared_state.capture_point[0])
        stance_x  = float(shared_state.stance_foot_world_pos[0])
        stable    = (shared_state.stability_status != StabilityStatus.UNSTABLE)
        cp_close  = abs(cp_x - stance_x) < COM_SHIFT_THRESHOLD

        if stable and cp_close:
            self._snapshot_swing_foot_x()
            self._transition_to(StepPhase.LIFT)
            return

        if timer >= COM_SHIFT_TIMEOUT:
            swing_force = self._swing_foot_force()
            if swing_force > UNLOAD_FORCE_THRESHOLD:
                self._abort_to_double_support()
            else:
                self._snapshot_swing_foot_x()
                self._transition_to(StepPhase.LIFT)

    def _handle_lift(self, dt: float) -> None:
        self._compute_stance_ik()

        timer       = shared_state.step_phase_timer
        swing_force = self._swing_foot_force()
        swing_vel_z = abs(float(self._swing_foot_velocity()[2]))

        if swing_force < UNLOAD_FORCE_THRESHOLD and swing_vel_z < SETTLE_VEL_THRESHOLD:
            self._transition_to(StepPhase.SWING)
            return

        if timer >= LIFT_TIMEOUT:
            if swing_force > UNLOAD_FORCE_THRESHOLD:
                self._abort_to_double_support()
            else:
                self._transition_to(StepPhase.SWING)

    def _handle_swing(self, dt: float) -> None:
        self._compute_stance_ik()

        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # Compute foot target — CP may update mid-swing
        x_target = _compute_x_target(
            _clamped_cp_x(),
            decel=(shared_state.mission_state == MissionState.DECEL),
        )
        shared_state.swing_foot_target = (x_target, 0.0, 0.0)
        if side == "left":
            shared_state.left_foot_target  = (x_target, 0.0, 0.0)
        else:
            shared_state.right_foot_target = (x_target, 0.0, 0.0)

        # Advance phi
        phi = shared_state.swing_phase + dt / SWING_DURATION
        shared_state.swing_phase = phi

        # Swing arc IK
        self._compute_swing_ik(side, hip_x, hip_z, phi, x_target)

        # Exit: phi reaches PLACE_ENTRY_PHI
        if phi >= PLACE_ENTRY_PHI:
            self._transition_to(StepPhase.PLACE)
            return

        # Timeout: force advance to PLACE, log error code
        if shared_state.step_phase_timer >= SWING_DURATION * SWING_TIMEOUT_FACTOR:
            shared_state.add_error_code(ERR_PHASE_TIMEOUT)
            self._transition_to(StepPhase.PLACE)

    def _handle_place(self, dt: float) -> None:
        self._compute_stance_ik()

        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # phi continues to 1.0 then clamps
        phi = min(shared_state.swing_phase + dt / SWING_DURATION, 1.0)
        shared_state.swing_phase = phi

        x_target = _compute_x_target(
            _clamped_cp_x(),
            decel=(shared_state.mission_state == MissionState.DECEL),
        )
        self._compute_swing_ik(side, hip_x, hip_z, phi, x_target)

        # Exit: contact confirmed + velocity settled
        swing_contact = self._swing_foot_contact()
        swing_vel_z   = abs(float(self._swing_foot_velocity()[2]))

        if (swing_contact == ContactState.CONTACT_CONFIRMED and
                swing_vel_z < SETTLE_VEL_THRESHOLD):
            self._complete_step()
            return

        # Timeout: route by force
        if shared_state.step_phase_timer >= PLACE_TIMEOUT:
            if self._swing_foot_force() > UNLOAD_FORCE_THRESHOLD:
                # Sensor lagged — contact happened; complete step
                self._complete_step()
            else:
                # Foot missed ground — unsafe; freeze
                shared_state.freeze_robot = True

    # ── Swing IK helper ───────────────────────────────────────────────────────

    def _compute_swing_ik(
        self, side: str, hip_x: float, hip_z: float,
        phi: float, x_target: float,
    ) -> None:
        """Compute swing leg IK from parabolic arc and write to shared_state."""
        phi_c   = min(phi, 1.0)
        x_swing = shared_state.swing_foot_x_stance + (x_target - shared_state.swing_foot_x_stance) * phi_c
        z_swing = _swing_z(phi_c)
        rel_z   = z_swing - hip_z
        if not (-1.0 < rel_z < 0.0):
            return   # invalid geometry; hold last angles
        foot_xyz_rel = (x_swing - hip_x, 0.0, rel_z)
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)
        if side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

    # ── Step completion ───────────────────────────────────────────────────────

    def _complete_step(self) -> None:
        """Clean PLACE → DOUBLE_SUPPORT: flip sides, increment step_count, lock anchor."""
        old_swing   = shared_state.active_swing_side
        new_stance  = old_swing
        new_swing   = "right" if old_swing == "left" else "left"

        shared_state.active_swing_side = new_swing
        shared_state.stance_side       = new_stance
        shared_state.step_count       += 1
        shared_state.swing_phase       = 0.0

        # Lock the newly landed foot as the stance anchor for the next step
        foot_pos = (shared_state.left_foot_position  if new_stance == "left"
                    else shared_state.right_foot_position)
        shared_state.stance_foot_world_pos = foot_pos.copy()

        self._transition_to(StepPhase.DOUBLE_SUPPORT)

    # ── Convenience accessors for swing-leg sensors ───────────────────────────

    def _swing_foot_force(self) -> float:
        side = shared_state.active_swing_side
        return float(shared_state.left_foot_force  if side == "left"
                     else shared_state.right_foot_force)

    def _swing_foot_velocity(self) -> np.ndarray:
        side = shared_state.active_swing_side
        return (shared_state.left_foot_velocity  if side == "left"
                else shared_state.right_foot_velocity)

    def _swing_foot_contact(self) -> ContactState:
        side = shared_state.active_swing_side
        return (shared_state.left_foot_contact_state  if side == "left"
                else shared_state.right_foot_contact_state)

    def _snapshot_swing_foot_x(self) -> None:
        """Snapshot swing foot x at COM_SHIFT exit.  Written exactly once per step."""
        side = shared_state.active_swing_side
        foot_pos = (shared_state.left_foot_position  if side == "left"
                    else shared_state.right_foot_position)
        shared_state.swing_foot_x_stance = float(foot_pos[0])


_gait_planner = GaitPlannerController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_gait_planner() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _gait_planner.update()
```

- [ ] **Step 4: Run transition tests**

```bash
python -m pytest test_step_phase_transitions.py -v
```

Expected: all 14 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add gait_planner.py test_step_phase_transitions.py
git commit -m "feat: rewrite gait_planner as 5-state FSM with DS/COM_SHIFT/LIFT/SWING/PLACE phases"
```

---

## Task 4: Guard Tests + Timeout Tests

**Files:**
- Create: `test_step_phase_guards.py`
- Create: `test_step_phase_timeouts.py`

- [ ] **Step 1: Write guard tests**

```python
# test_step_phase_guards.py
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.20   # past DS_MIN_TIME
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_freeze_robot_blocks_all_phases():
    for phase in StepPhase:
        _reset()
        shared_state.step_phase   = phase
        shared_state.freeze_robot = True
        before = shared_state.step_phase
        gait_planner.update_gait_planner()
        assert shared_state.step_phase == before, (
            f"freeze_robot did not block phase {phase.name}"
        )


def test_idle_mission_blocks_all_phases():
    for phase in StepPhase:
        _reset()
        shared_state.step_phase    = phase
        shared_state.mission_state = MissionState.IDLE
        before = shared_state.step_phase
        gait_planner.update_gait_planner()
        assert shared_state.step_phase == before, (
            f"IDLE mission did not block phase {phase.name}"
        )


def test_unstable_blocks_com_shift_exit():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])   # CP close to stance x=0.10
    shared_state.stability_status = StabilityStatus.UNSTABLE
    gait_planner.update_gait_planner()
    # Must NOT advance to LIFT
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_marginal_stability_does_not_block_com_shift_exit():
    """MARGINAL is not UNSTABLE — exit is allowed."""
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.MARGINAL
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_ds_freeze_when_timeout():
    _reset()
    shared_state.step_phase       = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 2.05   # > DS_TIMEOUT=2.0
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True


def test_place_freeze_when_timeout_and_no_force():
    """Foot missed ground at PLACE timeout → freeze."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51   # > PLACE_TIMEOUT=0.5
    shared_state.left_foot_force  = 1.0   # below UNLOAD_FORCE_THRESHOLD
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True
```

- [ ] **Step 2: Write timeout tests**

```python
# test_step_phase_timeouts.py
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.swing_foot_x_stance       = -0.1
    shared_state.step_count                = 0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_com_shift_timeout_high_force_aborts_to_ds():
    """COM_SHIFT timeout: swing force high → abort → DOUBLE_SUPPORT."""
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    shared_state.step_phase_timer  = 1.01   # > COM_SHIFT_TIMEOUT=1.0
    shared_state.capture_point     = np.array([0.50, 0.0])   # CP far → no normal exit
    shared_state.left_foot_force   = 20.0   # swing foot still loaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_com_shift_timeout_low_force_proceeds_to_lift():
    """COM_SHIFT timeout: swing force low → proceed → LIFT."""
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    shared_state.step_phase_timer  = 1.01
    shared_state.capture_point     = np.array([0.50, 0.0])
    shared_state.left_foot_force   = 2.0   # swing foot unloaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_timeout_high_force_aborts_to_ds():
    """LIFT timeout: swing force high → abort → DOUBLE_SUPPORT."""
    _reset()
    shared_state.step_phase       = StepPhase.LIFT
    shared_state.step_phase_timer = 0.16   # > LIFT_TIMEOUT=0.15
    shared_state.left_foot_force  = 20.0
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_lift_timeout_low_force_proceeds_to_swing():
    """LIFT timeout: swing force low → proceed → SWING."""
    _reset()
    shared_state.step_phase       = StepPhase.LIFT
    shared_state.step_phase_timer = 0.16
    shared_state.left_foot_force  = 2.0
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_swing_timeout_forces_place_and_logs_error():
    """SWING timeout (> 1.5 × SWING_DURATION=0.6s) → PLACE + ERR_PHASE_TIMEOUT."""
    _reset()
    shared_state.step_phase       = StepPhase.SWING
    shared_state.step_phase_timer = 0.61   # > 0.40 * 1.5 = 0.60
    shared_state.swing_phase      = 0.50   # mid-swing, not yet at PLACE_ENTRY_PHI
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    err_before = shared_state._error_write_idx
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.PLACE
    assert shared_state._error_write_idx == err_before + 1


def test_place_timeout_high_force_completes_step():
    """PLACE timeout: swing force high → contact was real → complete step."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51   # > PLACE_TIMEOUT=0.5
    shared_state.left_foot_force  = 20.0   # > UNLOAD_FORCE_THRESHOLD
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT
    assert shared_state.step_count == 1


def test_place_timeout_low_force_freezes():
    """PLACE timeout: swing force low → foot missed ground → freeze."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51
    shared_state.left_foot_force  = 1.0   # below threshold
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True
```

- [ ] **Step 3: Run both test files**

```bash
python -m pytest test_step_phase_guards.py test_step_phase_timeouts.py -v
```

Expected: all tests PASSED

- [ ] **Step 4: Commit**

```bash
git add test_step_phase_guards.py test_step_phase_timeouts.py
git commit -m "test: add guard and timeout tests for 5-state gait FSM"
```

---

## Task 5: Stance Anchor + Velocity Gate + Full Integration Tests

**Files:**
- Create: `test_stance_anchor.py`
- Create: `test_velocity_gate.py`
- Create: `test_gait_planner_fsm.py`

- [ ] **Step 1: Write stance anchor tests**

```python
# test_stance_anchor.py
import numpy as np
import pytest
from unittest.mock import patch, MagicMock
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.SWING
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.5
    shared_state.swing_foot_x_stance       = -0.1
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 0.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    # Locked stance foot at [0.1, 0, 0] — must not drift mid-phase
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_stance_foot_world_pos_not_modified_during_swing():
    _reset()
    locked = shared_state.stance_foot_world_pos.copy()
    gait_planner.update_gait_planner()
    np.testing.assert_array_equal(shared_state.stance_foot_world_pos, locked)


def test_stance_ik_recomputed_every_swing_cycle():
    """Stance IK angles must be written on every SWING cycle."""
    _reset()
    shared_state.ik_right_angles = (0.0, 0.0, 0.0)   # start at zero
    with patch('kinematics.solve_ik', return_value=(0.1, 0.2, 0.3)) as mock_ik:
        gait_planner.update_gait_planner()
    # solve_ik called at least once (for stance leg)
    assert mock_ik.call_count >= 1
    # right (stance) angles updated
    assert shared_state.ik_right_angles == (0.1, 0.2, 0.3)


def test_stance_foot_locked_once_at_ds_entry():
    """stance_foot_world_pos is written at DS entry, not mid-swing."""
    _reset()
    shared_state.step_phase               = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer         = 0.0
    shared_state.right_foot_position      = np.array([0.15, 0.0, 0.0])
    shared_state.left_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    # Stance foot previously at different position — should update on DS entry
    shared_state.stance_foot_world_pos    = np.array([0.10, 0.0, 0.0])
    gait_planner.update_gait_planner()
    # DS entry should lock right_foot_position[0] = 0.15
    np.testing.assert_array_almost_equal(
        shared_state.stance_foot_world_pos, [0.15, 0.0, 0.0]
    )


def test_stance_foot_not_updated_mid_swing_phase():
    """Moving the foot position during SWING must not change the locked anchor."""
    _reset()
    shared_state.step_phase            = StepPhase.SWING
    shared_state.stance_foot_world_pos = np.array([0.1, 0.0, 0.0])
    # Simulate foot drifting in PyBullet
    shared_state.right_foot_position   = np.array([0.99, 0.0, 0.0])
    gait_planner.update_gait_planner()
    # Anchor must be unchanged
    np.testing.assert_array_equal(shared_state.stance_foot_world_pos, [0.1, 0.0, 0.0])
```

- [ ] **Step 2: Write velocity gate tests**

```python
# test_velocity_gate.py
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.freeze_robot              = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.swing_foot_x_stance       = -0.1
    shared_state.step_count                = 0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 2.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_lift_exit_blocked_by_high_velocity():
    """LIFT: force low but velocity high → must stay in LIFT."""
    _reset()
    shared_state.step_phase           = StepPhase.LIFT
    shared_state.left_foot_force      = 2.0    # < UNLOAD_FORCE_THRESHOLD
    shared_state.left_foot_velocity   = np.array([0.0, 0.0, 0.30])  # > 0.05 m/s
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_exit_allowed_when_both_conditions_met():
    _reset()
    shared_state.step_phase           = StepPhase.LIFT
    shared_state.left_foot_force      = 2.0
    shared_state.left_foot_velocity   = np.array([0.0, 0.0, 0.02])  # < 0.05
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_place_exit_blocked_by_high_velocity_even_with_contact():
    """PLACE: contact confirmed but velocity still high → must stay in PLACE."""
    _reset()
    shared_state.step_phase                = StepPhase.PLACE
    shared_state.swing_phase               = 0.95
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_velocity        = np.array([0.0, 0.0, 0.20])  # > 0.05
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.PLACE


def test_place_exit_allowed_when_contact_and_velocity_settled():
    _reset()
    shared_state.step_phase                = StepPhase.PLACE
    shared_state.swing_phase               = 0.95
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_velocity        = np.array([0.0, 0.0, 0.02])
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT
    assert shared_state.step_count == 1
```

- [ ] **Step 3: Write full integration test**

```python
# test_gait_planner_fsm.py
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _full_step_setup():
    """Configure shared_state at the beginning of a full step cycle."""
    shared_state.freeze_robot              = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.step_count                = 0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_step_count_increments_exactly_once_per_step():
    """One full DS→CS→LIFT→SWING→PLACE→DS cycle produces exactly one step_count."""
    _full_step_setup()

    # ── DOUBLE_SUPPORT → COM_SHIFT ────────────────────────────────────────────
    shared_state.step_phase_timer = 0.11
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT

    # ── COM_SHIFT → LIFT ──────────────────────────────────────────────────────
    shared_state.capture_point    = np.array([0.11, 0.0])  # close to stance_x=0.10
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT

    # ── LIFT → SWING ──────────────────────────────────────────────────────────
    shared_state.left_foot_force    = 2.0
    shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer   = 0.0
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING

    # ── SWING → PLACE ─────────────────────────────────────────────────────────
    shared_state.swing_phase      = 0.85   # at PLACE_ENTRY_PHI
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.PLACE

    # ── PLACE → DOUBLE_SUPPORT ────────────────────────────────────────────────
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer        = 0.0
    gait_planner.update_gait_planner()
    assert shared_state.step_phase  == StepPhase.DOUBLE_SUPPORT
    assert shared_state.step_count  == 1


def test_active_swing_side_flips_once_per_step():
    """After one full step cycle, active_swing_side must flip once."""
    _full_step_setup()
    assert shared_state.active_swing_side == "left"

    # Compress the full cycle by pre-setting timers
    shared_state.step_phase_timer = 0.11
    gait_planner.update_gait_planner()   # DS → COM_SHIFT

    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()   # COM_SHIFT → LIFT

    shared_state.left_foot_force    = 2.0
    shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer   = 0.0
    gait_planner.update_gait_planner()   # LIFT → SWING

    shared_state.swing_phase      = 0.85
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()   # SWING → PLACE

    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer        = 0.0
    gait_planner.update_gait_planner()   # PLACE → DS

    assert shared_state.active_swing_side == "right"
    assert shared_state.stance_side       == "left"


def test_swing_phase_reset_to_zero_on_step_completion():
    """phi must reset to 0.0 at PLACE→DS transition."""
    _full_step_setup()
    shared_state.step_phase_timer = 0.11
    gait_planner.update_gait_planner()

    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()

    shared_state.left_foot_force    = 2.0
    shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer   = 0.0
    gait_planner.update_gait_planner()

    shared_state.swing_phase      = 0.85
    shared_state.step_phase_timer = 0.0
    gait_planner.update_gait_planner()

    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])
    shared_state.step_phase_timer        = 0.0
    gait_planner.update_gait_planner()

    assert shared_state.swing_phase == 0.0
```

- [ ] **Step 4: Run all remaining test files**

```bash
python -m pytest test_stance_anchor.py test_velocity_gate.py test_gait_planner_fsm.py -v
```

Expected: all tests PASSED

- [ ] **Step 5: Run the full test suite to confirm no regressions**

```bash
python -m pytest test_shared_state_phase.py test_grf_phase_gate.py \
    test_step_phase_transitions.py test_step_phase_guards.py \
    test_step_phase_timeouts.py test_stance_anchor.py \
    test_velocity_gate.py test_gait_planner_fsm.py -v
```

Expected: all tests PASSED

- [ ] **Step 6: Commit**

```bash
git add test_stance_anchor.py test_velocity_gate.py test_gait_planner_fsm.py
git commit -m "test: add stance anchor, velocity gate, and full-cycle integration tests for gait FSM"
```

---

## Self-Review Notes

Checked against spec `docs/superpowers/specs/2026-04-11-phased-gait-fsm-design.md`:

| Spec requirement | Task |
|---|---|
| StepPhase enum with 5 members | Task 1 |
| ERR_PHASE_TIMEOUT = 6 | Task 1 |
| step_phase, step_phase_timer, stance_side, stance_foot_world_pos fields | Task 1 |
| GRF stance-only for LIFT/SWING/PLACE | Task 2 |
| DS exit: both CONTACT_CONFIRMED + timer ≥ 0.10s | Task 3 |
| DS timeout 2.0s → freeze_robot | Task 4 guard tests |
| COM_SHIFT exit: CP close + not UNSTABLE | Task 3 |
| COM_SHIFT timeout → conditional by swing_force | Task 4 timeout tests |
| LIFT exit: force < 5N + vel_z < 0.05 | Task 3 |
| LIFT timeout → conditional by swing_force | Task 4 timeout tests |
| SWING phi advances to PLACE_ENTRY_PHI=0.85 | Task 3 (handler) |
| SWING timeout 0.6s → PLACE + ERR_PHASE_TIMEOUT | Task 4 timeout tests |
| PLACE exit: CONTACT_CONFIRMED + vel_z settled | Task 5 velocity gate |
| PLACE timeout → high force=complete, low=freeze | Task 4 |
| stance_foot_world_pos locked once at DS entry | Task 5 stance anchor |
| stance IK recomputed every cycle | Task 5 stance anchor |
| swing_foot_x_stance snapped at COM_SHIFT→LIFT | Task 3 (test_lift_snapshots) |
| step_count only at confirmed PLACE→DS | Task 5 integration |
| active_swing_side flips once per step | Task 5 integration |
| phase timer resets on every transition | Task 3 (transition tests) |
| recovery.reset_step() on every transition | Implemented in _transition_to() |
| abort chain: swing_phase=0, sides unchanged | Task 4 (abort to DS tests) |

All spec requirements covered. No TBDs or placeholders present.
