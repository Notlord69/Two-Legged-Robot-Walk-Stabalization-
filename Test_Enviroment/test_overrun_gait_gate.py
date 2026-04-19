"""
Tests for gait-phase gate on mid-cycle timing overrun (Fix 2).

Problem: when ERR_MID_CYCLE_OVERRUN fires (computation already exceeded 10 ms
before the gait pipeline even runs), gait_planner.update_gait_planner() still
executes.  The phase advance decision then uses sensor data from a stale,
overlong cycle.  If the cycle is long because the physics solver is in a bad
state, the resulting joint targets may destabilise the robot.

Fix: add timing_violation_this_cycle flag to shared_state.
  - HeartBeat.step() resets it to False at cycle start.
  - HeartBeat.step() sets it to True when mid-cycle overrun is detected.
  - gait_planner.update() returns immediately when the flag is True,
    holding the current phase and IK angles until the next clean cycle.
"""
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState, StepPhase, StabilityStatus,
)
import gait_planner


def _reset_lift():
    """LIFT-phase state where swing is unloaded and stance is bearing load —
    the exact condition that would normally trigger LIFT → SWING."""
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.LIFT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 2.0   # swing unloaded
    shared_state.right_foot_force          = 30.0  # stance loaded
    shared_state.left_foot_velocity        = np.array([0.0, 0.0, 0.01])
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.02])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_gait_advances_normally_without_overrun():
    """Baseline: flag False → LIFT advances to SWING as expected."""
    _reset_lift()
    shared_state.timing_violation_this_cycle = False
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_gait_phase_holds_on_overrun_cycle():
    """
    Flag True → gait_planner must not advance the phase.
    Same sensor state as above (would normally transition) but overrun set.
    """
    _reset_lift()
    shared_state.timing_violation_this_cycle = True
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT, (
        "Phase must not advance when timing_violation_this_cycle is True."
    )


def test_phase_timer_does_not_advance_on_overrun():
    """Timer increment must also be suppressed on an overrun cycle."""
    _reset_lift()
    shared_state.step_phase_timer            = 0.05
    shared_state.timing_violation_this_cycle = True
    gait_planner.update_gait_planner()
    # Timer should not have moved; update() returned before incrementing
    assert shared_state.step_phase_timer == pytest.approx(0.05, abs=1e-9)


def test_overrun_flag_does_not_affect_frozen_or_idle():
    """Frozen / IDLE gate still takes priority over the overrun gate."""
    _reset_lift()
    shared_state.timing_violation_this_cycle = True
    shared_state.freeze_robot = True
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT   # still held — no crash
