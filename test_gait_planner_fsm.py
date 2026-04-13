"""Full step-cycle integration tests for the 5-state gait FSM."""
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
    shared_state.freeze_robot                = False
    shared_state.timing_violation_this_cycle = False
    shared_state.mission_state               = MissionState.WALK
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
