"""Tests for velocity-settled gate in LIFT and PLACE phase exits."""
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
