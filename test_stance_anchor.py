"""Tests for stance foot world-position anchor locking in the gait FSM."""
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
