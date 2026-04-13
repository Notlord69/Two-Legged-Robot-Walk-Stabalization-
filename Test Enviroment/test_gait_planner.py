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
    # Set phase just below 1.0; one more dt will push it past 1.0
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) + 1e-9
    update_gait_planner()
    assert shared_state.step_count == 1
    assert shared_state.swing_phase == 0.0


def test_swing_side_swaps_after_step():
    """active_swing_side flips left↔right after each step completes."""
    from gait_planner import update_gait_planner, SWING_DURATION
    _reset_for_planner()
    shared_state.active_swing_side = "left"
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) + 1e-9
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


def test_reset_step_called_at_touchdown():
    """recovery.reset_step() is called exactly once when swing_phase crosses 1.0."""
    from unittest.mock import patch
    from gait_planner import update_gait_planner, SWING_DURATION

    _reset_for_planner()
    # Set phase so the next update pushes phi past 1.0
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) + 1e-9

    with patch('recovery.reset_step') as mock_reset:
        update_gait_planner()

    mock_reset.assert_called_once()
    # Also verify step_start_time was updated (reset_step writes sim_time)
    # We verify the mock was called; the actual write is tested in recovery tests.
