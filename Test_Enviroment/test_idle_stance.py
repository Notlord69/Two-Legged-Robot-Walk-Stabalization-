"""Tests for idle stance — gait planner must produce IK targets during IDLE."""
import math
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import shared_state, MissionState, StepPhase, ContactState


def _reset_for_idle():
    """Set up shared_state for an IDLE-standing robot."""
    shared_state.reset()
    shared_state.mission_state = MissionState.IDLE
    shared_state.ramp_gain = 0.0
    shared_state.freeze_robot = False
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 0.0
    shared_state.stance_side = "right"
    shared_state.timing_violation_this_cycle = False
    shared_state.com_position = np.array([0.0, 0.0, 0.75])
    shared_state.com_velocity = np.zeros(3)
    shared_state.link_positions = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.1, 0.75]),
        'Right_Upper_Leg_1': np.array([0.0, -0.1, 0.75]),
    }
    shared_state.left_foot_position = np.array([0.0, 0.1, 0.0])
    shared_state.right_foot_position = np.array([0.0, -0.1, 0.0])
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)


def test_idle_sets_nonzero_ik_targets():
    """IDLE must produce non-zero IK angles — not (0,0,0,0)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    update_gait_planner()
    left = shared_state.ik_left_angles
    right = shared_state.ik_right_angles
    assert any(abs(a) > 0.01 for a in left), f"Left IK still zero: {left}"
    assert any(abs(a) > 0.01 for a in right), f"Right IK still zero: {right}"


def test_idle_ik_within_joint_limits():
    """IDLE stance angles must be within URDF limits (±1.571 rad)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    update_gait_planner()
    limit = 1.570796  # rad, URDF ±π/2
    for label, angles in [("left", shared_state.ik_left_angles),
                          ("right", shared_state.ik_right_angles)]:
        for i, a in enumerate(angles):
            assert abs(a) <= limit + 0.001, (
                f"{label}[{i}] = {a:.4f} exceeds ±{limit:.4f}")


def test_idle_fallback_on_ik_failure():
    """If IK raises ValueError, fallback angles must be used (non-zero, within limits)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    with patch('gait_planner.kinematics.solve_ik', side_effect=ValueError("mocked")):
        update_gait_planner()
    left = shared_state.ik_left_angles
    right = shared_state.ik_right_angles
    limit = 1.570796
    assert any(abs(a) > 0.01 for a in left), f"Left fallback still zero: {left}"
    assert any(abs(a) > 0.01 for a in right), f"Right fallback still zero: {right}"
    for label, angles in [("left", left), ("right", right)]:
        for i, a in enumerate(angles):
            assert abs(a) <= limit + 0.001, (
                f"Fallback {label}[{i}] = {a:.4f} exceeds limits")
