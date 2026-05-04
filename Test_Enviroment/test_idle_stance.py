"""Tests for idle stance — gait planner must produce IK targets during IDLE."""
import math
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import shared_state, MissionState, StepPhase, ContactState


def _reset_for_idle():
    """Set up shared_state for an IDLE-standing robot."""
    from gait_planner import reset_gait_planner
    shared_state.reset()
    shared_state.joint_positions.clear()
    shared_state.joint_velocities.clear()
    reset_gait_planner()
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


# ── Ramp-to-stance tests ────────────────────────────────────────────────────

def test_ramp_first_cycle_is_partial():
    """First idle cycle produces angles between start (0) and target, not full target."""
    from gait_planner import update_gait_planner, _IDLE_FALLBACK_LEFT
    _reset_for_idle()
    update_gait_planner()
    left = shared_state.ik_left_angles
    for i in range(4):
        if abs(_IDLE_FALLBACK_LEFT[i]) > 0.01:
            assert abs(left[i]) < abs(_IDLE_FALLBACK_LEFT[i]) * 0.5, (
                f"Left[{i}]={left[i]:.4f} is too close to full target "
                f"{_IDLE_FALLBACK_LEFT[i]:.4f} on first cycle")


def test_ramp_completes_after_n_cycles():
    """After STANCE_RAMP_CYCLES calls, IK angles reach the full stance target."""
    from gait_planner import update_gait_planner, STANCE_RAMP_CYCLES
    _reset_for_idle()
    for _ in range(STANCE_RAMP_CYCLES + 1):
        update_gait_planner()
    left = shared_state.ik_left_angles
    right = shared_state.ik_right_angles
    assert abs(left[1]) > 1.0, f"Left hip_pitch {left[1]:.4f} not at stance after ramp"
    assert abs(right[1]) > 1.0, f"Right hip_pitch {right[1]:.4f} not at stance after ramp"


def test_get_idle_stance_angles_covers_all_joints():
    """get_idle_stance_angles() must return all 8 controlled joints."""
    from gait_planner import get_idle_stance_angles
    angles = get_idle_stance_angles()
    expected = [
        'Left_Hip_Inwards', 'Left_Hip_Forwards', 'Left_Knee', 'Left_Ankle',
        'Right_Hip_Inwards', 'Right_Hip_Fowards', 'Right_Knee', 'Right_Ankle',
    ]
    for name in expected:
        assert name in angles, f"Missing joint: {name}"


def test_get_idle_stance_angles_hip_signs():
    """Left hip pitch must be positive, right negative (URDF sign convention)."""
    from gait_planner import get_idle_stance_angles
    angles = get_idle_stance_angles()
    assert angles['Left_Hip_Forwards'] > 1.0, (
        f"Left_Hip_Forwards={angles['Left_Hip_Forwards']:.4f} should be ~+1.22 rad")
    assert angles['Right_Hip_Fowards'] < -1.0, (
        f"Right_Hip_Fowards={angles['Right_Hip_Fowards']:.4f} should be ~-1.22 rad")


def test_ramp_resets_on_planner_reset():
    """reset_gait_planner() clears ramp state so next idle call re-snapshots."""
    from gait_planner import update_gait_planner, reset_gait_planner, STANCE_RAMP_CYCLES
    _reset_for_idle()
    for _ in range(STANCE_RAMP_CYCLES + 1):
        update_gait_planner()
    left_before = shared_state.ik_left_angles
    reset_gait_planner()
    shared_state.mission_state = MissionState.IDLE
    update_gait_planner()
    left_after = shared_state.ik_left_angles
    assert abs(left_after[1]) < abs(left_before[1]), (
        f"After reset, first-cycle angle {left_after[1]:.4f} should be less than "
        f"full target {left_before[1]:.4f}")
