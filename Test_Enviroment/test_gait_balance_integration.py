# Test_Enviroment/test_gait_balance_integration.py
"""Tests for gait_planner reading hip_roll from balance_controller via shared_state."""
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, ContactState, MissionState, StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.reset()
    shared_state.freeze_robot = False
    shared_state.timing_violation_this_cycle = False
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = 1.0
    shared_state.step_phase = StepPhase.SWING
    shared_state.step_phase_timer = 0.0
    shared_state.active_swing_side = "left"
    shared_state.stance_side = "right"
    shared_state.swing_phase = 0.3
    shared_state.swing_foot_x_stance = 0.0
    shared_state.capture_point = np.array([0.0, 0.0])
    shared_state.stability_status = StabilityStatus.STABLE
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force = 0.0
    shared_state.right_foot_force = 30.0
    shared_state.left_foot_position = np.array([0.0, 0.0, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.0, 0.0])
    shared_state.left_foot_velocity = np.zeros(3)
    shared_state.right_foot_velocity = np.zeros(3)
    shared_state.stance_foot_world_pos = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions = {
        'Left_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }
    shared_state.com_position = np.array([0.0, 0.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    # Balance controller outputs (would be written by balance_controller.py)
    shared_state.balance_hip_roll_left = 0.0
    shared_state.balance_hip_roll_right = 0.0


def test_stance_ik_uses_balance_hip_roll():
    """Stance IK hip_roll comes from shared_state.balance_hip_roll_*, not internal computation."""
    _reset()
    # Set a specific hip roll via balance controller output
    shared_state.balance_hip_roll_right = 0.12  # stance side is right
    with patch('kinematics.solve_ik', return_value=(0.1, 0.2, 0.3)):
        gait_planner.update_gait_planner()
    # Stance (right) angles should use the balance roll
    assert shared_state.ik_right_angles[0] == 0.12


def test_swing_ik_hip_roll_is_zero():
    """Swing leg hip_roll is zero during swing (leg returns to neutral)."""
    _reset()
    shared_state.balance_hip_roll_left = 0.15  # swing side is left
    with patch('kinematics.solve_ik', return_value=(0.1, 0.2, 0.3)):
        gait_planner.update_gait_planner()
    # Swing (left) angles: hip_roll should be 0 during swing
    assert shared_state.ik_left_angles[0] == 0.0


def test_non_stance_ik_uses_balance_roll_in_ds():
    """During DOUBLE_SUPPORT, non-stance leg reads hip roll from balance_controller."""
    _reset()
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 0.0
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force = 30.0
    shared_state.right_foot_force = 30.0
    shared_state.balance_hip_roll_left = -0.05  # non-stance (left)
    shared_state.joint_positions = {
        'Left_Hip_Forwards': 0.1, 'Left_Knee': 0.2, 'Left_Ankle': 0.0,
        'Right_Hip_Fowards': 0.1, 'Right_Knee': 0.2, 'Right_Ankle': 0.0,
        'Left_Hip_Inwards': 0.0, 'Right_Hip_Inwards': 0.0,
    }
    gait_planner.update_gait_planner()
    # Non-stance (left) should use balance_hip_roll_left
    assert shared_state.ik_left_angles[0] == -0.05


def test_com_shift_sagittal_gate():
    """COM_SHIFT does not exit until BOTH lateral AND sagittal CP are within threshold."""
    _reset()
    shared_state.step_phase = StepPhase.COM_SHIFT
    shared_state.step_phase_timer = 0.0
    shared_state.stability_status = StabilityStatus.STABLE
    # Lateral CP close to stance foot, but sagittal CP far
    shared_state.capture_point = np.array([0.1, 0.5])  # sagittal far from stance Y=0.0
    shared_state.stance_foot_world_pos = np.array([0.1, 0.0, 0.0])
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.joint_positions = {
        'Left_Hip_Forwards': 0.0, 'Left_Knee': 0.0, 'Left_Ankle': 0.0,
        'Right_Hip_Fowards': 0.0, 'Right_Knee': 0.0, 'Right_Ankle': 0.0,
        'Left_Hip_Inwards': 0.0, 'Right_Hip_Inwards': 0.0,
    }
    gait_planner.update_gait_planner()
    # Should still be in COM_SHIFT (sagittal gate blocks)
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_gait_planner_has_no_compute_hip_roll():
    """_compute_hip_roll method should not exist after refactor."""
    assert not hasattr(gait_planner._gait_planner, '_compute_hip_roll')
