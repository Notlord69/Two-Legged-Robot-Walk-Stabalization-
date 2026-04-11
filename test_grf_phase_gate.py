"""Tests for stance-only GRF gating in LIFT/SWING/PLACE phases."""
import numpy as np
import pytest
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
    # Stance (left) still gets GRF
    assert result['Left_Hip_Forwards'] != 0.0
