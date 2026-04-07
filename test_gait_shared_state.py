"""Tests for new shared_state fields added for the Dynamic Gait Controller."""
import pytest
from shared_state import shared_state, Siclo1State


def test_mission_state_enum_values():
    from shared_state import MissionState
    assert hasattr(MissionState, 'IDLE')
    assert hasattr(MissionState, 'RAMP')
    assert hasattr(MissionState, 'WALK')
    assert hasattr(MissionState, 'DECEL')
    assert hasattr(MissionState, 'STOP')


def test_new_fields_exist_with_defaults():
    from shared_state import MissionState
    s = Siclo1State()
    assert s.grf_torque_correction == {}
    assert s.active_swing_side == "left"
    assert s.step_count == 0
    assert s.mission_state == MissionState.IDLE
    assert s.steps_remaining == 0
    assert s.ramp_gain == 0.0
    assert s.swing_foot_x_stance == 0.0


def test_grf_torque_correction_is_dict():
    s = Siclo1State()
    s.grf_torque_correction['Left_Hip_Forwards'] = 3.5
    assert s.grf_torque_correction['Left_Hip_Forwards'] == 3.5


def test_ramp_gain_bounds():
    s = Siclo1State()
    assert 0.0 <= s.ramp_gain <= 1.0
