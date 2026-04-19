"""Tests for StepPhase enum, ERR_PHASE_TIMEOUT, and phase FSM fields in shared_state."""
import numpy as np
import pytest
from shared_state import (
    Siclo1State, StepPhase,
    ERR_PHASE_TIMEOUT,
)


def test_step_phase_enum_members():
    phases = [p.name for p in StepPhase]
    assert phases == ["DOUBLE_SUPPORT", "COM_SHIFT", "LIFT", "SWING", "PLACE"]


def test_err_phase_timeout_value():
    assert ERR_PHASE_TIMEOUT == 6


def test_step_phase_default():
    s = Siclo1State()
    assert s.step_phase == StepPhase.DOUBLE_SUPPORT


def test_step_phase_timer_default():
    s = Siclo1State()
    assert s.step_phase_timer == 0.0


def test_stance_side_default():
    s = Siclo1State()
    assert s.stance_side == "right"   # complement of active_swing_side="left"


def test_stance_foot_world_pos_default():
    s = Siclo1State()
    assert isinstance(s.stance_foot_world_pos, np.ndarray)
    assert s.stance_foot_world_pos.shape == (3,)
    np.testing.assert_array_equal(s.stance_foot_world_pos, [0.0, 0.0, 0.0])
