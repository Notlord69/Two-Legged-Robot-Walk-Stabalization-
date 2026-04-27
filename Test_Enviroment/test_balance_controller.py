"""Tests for the unified balance controller (balance_controller.py)."""
import pytest
import numpy as np
from shared_state import (
    shared_state, Siclo1State, ContactState, StepPhase, MissionState,
)
from balance_controller import (
    update_balance, reset_balance, get_balance_diagnostics,
    HIP_ROLL_MAX, HIP_ROLL_RATE_LIMIT, PITCH_OFFSET_MAX,
    EMERGENCY_THRESHOLD,
)


def _reset_for_balance():
    """Configure shared_state for a nominal walking scenario."""
    shared_state.reset()
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = 1.0
    shared_state.freeze_robot = False
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.stance_side = "right"
    shared_state.com_position = np.array([0.0, 0.25, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_position = np.array([-0.1, 0.25, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.25, 0.0])
    shared_state.stance_foot_world_pos = np.array([0.1, 0.25, 0.0])
    shared_state.capture_point = np.array([0.0, 0.25])
    reset_balance()


# ── Lateral axis tests ────────────────────────────────────────────────────────

def test_lateral_zero_error_zero_roll():
    """Zero lateral CP error produces near-zero hip roll."""
    _reset_for_balance()
    update_balance()
    assert abs(shared_state.balance_hip_roll_left) < 1e-6
    assert abs(shared_state.balance_hip_roll_right) < 1e-6


def test_lateral_positive_error_negative_roll():
    """COM shifted right of support center should produce some roll < 0."""
    _reset_for_balance()
    # Shift COM to the right (positive X) of the support center
    shared_state.com_position = np.array([0.15, 0.25, 0.8806])
    shared_state.capture_point = np.array([0.15, 0.25])
    # Run several cycles so rate limiting doesn't mask the direction
    for _ in range(10):
        update_balance()
    # During walking, stance side gets the full target: -(GAIN * positive_error)
    # So the stance-side roll should be negative
    assert shared_state.balance_hip_roll_right < 0.0


def test_lateral_roll_clipped_to_max():
    """Extreme lateral error should clip hip roll to HIP_ROLL_MAX."""
    _reset_for_balance()
    shared_state.com_position = np.array([5.0, 0.25, 0.8806])
    shared_state.capture_point = np.array([5.0, 0.25])
    for _ in range(100):
        update_balance()
    assert abs(shared_state.balance_hip_roll_left) <= HIP_ROLL_MAX + 1e-9
    assert abs(shared_state.balance_hip_roll_right) <= HIP_ROLL_MAX + 1e-9


def test_lateral_rate_limiting():
    """A single cycle with large error should not exceed HIP_ROLL_RATE_LIMIT."""
    _reset_for_balance()
    shared_state.com_position = np.array([5.0, 0.25, 0.8806])
    shared_state.capture_point = np.array([5.0, 0.25])
    update_balance()
    # From zero, a single cycle can move at most HIP_ROLL_RATE_LIMIT
    assert abs(shared_state.balance_hip_roll_left) <= HIP_ROLL_RATE_LIMIT + 1e-9
    assert abs(shared_state.balance_hip_roll_right) <= HIP_ROLL_RATE_LIMIT + 1e-9


# ── Sagittal axis tests ──────────────────────────────────────────────────────

def test_sagittal_zero_error_zero_offset():
    """Zero sagittal CP error produces near-zero pitch offset."""
    _reset_for_balance()
    update_balance()
    assert abs(shared_state.sagittal_pitch_offset) < 1e-6


def test_sagittal_positive_error_negative_offset():
    """COM forward of support center should produce a negative pitch offset after iterations."""
    _reset_for_balance()
    # CP_y (forward) is larger than support center y
    shared_state.com_position = np.array([0.0, 0.30, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.30])
    for _ in range(20):
        update_balance()
    assert shared_state.sagittal_pitch_offset < 0.0


def test_sagittal_pitch_offset_clipped():
    """Extreme sagittal error clips pitch offset to PITCH_OFFSET_MAX."""
    _reset_for_balance()
    shared_state.com_position = np.array([0.0, 5.0, 0.8806])
    shared_state.capture_point = np.array([0.0, 5.0])
    for _ in range(100):
        update_balance()
    assert abs(shared_state.sagittal_pitch_offset) <= PITCH_OFFSET_MAX + 1e-9


# ── Emergency mode tests ─────────────────────────────────────────────────────

def test_emergency_activation():
    """|e_y| > EMERGENCY_THRESHOLD triggers non-zero emergency torque."""
    _reset_for_balance()
    # Forward error > 0.08 m
    shared_state.com_position = np.array([0.0, 0.45, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.45])
    for _ in range(5):
        update_balance()
    assert len(shared_state.emergency_sagittal_torque) > 0
    assert any(v != 0.0 for v in shared_state.emergency_sagittal_torque.values())


def test_emergency_hysteresis():
    """Emergency activates at 0.20m, stays active at 0.07m, exits below 0.06m."""
    _reset_for_balance()
    # 1) Activate emergency: large forward error
    shared_state.com_position = np.array([0.0, 0.45, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.45])
    for _ in range(5):
        update_balance()
    assert shared_state.balance_mode_sagittal == "EMERGENCY"

    # 2) Reduce to just below threshold but above (threshold - hysteresis)
    # e_y ~ 0.07 (support center y ~ 0.25, so CP_y ~ 0.32)
    shared_state.com_position = np.array([0.0, 0.32, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.32])
    for _ in range(3):
        update_balance()
    assert shared_state.balance_mode_sagittal == "EMERGENCY"

    # 3) Reduce below (threshold - hysteresis) to exit
    # e_y < 0.06 (support center y ~ 0.25, so CP_y ~ 0.30)
    shared_state.com_position = np.array([0.0, 0.30, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.30])
    for _ in range(3):
        update_balance()
    assert shared_state.balance_mode_sagittal == "NORMAL"


# ── Safety gate tests ────────────────────────────────────────────────────────

def test_freeze_produces_zero_outputs():
    """freeze_robot set to True should produce all zero outputs."""
    _reset_for_balance()
    shared_state.freeze_robot = True
    update_balance()
    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.balance_hip_roll_right == 0.0
    assert shared_state.sagittal_pitch_offset == 0.0
    assert shared_state.emergency_sagittal_torque == {}


def test_emergency_stop_produces_zero_outputs():
    """emergency_stop_triggered should produce all zero outputs."""
    _reset_for_balance()
    shared_state.emergency_stop_triggered = True
    update_balance()
    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.balance_hip_roll_right == 0.0
    assert shared_state.sagittal_pitch_offset == 0.0
    assert shared_state.emergency_sagittal_torque == {}


# ── Diagnostics test ─────────────────────────────────────────────────────────

def test_diagnostics_returns_dict():
    """get_balance_diagnostics returns a dict with expected keys."""
    _reset_for_balance()
    update_balance()
    diag = get_balance_diagnostics()
    assert isinstance(diag, dict)
    assert 'lateral_error' in diag
    assert 'sagittal_error' in diag
    assert 'mode_lateral' in diag
    assert 'mode_sagittal' in diag
