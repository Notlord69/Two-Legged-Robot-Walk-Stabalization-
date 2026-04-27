"""Tests for balance controller fields in shared_state.py."""
import pytest
import numpy as np
from shared_state import shared_state, Siclo1State, STAGE_NAMES


def test_balance_hip_roll_fields_exist():
    """balance_hip_roll_left/right default to 0.0."""
    s = Siclo1State()
    assert hasattr(s, 'balance_hip_roll_left')
    assert hasattr(s, 'balance_hip_roll_right')
    assert s.balance_hip_roll_left == 0.0
    assert s.balance_hip_roll_right == 0.0


def test_sagittal_pitch_offset_field_exists():
    """sagittal_pitch_offset defaults to 0.0."""
    s = Siclo1State()
    assert hasattr(s, 'sagittal_pitch_offset')
    assert s.sagittal_pitch_offset == 0.0


def test_emergency_sagittal_torque_field_exists():
    """emergency_sagittal_torque defaults to empty dict."""
    s = Siclo1State()
    assert hasattr(s, 'emergency_sagittal_torque')
    assert s.emergency_sagittal_torque == {}


def test_stability_margin_lateral_field_exists():
    """stability_margin_lateral defaults to 0.0."""
    s = Siclo1State()
    assert hasattr(s, 'stability_margin_lateral')
    assert s.stability_margin_lateral == 0.0


def test_stability_margin_sagittal_field_exists():
    """stability_margin_sagittal defaults to 0.0."""
    s = Siclo1State()
    assert hasattr(s, 'stability_margin_sagittal')
    assert s.stability_margin_sagittal == 0.0


def test_capture_point_error_fields_exist():
    """capture_point_error_lateral and _sagittal default to 0.0."""
    s = Siclo1State()
    assert hasattr(s, 'capture_point_error_lateral')
    assert hasattr(s, 'capture_point_error_sagittal')
    assert s.capture_point_error_lateral == 0.0
    assert s.capture_point_error_sagittal == 0.0


def test_balance_mode_fields_exist():
    """balance_mode_lateral and _sagittal default to INACTIVE."""
    s = Siclo1State()
    assert hasattr(s, 'balance_mode_lateral')
    assert hasattr(s, 'balance_mode_sagittal')
    assert s.balance_mode_lateral == "INACTIVE"
    assert s.balance_mode_sagittal == "INACTIVE"


def test_stage_names_has_balance():
    """'balance' in STAGE_NAMES, 'active_balance' not."""
    assert 'balance' in STAGE_NAMES
    assert 'active_balance' not in STAGE_NAMES


def test_reset_clears_balance_fields():
    """Set all balance fields to non-default, call reset(), verify cleared."""
    s = Siclo1State()
    # Set to non-default values
    s.balance_hip_roll_left = 0.15
    s.balance_hip_roll_right = -0.10
    s.sagittal_pitch_offset = 0.05
    s.emergency_sagittal_torque = {'Left_Hip_Forwards': 10.0}
    s.stability_margin_lateral = 0.03
    s.stability_margin_sagittal = 0.04
    s.capture_point_error_lateral = 0.02
    s.capture_point_error_sagittal = -0.01
    s.balance_mode_lateral = "ACTIVE"
    s.balance_mode_sagittal = "EMERGENCY"

    s.reset()

    assert s.balance_hip_roll_left == 0.0
    assert s.balance_hip_roll_right == 0.0
    assert s.sagittal_pitch_offset == 0.0
    assert s.emergency_sagittal_torque == {}
    assert s.stability_margin_lateral == 0.0
    assert s.stability_margin_sagittal == 0.0
    assert s.capture_point_error_lateral == 0.0
    assert s.capture_point_error_sagittal == 0.0
    assert s.balance_mode_lateral == "INACTIVE"
    assert s.balance_mode_sagittal == "INACTIVE"
