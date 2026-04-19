"""Tests for WBC tracking telemetry fields."""

import pytest
from shared_state import shared_state


def test_tracking_error_field_exists():
    """shared_state has wbc_tracking_error dict."""
    assert hasattr(shared_state, 'wbc_tracking_error')
    assert isinstance(shared_state.wbc_tracking_error, dict)


def test_saturation_flag_field_exists():
    """shared_state has wbc_torque_saturated dict."""
    assert hasattr(shared_state, 'wbc_torque_saturated')
    assert isinstance(shared_state.wbc_torque_saturated, dict)


def test_tracking_error_initially_empty():
    """wbc_tracking_error starts as empty dict."""
    shared_state.reset()
    assert shared_state.wbc_tracking_error == {}


def test_saturation_flag_initially_empty():
    """wbc_torque_saturated starts as empty dict."""
    shared_state.reset()
    assert shared_state.wbc_torque_saturated == {}


def test_wbc_kp_reduced():
    """WBC_KP should be 100.0 N·m/rad (reduced from 200)."""
    from HeartBeat import WBC_KP
    assert WBC_KP == 100.0, f"WBC_KP={WBC_KP}, expected 100.0"


def test_wbc_kd_increased():
    """WBC_KD should be 28.0 N·m·s/rad (increased from 15)."""
    from HeartBeat import WBC_KD
    assert WBC_KD == 28.0, f"WBC_KD={WBC_KD}, expected 28.0"
