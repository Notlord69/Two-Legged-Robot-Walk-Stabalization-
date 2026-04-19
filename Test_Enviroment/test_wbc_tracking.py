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
