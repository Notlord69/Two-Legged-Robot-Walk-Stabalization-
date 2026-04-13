"""Tests for HeartbeatController strict/non-strict timing modes."""
from unittest.mock import patch, MagicMock
import pytest


def test_strict_mode_propagates_violation_to_shared_state():
    from HeartBeat import HeartbeatController
    hb = HeartbeatController(target_dt=0.000001, strict=True)
    hb.start_cycle()
    with patch('HeartBeat.shared_state') as mock_ss:
        mock_ss.increment_timing_violations = MagicMock()
        mock_ss.add_error_code = MagicMock()
        violation, _ = hb.end_cycle()
    assert violation is True
    mock_ss.increment_timing_violations.assert_called_once()


def test_non_strict_mode_does_not_propagate_violation():
    from HeartBeat import HeartbeatController
    hb = HeartbeatController(target_dt=0.000001, strict=False)
    hb.start_cycle()
    with patch('HeartBeat.shared_state') as mock_ss:
        mock_ss.increment_timing_violations = MagicMock()
        mock_ss.add_error_code = MagicMock()
        violation, _ = hb.end_cycle()
    assert violation is True
    mock_ss.increment_timing_violations.assert_not_called()
    mock_ss.add_error_code.assert_not_called()


def test_non_strict_still_counts_violation_internally():
    from HeartBeat import HeartbeatController
    for strict in (True, False):
        hb = HeartbeatController(target_dt=0.000001, strict=strict)
        hb.start_cycle()
        with patch('HeartBeat.shared_state'):
            violation, _ = hb.end_cycle()
        assert hb._violations_count == 1, f"failed for strict={strict}"


def test_strict_defaults_to_true():
    from HeartBeat import HeartbeatController, TARGET_DT
    hb = HeartbeatController(target_dt=TARGET_DT)
    assert hb.strict is True
