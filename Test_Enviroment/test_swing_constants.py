"""Tests for updated swing trajectory constants."""

import pytest


def test_swing_height_increased():
    """SWING_HEIGHT should be 0.06 m (increased from 0.04)."""
    from gait_planner import SWING_HEIGHT
    assert SWING_HEIGHT == 0.06, f"SWING_HEIGHT={SWING_HEIGHT}, expected 0.06"


def test_swing_duration_increased():
    """SWING_DURATION should be 0.50 s (increased from 0.40)."""
    from gait_planner import SWING_DURATION
    assert SWING_DURATION == 0.50, f"SWING_DURATION={SWING_DURATION}, expected 0.50"


def test_swing_height_units():
    """SWING_HEIGHT is in meters, must be positive and reasonable."""
    from gait_planner import SWING_HEIGHT
    assert 0.02 < SWING_HEIGHT < 0.15, "SWING_HEIGHT out of reasonable range"


def test_swing_duration_units():
    """SWING_DURATION is in seconds, must be positive and reasonable."""
    from gait_planner import SWING_DURATION
    assert 0.2 < SWING_DURATION < 1.0, "SWING_DURATION out of reasonable range"
