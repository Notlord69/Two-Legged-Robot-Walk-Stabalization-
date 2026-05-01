"""Tests for regime_monitor.py — confidence function, regime lookup, condition overlay."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest
from regime_monitor import compute_confidence, PrimaryRegime, Condition


class TestComputeConfidence:
    """Piecewise linear confidence: 1.0 inside band, ramp to 0.5, ramp to 0.0."""

    def test_at_optimal_returns_1(self):
        assert compute_confidence(0.88, 0.88, 0.02, 0.05, 0.15) == 1.0

    def test_within_acceptable_band_returns_1(self):
        assert compute_confidence(0.87, 0.88, 0.02, 0.05, 0.15) == 1.0
        assert compute_confidence(0.90, 0.88, 0.02, 0.05, 0.15) == pytest.approx(1.0)

    def test_at_band_edge_returns_1(self):
        assert compute_confidence(0.86, 0.88, 0.02, 0.05, 0.15) == pytest.approx(1.0)

    def test_between_band_and_05_threshold(self):
        # deviation = 0.035, band = 0.02, threshold_05 = 0.05
        # expected = 0.5 + 0.5 * (0.05 - 0.035) / (0.05 - 0.02) = 0.5 + 0.5 * 0.5 = 0.75
        result = compute_confidence(0.845, 0.88, 0.02, 0.05, 0.15)
        assert abs(result - 0.75) < 1e-9

    def test_at_05_threshold_returns_05(self):
        result = compute_confidence(0.83, 0.88, 0.02, 0.05, 0.15)
        assert abs(result - 0.5) < 1e-9

    def test_between_05_and_00_threshold(self):
        # deviation = 0.10, threshold_05 = 0.05, threshold_00 = 0.15
        # expected = 0.5 * (0.15 - 0.10) / (0.15 - 0.05) = 0.5 * 0.5 = 0.25
        result = compute_confidence(0.78, 0.88, 0.02, 0.05, 0.15)
        assert abs(result - 0.25) < 1e-9

    def test_at_00_threshold_returns_0(self):
        result = compute_confidence(0.73, 0.88, 0.02, 0.05, 0.15)
        assert abs(result - 0.0) < 1e-9

    def test_beyond_00_threshold_returns_0(self):
        assert compute_confidence(0.50, 0.88, 0.02, 0.05, 0.15) == 0.0

    def test_negative_deviation_symmetric(self):
        above = compute_confidence(0.92, 0.88, 0.02, 0.05, 0.15)
        below = compute_confidence(0.84, 0.88, 0.02, 0.05, 0.15)
        assert abs(above - below) < 1e-9

    def test_zero_band_degenerates_cleanly(self):
        result = compute_confidence(0.88, 0.88, 0.0, 0.05, 0.15)
        assert result == 1.0


import numpy as np
from regime_monitor import classify_regime, COL_MISSION_STATE, COL_STEP_PHASE, COL_CYCLE


def _make_row(**overrides) -> np.ndarray:
    """Build a 72-column telemetry row with sensible defaults."""
    row = np.zeros(72, dtype=np.float64)
    row[COL_CYCLE] = 100.0
    row[COL_MISSION_STATE] = 1.0  # IDLE
    row[COL_STEP_PHASE] = 1.0    # DOUBLE_SUPPORT
    for col, val in overrides.items():
        row[int(col)] = val
    return row


class TestClassifyRegime:

    def test_idle_standing(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 1.0  # IDLE
        assert classify_regime(row) == PrimaryRegime.IDLE_STANDING

    def test_ramp_up(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 2.0  # RAMP
        assert classify_regime(row) == PrimaryRegime.RAMP_UP

    def test_walk_double_support(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 3.0  # WALK
        row[COL_STEP_PHASE] = 1.0     # DOUBLE_SUPPORT
        assert classify_regime(row) == PrimaryRegime.WALK_DS

    def test_walk_com_shift(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 3.0
        row[COL_STEP_PHASE] = 2.0     # COM_SHIFT
        assert classify_regime(row) == PrimaryRegime.WALK_COM_SHIFT

    def test_walk_lift(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 3.0
        row[COL_STEP_PHASE] = 3.0     # LIFT
        assert classify_regime(row) == PrimaryRegime.WALK_LIFT

    def test_walk_swing(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 3.0
        row[COL_STEP_PHASE] = 4.0     # SWING
        assert classify_regime(row) == PrimaryRegime.WALK_SWING

    def test_walk_place(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 3.0
        row[COL_STEP_PHASE] = 5.0     # PLACE
        assert classify_regime(row) == PrimaryRegime.WALK_PLACE

    def test_decel_swing(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 4.0  # DECEL
        row[COL_STEP_PHASE] = 4.0     # SWING
        assert classify_regime(row) == PrimaryRegime.DECEL_SWING

    def test_decel_place(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 4.0
        row[COL_STEP_PHASE] = 5.0     # PLACE
        assert classify_regime(row) == PrimaryRegime.DECEL_SWING

    def test_decel_ds(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 4.0
        row[COL_STEP_PHASE] = 1.0     # DOUBLE_SUPPORT
        assert classify_regime(row) == PrimaryRegime.DECEL_DS

    def test_decel_com_shift(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 4.0
        row[COL_STEP_PHASE] = 2.0     # COM_SHIFT
        assert classify_regime(row) == PrimaryRegime.DECEL_DS

    def test_decel_lift(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 4.0
        row[COL_STEP_PHASE] = 3.0     # LIFT
        assert classify_regime(row) == PrimaryRegime.DECEL_DS

    def test_ramp_down(self):
        row = _make_row()
        row[COL_MISSION_STATE] = 5.0  # STOP
        assert classify_regime(row) == PrimaryRegime.RAMP_DOWN

    def test_frozen_detected_from_stale_cycle(self):
        row = _make_row()
        row[COL_CYCLE] = 100.0
        assert classify_regime(row) == PrimaryRegime.IDLE_STANDING
