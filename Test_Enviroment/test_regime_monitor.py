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
