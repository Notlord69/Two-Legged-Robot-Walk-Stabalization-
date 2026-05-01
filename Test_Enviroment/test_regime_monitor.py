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


from regime_monitor import SignalSpec, REGIME_PROFILES


class TestSignalSpec:

    def test_evaluate_at_optimal(self):
        spec = SignalSpec(optimal=0.88, acceptable_band=0.02,
                          threshold_05=0.05, threshold_00=0.15)
        assert spec.evaluate(0.88) == 1.0

    def test_evaluate_beyond_threshold(self):
        spec = SignalSpec(optimal=0.88, acceptable_band=0.02,
                          threshold_05=0.05, threshold_00=0.15)
        assert spec.evaluate(0.50) == 0.0


class TestRegimeProfiles:

    def test_all_regimes_have_profiles(self):
        for regime in PrimaryRegime:
            assert regime in REGIME_PROFILES, f"Missing profile for {regime.name}"

    def test_idle_has_base_z_signal(self):
        profile = REGIME_PROFILES[PrimaryRegime.IDLE_STANDING]
        assert 'base_z' in profile

    def test_idle_base_z_optimal(self):
        spec = REGIME_PROFILES[PrimaryRegime.IDLE_STANDING]['base_z']
        assert spec.optimal == 0.88

    def test_walk_swing_has_swing_foot_z(self):
        profile = REGIME_PROFILES[PrimaryRegime.WALK_SWING]
        assert 'contact_force_stance' in profile

    def test_walk_swing_stance_force_optimal(self):
        spec = REGIME_PROFILES[PrimaryRegime.WALK_SWING]['contact_force_stance']
        assert spec.optimal == 79.5

    def test_frozen_has_lenient_thresholds(self):
        profile = REGIME_PROFILES[PrimaryRegime.FROZEN]
        spec = profile['base_z']
        assert spec.threshold_00 >= 0.50

    def test_profile_values_are_signal_specs(self):
        for regime, profile in REGIME_PROFILES.items():
            for signal_name, spec in profile.items():
                assert isinstance(spec, SignalSpec), (
                    f"{regime.name}.{signal_name} is {type(spec)}, expected SignalSpec"
                )

    def test_all_specs_have_valid_thresholds(self):
        for regime, profile in REGIME_PROFILES.items():
            for signal_name, spec in profile.items():
                assert spec.acceptable_band <= spec.threshold_05 <= spec.threshold_00, (
                    f"{regime.name}.{signal_name}: band={spec.acceptable_band} "
                    f"t05={spec.threshold_05} t00={spec.threshold_00}"
                )


from regime_monitor import RegimeMonitor, COL_COM_Z, COL_LEFT_CONTACT, COL_RIGHT_CONTACT
from regime_monitor import COL_LEFT_FORCE, COL_RIGHT_FORCE
from regime_monitor import COL_ANG_VEL_X, COL_ANG_VEL_Y, COL_ANG_VEL_Z
from regime_monitor import COL_RAMP_GAIN, COL_SWING_PHASE


class TestRegimeMonitor:

    def _idle_row(self) -> np.ndarray:
        """A healthy IDLE row with all signals at nominal."""
        row = np.zeros(72, dtype=np.float64)
        row[COL_CYCLE] = 200.0
        row[COL_COM_Z] = 0.88
        row[COL_LEFT_CONTACT] = 4.0   # CONTACT_CONFIRMED
        row[COL_RIGHT_CONTACT] = 4.0
        row[COL_LEFT_FORCE] = 39.7
        row[COL_RIGHT_FORCE] = 39.7
        row[COL_ANG_VEL_X] = 0.0
        row[COL_ANG_VEL_Y] = 0.0
        row[COL_ANG_VEL_Z] = 0.0
        row[COL_STEP_PHASE] = 1.0     # DOUBLE_SUPPORT
        row[COL_MISSION_STATE] = 1.0  # IDLE
        row[COL_RAMP_GAIN] = 0.0
        return row

    def test_nominal_idle(self):
        mon = RegimeMonitor()
        row = self._idle_row()
        regime, condition, conf = mon.classify(row)
        assert regime == PrimaryRegime.IDLE_STANDING
        assert condition == Condition.NOMINAL

    def test_degraded_when_base_z_drifts(self):
        mon = RegimeMonitor()
        row = self._idle_row()
        row[COL_COM_Z] = 0.83  # deviation 0.05 → at threshold_05 → conf = 0.5
        regime, condition, conf = mon.classify(row)
        assert regime == PrimaryRegime.IDLE_STANDING
        assert condition == Condition.DEGRADED
        assert conf['base_z'] <= 0.5

    def test_critical_when_base_z_extreme(self):
        mon = RegimeMonitor()
        row = self._idle_row()
        row[COL_COM_Z] = 0.50  # way below 0.0 threshold
        regime, condition, conf = mon.classify(row)
        assert regime == PrimaryRegime.IDLE_STANDING
        assert condition == Condition.CRITICAL
        assert conf['base_z'] == 0.0

    def test_frozen_on_stale_cycle(self):
        mon = RegimeMonitor()
        row1 = self._idle_row()
        row1[COL_CYCLE] = 100.0
        mon.classify(row1)

        row2 = self._idle_row()
        row2[COL_CYCLE] = 100.0  # same cycle = stale
        regime, condition, conf = mon.classify(row2)
        assert regime == PrimaryRegime.FROZEN
        assert condition == Condition.FALLEN

    def test_walk_swing_regime(self):
        mon = RegimeMonitor()
        row = np.zeros(72, dtype=np.float64)
        row[COL_CYCLE] = 300.0
        row[COL_COM_Z] = 0.84
        row[COL_MISSION_STATE] = 3.0  # WALK
        row[COL_STEP_PHASE] = 4.0     # SWING
        row[COL_LEFT_FORCE] = 79.5    # stance
        row[COL_RIGHT_FORCE] = 0.0    # swing
        row[COL_SWING_PHASE] = 0.5
        regime, condition, conf = mon.classify(row)
        assert regime == PrimaryRegime.WALK_SWING

    def test_confidence_dict_has_all_profile_signals(self):
        mon = RegimeMonitor()
        row = self._idle_row()
        regime, condition, conf = mon.classify(row)
        profile = REGIME_PROFILES[PrimaryRegime.IDLE_STANDING]
        for signal_name in profile:
            assert signal_name in conf, f"Missing {signal_name} in confidence dict"

    def test_all_confidences_are_floats_in_range(self):
        mon = RegimeMonitor()
        row = self._idle_row()
        _, _, conf = mon.classify(row)
        for name, val in conf.items():
            assert isinstance(val, float), f"{name} is {type(val)}"
            assert 0.0 <= val <= 1.0, f"{name} = {val}"
