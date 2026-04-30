# RegimeMonitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a 10 Hz RegimeMonitor observer that classifies robot state into 11 primary regimes with per-signal confidence scoring and a 4-level condition overlay.

**Architecture:** Pure observer on the telemetry consumer thread. Reads 72-column telemetry rows from `TelemetryRingBuffer.read_batch()`. Outputs `(PrimaryRegime, Condition, confidence_dict)` per row. No writes to `shared_state`. Integration point is `TelemetryThread._format_batch()` — one extra method call per row after CSV write.

**Tech Stack:** Python 3.10+, numpy, pytest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-04-30-regime-discovery-design.md`

---

## File Structure

| File | Responsibility |
|------|---------------|
| **Create:** `regime_monitor.py` | Enums (`PrimaryRegime`, `Condition`), `SignalSpec` dataclass, `REGIME_PROFILES` dict, `compute_confidence()`, `RegimeMonitor` class |
| **Create:** `Test_Enviroment/test_regime_monitor.py` | Unit tests for confidence function, regime lookup, condition aggregation, profile evaluation |
| **Modify:** `telemetry.py:251-283` | Call `RegimeMonitor.classify(row)` inside `_format_batch()`, append regime/condition to log line |

All regime logic lives in `regime_monitor.py`. The telemetry integration is a 4-line change.

---

## Task 1: Enums and Confidence Function

**Files:**
- Create: `regime_monitor.py`
- Create: `Test_Enviroment/test_regime_monitor.py`

### Step 1.1: Write failing tests for `compute_confidence()`

- [ ] Create `Test_Enviroment/test_regime_monitor.py` with these tests:

```python
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
        assert compute_confidence(0.90, 0.88, 0.02, 0.05, 0.15) == 1.0

    def test_at_band_edge_returns_1(self):
        assert compute_confidence(0.86, 0.88, 0.02, 0.05, 0.15) == 1.0

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
```

- [ ] Run tests to verify they fail:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py -v
```

Expected: `ModuleNotFoundError: No module named 'regime_monitor'`

### Step 1.2: Implement enums and `compute_confidence()`

- [ ] Create `regime_monitor.py`:

```python
"""
================================================================================
PROJECT SICLO1 — REGIME MONITOR  (regime_monitor.py)
================================================================================

10 Hz observer: classifies robot state into primary regimes with per-signal
confidence scoring and condition overlay.

Reads 72-column telemetry rows from TelemetryRingBuffer.
Does NOT write to shared_state — pure observer.

INPUTS:  telemetry row (numpy array, 72 columns)
OUTPUTS: (PrimaryRegime, Condition, confidence_dict)

Spec: docs/superpowers/specs/2026-04-30-regime-discovery-design.md

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

from enum import IntEnum, auto
from typing import Dict, Tuple, NamedTuple


# ============================================================================
# ENUMS
# ============================================================================

class PrimaryRegime(IntEnum):
    FROZEN         = 0
    IDLE_STANDING  = 1
    RAMP_UP        = 2
    WALK_DS        = 3
    WALK_COM_SHIFT = 4
    WALK_LIFT      = 5
    WALK_SWING     = 6
    WALK_PLACE     = 7
    DECEL_SWING    = 8
    DECEL_DS       = 9
    RAMP_DOWN      = 10


class Condition(IntEnum):
    NOMINAL  = 0
    DEGRADED = 1
    CRITICAL = 2
    FALLEN   = 3


# ============================================================================
# TELEMETRY COLUMN INDICES (from shared_state.py TelemetryRingBuffer layout)
# ============================================================================

COL_CYCLE         = 1
COL_ERROR_CODE    = 2
COL_COM_Z         = 5
COL_LEFT_CONTACT  = 9
COL_RIGHT_CONTACT = 10
COL_LEFT_FORCE    = 12
COL_RIGHT_FORCE   = 13
COL_ANG_VEL_X     = 16
COL_ANG_VEL_Y     = 17
COL_ANG_VEL_Z     = 18
COL_ERR_L_HIP     = 33
COL_ERR_L_KNEE    = 34
COL_ERR_L_ANKLE   = 35
COL_ERR_R_HIP     = 36
COL_ERR_R_KNEE    = 37
COL_ERR_R_ANKLE   = 38
COL_STEP_PHASE    = 51
COL_SWING_PHASE   = 52
COL_MISSION_STATE = 54
COL_RAMP_GAIN     = 55
COL_L_FOOT_Z      = 58
COL_R_FOOT_Z      = 61

# MissionState enum values (from shared_state.py auto())
_MS_IDLE  = 1
_MS_RAMP  = 2
_MS_WALK  = 3
_MS_DECEL = 4
_MS_STOP  = 5

# StepPhase enum values
_SP_DOUBLE_SUPPORT = 1
_SP_COM_SHIFT      = 2
_SP_LIFT           = 3
_SP_SWING          = 4
_SP_PLACE          = 5

# ContactState.CONTACT_CONFIRMED value
_CONTACT_CONFIRMED = 4


# ============================================================================
# CONFIDENCE FUNCTION
# ============================================================================

def compute_confidence(measured: float, optimal: float,
                       acceptable_band: float,
                       threshold_05: float,
                       threshold_00: float) -> float:
    """Piecewise linear confidence with explicit 0.5 knee.

    Returns 1.0 inside acceptable_band, ramps linearly to 0.5 at
    threshold_05, then linearly to 0.0 at threshold_00.
    """
    deviation = abs(measured - optimal)
    if deviation <= acceptable_band:
        return 1.0
    if deviation <= threshold_05:
        return 0.5 + 0.5 * (threshold_05 - deviation) / (threshold_05 - acceptable_band)
    if deviation <= threshold_00:
        return 0.5 * (threshold_00 - deviation) / (threshold_00 - threshold_05)
    return 0.0
```

- [ ] Run tests to verify they pass:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestComputeConfidence -v
```

Expected: All 10 tests PASS.

### Step 1.3: Commit

- [ ] Commit:

```bash
git add regime_monitor.py Test_Enviroment/test_regime_monitor.py
git commit -m "feat(regime_monitor): add enums and confidence function with tests"
```

---

## Task 2: Regime Lookup

**Files:**
- Modify: `regime_monitor.py`
- Modify: `Test_Enviroment/test_regime_monitor.py`

### Step 2.1: Write failing tests for `classify_regime()`

- [ ] Append to `Test_Enviroment/test_regime_monitor.py`:

```python
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
        # FROZEN detection: classify_regime itself doesn't detect frozen
        # (that's RegimeMonitor's job via cycle staleness). Here we just
        # verify the lookup handles all valid states.
        assert classify_regime(row) == PrimaryRegime.IDLE_STANDING
```

- [ ] Run tests to verify they fail:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestClassifyRegime -v
```

Expected: `ImportError: cannot import name 'classify_regime'`

### Step 2.2: Implement `classify_regime()`

- [ ] Append to `regime_monitor.py`:

```python
import numpy as np


def classify_regime(row: np.ndarray) -> PrimaryRegime:
    """Deterministic regime lookup from telemetry row.

    Does NOT detect FROZEN — that requires cross-row cycle staleness
    detection, handled by RegimeMonitor.classify().
    """
    ms = int(row[COL_MISSION_STATE])
    sp = int(row[COL_STEP_PHASE])

    if ms == _MS_IDLE:
        return PrimaryRegime.IDLE_STANDING
    if ms == _MS_RAMP:
        return PrimaryRegime.RAMP_UP
    if ms == _MS_STOP:
        return PrimaryRegime.RAMP_DOWN

    if ms == _MS_WALK:
        if sp == _SP_DOUBLE_SUPPORT:
            return PrimaryRegime.WALK_DS
        if sp == _SP_COM_SHIFT:
            return PrimaryRegime.WALK_COM_SHIFT
        if sp == _SP_LIFT:
            return PrimaryRegime.WALK_LIFT
        if sp == _SP_SWING:
            return PrimaryRegime.WALK_SWING
        if sp == _SP_PLACE:
            return PrimaryRegime.WALK_PLACE

    if ms == _MS_DECEL:
        if sp in (_SP_SWING, _SP_PLACE):
            return PrimaryRegime.DECEL_SWING
        return PrimaryRegime.DECEL_DS

    return PrimaryRegime.IDLE_STANDING
```

- [ ] Run tests:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestClassifyRegime -v
```

Expected: All 14 tests PASS.

### Step 2.3: Commit

- [ ] Commit:

```bash
git add regime_monitor.py Test_Enviroment/test_regime_monitor.py
git commit -m "feat(regime_monitor): add regime lookup from telemetry row"
```

---

## Task 3: Signal Specs and Regime Profiles

**Files:**
- Modify: `regime_monitor.py`
- Modify: `Test_Enviroment/test_regime_monitor.py`

This task defines the `SignalSpec` dataclass and all 11 regime profiles with their derived thresholds.

### Step 3.1: Write failing tests for regime profiles

- [ ] Append to `Test_Enviroment/test_regime_monitor.py`:

```python
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
```

- [ ] Run to verify failure:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestSignalSpec -v
```

Expected: `ImportError: cannot import name 'SignalSpec'`

### Step 3.2: Implement `SignalSpec` and `REGIME_PROFILES`

- [ ] Add to `regime_monitor.py` (after `compute_confidence`, before `classify_regime`):

```python
# ============================================================================
# SIGNAL SPECIFICATION
# ============================================================================

class SignalSpec(NamedTuple):
    """Expected value and confidence thresholds for one signal in one regime."""
    optimal: float
    acceptable_band: float
    threshold_05: float
    threshold_00: float

    def evaluate(self, measured: float) -> float:
        return compute_confidence(measured, self.optimal,
                                  self.acceptable_band,
                                  self.threshold_05,
                                  self.threshold_00)


# ============================================================================
# REGIME PROFILES — all thresholds derived in the spec
# ============================================================================

# Shared signal specs reused across regimes
_BASE_Z_IDLE       = SignalSpec(0.88, 0.02, 0.05, 0.15)
_BASE_Z_RAMP       = SignalSpec(0.88, 0.03, 0.06, 0.25)
_BASE_Z_DS         = SignalSpec(0.86, 0.04, 0.08, 0.26)
_BASE_Z_SHIFT      = SignalSpec(0.86, 0.04, 0.08, 0.26)
_BASE_Z_LIFT       = SignalSpec(0.85, 0.04, 0.08, 0.25)
_BASE_Z_SWING      = SignalSpec(0.84, 0.05, 0.10, 0.29)
_BASE_Z_PLACE      = SignalSpec(0.84, 0.05, 0.10, 0.29)
_BASE_Z_RAMP_DOWN  = SignalSpec(0.87, 0.03, 0.06, 0.27)
_BASE_Z_FROZEN     = SignalSpec(0.50, 0.50, 0.50, 0.50)

_ANG_VEL_IDLE      = SignalSpec(0.0, 0.1, 0.5, 1.0)
_ANG_VEL_RAMP      = SignalSpec(0.0, 0.3, 0.8, 1.5)
_ANG_VEL_DS        = SignalSpec(0.0, 0.5, 1.0, 2.0)
_ANG_VEL_SHIFT     = SignalSpec(0.0, 0.8, 1.5, 2.5)
_ANG_VEL_SWING     = SignalSpec(0.0, 1.0, 2.0, 3.0)
_ANG_VEL_FROZEN    = SignalSpec(0.0, 3.0, 3.0, 5.0)

_FORCE_DS          = SignalSpec(39.7, 12.0, 20.0, 34.7)
_FORCE_DS_WIDE     = SignalSpec(39.7, 15.0, 25.0, 34.7)
_FORCE_STANCE      = SignalSpec(79.5, 24.5, 44.5, 69.5)
_FORCE_STANCE_WIDE = SignalSpec(79.5, 30.0, 50.0, 69.5)
_FORCE_SWING_ZERO  = SignalSpec(0.0, 0.0, 3.0, 10.0)

_WBC_ERR_IDLE      = SignalSpec(0.0, 0.05, 0.2, 0.5)
_WBC_ERR_RAMP      = SignalSpec(0.0, 0.1, 0.3, 0.5)
_WBC_ERR_DS        = SignalSpec(0.0, 0.15, 0.3, 0.5)
_WBC_ERR_SWING     = SignalSpec(0.0, 0.2, 0.6, 1.0)

_RAMP_GAIN_ZERO    = SignalSpec(0.0, 0.0, 0.0, 0.5)
_RAMP_GAIN_FULL    = SignalSpec(1.0, 0.0, 0.0, 0.5)

_N_CONTACTS_2      = SignalSpec(2.0, 0.0, 1.0, 2.0)
_N_CONTACTS_1      = SignalSpec(1.0, 0.0, 1.0, 1.0)


REGIME_PROFILES: Dict[PrimaryRegime, Dict[str, SignalSpec]] = {

    PrimaryRegime.IDLE_STANDING: {
        'base_z':              _BASE_Z_IDLE,
        'angular_velocity':    _ANG_VEL_IDLE,
        'contact_force_left':  _FORCE_DS,
        'contact_force_right': _FORCE_DS,
        'wbc_tracking_error':  _WBC_ERR_IDLE,
        'ramp_gain':           _RAMP_GAIN_ZERO,
    },

    PrimaryRegime.RAMP_UP: {
        'base_z':              _BASE_Z_RAMP,
        'angular_velocity':    _ANG_VEL_RAMP,
        'contact_force_left':  _FORCE_DS_WIDE,
        'contact_force_right': _FORCE_DS_WIDE,
        'wbc_tracking_error':  _WBC_ERR_RAMP,
    },

    PrimaryRegime.WALK_DS: {
        'base_z':              _BASE_Z_DS,
        'angular_velocity':    _ANG_VEL_DS,
        'contact_force_left':  _FORCE_DS_WIDE,
        'contact_force_right': _FORCE_DS_WIDE,
        'wbc_tracking_error':  _WBC_ERR_DS,
    },

    PrimaryRegime.WALK_COM_SHIFT: {
        'base_z':              _BASE_Z_SHIFT,
        'angular_velocity':    _ANG_VEL_SHIFT,
        'contact_force_left':  _FORCE_DS_WIDE,
        'contact_force_right': _FORCE_DS_WIDE,
        'wbc_tracking_error':  _WBC_ERR_DS,
    },

    PrimaryRegime.WALK_LIFT: {
        'base_z':              _BASE_Z_LIFT,
        'angular_velocity':    _ANG_VEL_SHIFT,
        'contact_force_stance': _FORCE_STANCE,
        'contact_force_swing':  _FORCE_SWING_ZERO,
        'wbc_tracking_error':   _WBC_ERR_DS,
    },

    PrimaryRegime.WALK_SWING: {
        'base_z':              _BASE_Z_SWING,
        'angular_velocity':    _ANG_VEL_SWING,
        'contact_force_stance': _FORCE_STANCE,
        'contact_force_swing':  _FORCE_SWING_ZERO,
        'wbc_tracking_error':   _WBC_ERR_SWING,
    },

    PrimaryRegime.WALK_PLACE: {
        'base_z':              _BASE_Z_PLACE,
        'angular_velocity':    _ANG_VEL_SWING,
        'contact_force_stance': _FORCE_STANCE_WIDE,
        'wbc_tracking_error':   _WBC_ERR_SWING,
    },

    PrimaryRegime.DECEL_SWING: {
        'base_z':              _BASE_Z_SWING,
        'angular_velocity':    _ANG_VEL_SWING,
        'contact_force_stance': _FORCE_STANCE_WIDE,
        'contact_force_swing':  _FORCE_SWING_ZERO,
        'wbc_tracking_error':   _WBC_ERR_SWING,
    },

    PrimaryRegime.DECEL_DS: {
        'base_z':              _BASE_Z_DS,
        'angular_velocity':    _ANG_VEL_DS,
        'contact_force_left':  _FORCE_DS_WIDE,
        'contact_force_right': _FORCE_DS_WIDE,
        'wbc_tracking_error':  _WBC_ERR_DS,
    },

    PrimaryRegime.RAMP_DOWN: {
        'base_z':              _BASE_Z_RAMP_DOWN,
        'angular_velocity':    _ANG_VEL_RAMP,
        'contact_force_left':  _FORCE_DS_WIDE,
        'contact_force_right': _FORCE_DS_WIDE,
    },

    PrimaryRegime.FROZEN: {
        'base_z':              _BASE_Z_FROZEN,
        'angular_velocity':    _ANG_VEL_FROZEN,
    },
}
```

- [ ] Run all tests:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py -v
```

Expected: All tests PASS (confidence, regime lookup, signal spec, regime profiles).

### Step 3.3: Commit

- [ ] Commit:

```bash
git add regime_monitor.py Test_Enviroment/test_regime_monitor.py
git commit -m "feat(regime_monitor): add signal specs and regime profiles"
```

---

## Task 4: RegimeMonitor Class

**Files:**
- Modify: `regime_monitor.py`
- Modify: `Test_Enviroment/test_regime_monitor.py`

The `RegimeMonitor` class is the main entry point. It holds a `_prev_cycle` for FROZEN detection, dispatches `classify_regime()`, evaluates signals against the profile, and returns the output tuple.

### Step 4.1: Write failing tests for `RegimeMonitor.classify()`

- [ ] Append to `Test_Enviroment/test_regime_monitor.py`:

```python
from regime_monitor import RegimeMonitor


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
```

- [ ] Run to verify failure:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestRegimeMonitor -v
```

Expected: `ImportError: cannot import name 'RegimeMonitor'`

### Step 4.2: Implement `RegimeMonitor`

- [ ] Append to `regime_monitor.py`:

```python
# ============================================================================
# SIGNAL EXTRACTION HELPERS
# ============================================================================

def _extract_signal(row: np.ndarray, signal_name: str,
                    regime: PrimaryRegime) -> float:
    """Extract a named signal value from the 72-column telemetry row.

    For stance/swing force signals, determines which foot is stance
    from the swing_side column (col 53: 0=left swing, 1=right swing).
    """
    if signal_name == 'base_z':
        return float(row[COL_COM_Z])

    if signal_name == 'angular_velocity':
        ax = row[COL_ANG_VEL_X]
        ay = row[COL_ANG_VEL_Y]
        az = row[COL_ANG_VEL_Z]
        return float(np.sqrt(ax*ax + ay*ay + az*az))

    if signal_name == 'contact_force_left':
        return float(row[COL_LEFT_FORCE])

    if signal_name == 'contact_force_right':
        return float(row[COL_RIGHT_FORCE])

    if signal_name == 'contact_force_stance':
        swing_side_val = row[53]  # 0 = left swing, 1 = right swing
        if swing_side_val < 0.5:
            return float(row[COL_RIGHT_FORCE])  # right is stance
        return float(row[COL_LEFT_FORCE])       # left is stance

    if signal_name == 'contact_force_swing':
        swing_side_val = row[53]
        if swing_side_val < 0.5:
            return float(row[COL_LEFT_FORCE])   # left is swing
        return float(row[COL_RIGHT_FORCE])      # right is swing

    if signal_name == 'wbc_tracking_error':
        errors = [abs(row[i]) for i in range(COL_ERR_L_HIP, COL_ERR_R_ANKLE + 1)]
        return float(max(errors)) if errors else 0.0

    if signal_name == 'ramp_gain':
        return float(row[COL_RAMP_GAIN])

    return 0.0


# ============================================================================
# REGIME MONITOR
# ============================================================================

ClassifyResult = Tuple[PrimaryRegime, Condition, Dict[str, float]]


class RegimeMonitor:
    """10 Hz observer: classifies each telemetry row into regime + condition.

    Call classify(row) for each 72-column telemetry row.
    Stateful: tracks previous cycle count for FROZEN detection.
    """

    def __init__(self):
        self._prev_cycle: float = -1.0

    def classify(self, row: np.ndarray) -> ClassifyResult:
        """Classify one telemetry row.

        Returns (PrimaryRegime, Condition, confidence_dict).
        """
        cycle = float(row[COL_CYCLE])

        # FROZEN detection: cycle count did not advance since last call
        if cycle == self._prev_cycle and self._prev_cycle >= 0:
            self._prev_cycle = cycle
            return self._evaluate_frozen(row)

        self._prev_cycle = cycle

        regime = classify_regime(row)
        return self._evaluate(row, regime)

    def _evaluate_frozen(self, row: np.ndarray) -> ClassifyResult:
        """Evaluate signals against FROZEN profile."""
        profile = REGIME_PROFILES[PrimaryRegime.FROZEN]
        conf = {}
        for signal_name, spec in profile.items():
            measured = _extract_signal(row, signal_name, PrimaryRegime.FROZEN)
            conf[signal_name] = spec.evaluate(measured)
        return PrimaryRegime.FROZEN, Condition.FALLEN, conf

    def _evaluate(self, row: np.ndarray,
                  regime: PrimaryRegime) -> ClassifyResult:
        """Evaluate signals against the regime's profile and compute condition."""
        profile = REGIME_PROFILES[regime]
        conf: Dict[str, float] = {}

        for signal_name, spec in profile.items():
            measured = _extract_signal(row, signal_name, regime)
            conf[signal_name] = spec.evaluate(measured)

        if not conf:
            return regime, Condition.NOMINAL, conf

        min_conf = min(conf.values())

        if min_conf == 0.0:
            condition = Condition.CRITICAL
        elif min_conf <= 0.5:
            condition = Condition.DEGRADED
        else:
            condition = Condition.NOMINAL

        return regime, condition, conf
```

- [ ] Run all tests:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py -v
```

Expected: All tests PASS.

### Step 4.3: Commit

- [ ] Commit:

```bash
git add regime_monitor.py Test_Enviroment/test_regime_monitor.py
git commit -m "feat(regime_monitor): add RegimeMonitor class with signal evaluation"
```

---

## Task 5: Telemetry Thread Integration

**Files:**
- Modify: `telemetry.py:206-283`
- Modify: `Test_Enviroment/test_regime_monitor.py`

### Step 5.1: Write failing integration test

- [ ] Append to `Test_Enviroment/test_regime_monitor.py`:

```python
class TestTelemetryIntegration:
    """Verify RegimeMonitor is callable from telemetry drain context."""

    def test_monitor_classify_returns_valid_tuple(self):
        mon = RegimeMonitor()
        row = np.zeros(72, dtype=np.float64)
        row[COL_CYCLE] = 100.0
        row[COL_COM_Z] = 0.88
        row[COL_MISSION_STATE] = 1.0
        row[COL_STEP_PHASE] = 1.0
        row[COL_LEFT_FORCE] = 39.7
        row[COL_RIGHT_FORCE] = 39.7
        result = mon.classify(row)
        assert len(result) == 3
        regime, condition, conf = result
        assert isinstance(regime, PrimaryRegime)
        assert isinstance(condition, Condition)
        assert isinstance(conf, dict)

    def test_monitor_processes_batch(self):
        """Simulate what _format_batch does: classify each row in a batch."""
        mon = RegimeMonitor()
        batch = np.zeros((10, 72), dtype=np.float64)
        for i in range(10):
            batch[i, COL_CYCLE] = float(100 + i)
            batch[i, COL_COM_Z] = 0.88
            batch[i, COL_MISSION_STATE] = 1.0
            batch[i, COL_STEP_PHASE] = 1.0
            batch[i, COL_LEFT_FORCE] = 39.7
            batch[i, COL_RIGHT_FORCE] = 39.7

        results = []
        for row in batch:
            results.append(mon.classify(row))

        assert len(results) == 10
        assert all(r[0] == PrimaryRegime.IDLE_STANDING for r in results)
        assert all(r[1] == Condition.NOMINAL for r in results)
```

- [ ] Run to verify they pass (these tests only exercise regime_monitor.py, not the telemetry wiring):

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py::TestTelemetryIntegration -v
```

Expected: PASS.

### Step 5.2: Wire RegimeMonitor into TelemetryThread

- [ ] Modify `telemetry.py`. Add import at top (after existing imports):

Add after line 25 (`from shared_state import Siclo1State, ERR_TIMING_VIOLATION`):
```python
from regime_monitor import RegimeMonitor
```

- [ ] Add `self._regime_monitor = RegimeMonitor()` in `TelemetryThread.__init__`, after `self._quiet = quiet` (line 213):

```python
        self._regime_monitor = RegimeMonitor()
```

- [ ] In `_format_batch()`, add regime classification after the CSV write and before the log line construction. After line 255 (`self._session.append_row(row)`) and before line 258 (`# Unpack key columns`), insert:

```python
            # Regime classification (10 Hz observer)
            regime, condition, _conf = self._regime_monitor.classify(row)
```

- [ ] Modify the log line construction (around line 280) to include regime and condition. Change:

```python
            line = (f"[t={ts:7.3f}s c={int(cycle):>6d}] "
                    f"COM=[{cx:+.3f},{cy:+.3f},{cz:+.3f}] "
                    f"L={int(lc)} R={int(rc)} Stab={int(stab)} "
                    f"F=[{lf:.0f},{rf:.0f}] "
                    f"margin={margin:.4f} "
                    f"phase={int(step_ph)} swing={swing_ph:.2f} "
                    f"compute={comp_us:.0f}us")
```

To:

```python
            line = (f"[t={ts:7.3f}s c={int(cycle):>6d}] "
                    f"COM=[{cx:+.3f},{cy:+.3f},{cz:+.3f}] "
                    f"L={int(lc)} R={int(rc)} Stab={int(stab)} "
                    f"F=[{lf:.0f},{rf:.0f}] "
                    f"margin={margin:.4f} "
                    f"phase={int(step_ph)} swing={swing_ph:.2f} "
                    f"compute={comp_us:.0f}us "
                    f"regime={regime.name} cond={condition.name}")
```

- [ ] Run existing telemetry tests to check for regressions:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_telemetry.py -v
```

Expected: All existing telemetry tests PASS.

- [ ] Run all regime monitor tests:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_regime_monitor.py -v
```

Expected: All PASS.

### Step 5.3: Commit

- [ ] Commit:

```bash
git add telemetry.py regime_monitor.py Test_Enviroment/test_regime_monitor.py
git commit -m "feat(telemetry): wire RegimeMonitor into 10 Hz drain loop"
```

---

## Task 6: Full Test Suite Run and Verification

**Files:**
- No new files

### Step 6.1: Run full test suite

- [ ] Run all tests to verify no regressions:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/ -v --tb=short 2>&1 | tail -30
```

Expected: All tests PASS. Zero failures.

### Step 6.2: Verify module imports cleanly

- [ ] Quick smoke test:

```bash
cd /home/notlord/ros2_ws/Siclo1_V1 && python -c "
from regime_monitor import RegimeMonitor, PrimaryRegime, Condition, REGIME_PROFILES
import numpy as np

mon = RegimeMonitor()
row = np.zeros(72)
row[1] = 100  # cycle
row[5] = 0.88  # com_z
row[12] = 39.7  # left force
row[13] = 39.7  # right force
row[51] = 1  # DOUBLE_SUPPORT
row[54] = 1  # IDLE
regime, cond, conf = mon.classify(row)
print(f'Regime: {regime.name}')
print(f'Condition: {cond.name}')
print(f'Confidence: {conf}')
print(f'Profiles: {len(REGIME_PROFILES)} regimes')
print('OK')
"
```

Expected output:
```
Regime: IDLE_STANDING
Condition: NOMINAL
Confidence: {'base_z': 1.0, 'angular_velocity': 1.0, ...}
Profiles: 11 regimes
OK
```

### Step 6.3: Commit (if any fixes were needed)

- [ ] If fixes were needed, commit:

```bash
git add -A && git commit -m "fix(regime_monitor): address test suite issues"
```

---

## Summary

| Task | What | Files | Tests |
|------|------|-------|-------|
| 1 | Enums + confidence function | regime_monitor.py (create) | 10 |
| 2 | Regime lookup | regime_monitor.py | 14 |
| 3 | SignalSpec + profiles | regime_monitor.py | 8 |
| 4 | RegimeMonitor class | regime_monitor.py | 7 |
| 5 | Telemetry integration | telemetry.py | 2 + regression |
| 6 | Full suite verification | — | all |

**Total new tests:** ~41
**New files:** 1 (`regime_monitor.py`)
**Modified files:** 1 (`telemetry.py` — 4 lines changed)
