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

import numpy as np


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

        if cycle == self._prev_cycle and self._prev_cycle >= 0:
            self._prev_cycle = cycle
            return self._evaluate_frozen(row)

        self._prev_cycle = cycle

        regime = classify_regime(row)
        return self._evaluate(row, regime)

    def _evaluate_frozen(self, row: np.ndarray) -> ClassifyResult:
        profile = REGIME_PROFILES[PrimaryRegime.FROZEN]
        conf = {}
        for signal_name, spec in profile.items():
            measured = _extract_signal(row, signal_name, PrimaryRegime.FROZEN)
            conf[signal_name] = spec.evaluate(measured)
        return PrimaryRegime.FROZEN, Condition.FALLEN, conf

    def _evaluate(self, row: np.ndarray,
                  regime: PrimaryRegime) -> ClassifyResult:
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
