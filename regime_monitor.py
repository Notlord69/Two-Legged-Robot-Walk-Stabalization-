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
