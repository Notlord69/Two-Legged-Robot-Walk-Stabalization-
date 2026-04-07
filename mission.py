"""
================================================================================
PROJECT SICLO1 — MISSION CONTROLLER  (mission.py)
================================================================================

Five-state gait state machine:

    IDLE ──(walk commanded, both feet CONFIRMED)──► RAMP
    RAMP ──(ramp_gain == 1.0)──────────────────────► WALK
    WALK ──(steps_remaining == 1)──────────────────► DECEL
    DECEL ──(steps_remaining == 0)─────────────────► STOP
    STOP ──(ramp_gain == 0.0)──────────────────────► IDLE

RAMP: ramp_gain += RAMP_RATE (1/50) per cycle → 0.5 s to full torque
STOP: ramp_gain -= STOP_RATE (1/20) per cycle → 0.2 s to zero torque

STEP_LENGTH must match gait_planner.py's constant (0.12 m).

INPUTS (shared_state):
    step_count, left/right_foot_contact_state, freeze_robot, emergency_stop_triggered

OUTPUTS (shared_state):
    mission_state, steps_remaining, ramp_gain

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
from typing import Optional

from shared_state import shared_state, ContactState, MissionState


# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH: float = 0.12   # m, must match gait_planner.STEP_LENGTH exactly

RAMP_RATE: float = 1.0 / 50.0   # dimensionless/cycle, 0→1 over 0.5 s at 100 Hz
STOP_RATE: float = 1.0 / 20.0   # dimensionless/cycle, 1→0 over 0.2 s at 100 Hz


# ============================================================================
# MISSION CONTROLLER
# ============================================================================

class MissionController:
    """Gait state machine. One instance per simulation run.

    walk_distance: metres to walk before stopping. None → stay IDLE forever.
    """

    def __init__(self, walk_distance: Optional[float] = None):
        self.walk_distance: Optional[float] = walk_distance
        self._steps_total: int = (
            math.ceil(walk_distance / STEP_LENGTH)
            if walk_distance is not None
            else 0
        )

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        # Emergency stop: collapse everything immediately
        if shared_state.emergency_stop_triggered:
            shared_state.ramp_gain     = 0.0
            shared_state.mission_state = MissionState.IDLE
            return

        if shared_state.freeze_robot:
            return

        state = shared_state.mission_state

        if state == MissionState.IDLE:
            self._handle_idle()

        elif state == MissionState.RAMP:
            self._handle_ramp()

        elif state == MissionState.WALK:
            self._handle_walk()

        elif state == MissionState.DECEL:
            self._handle_decel()

        elif state == MissionState.STOP:
            self._handle_stop()

    # ------------------------------------------------------------------ #
    # STATE HANDLERS
    # ------------------------------------------------------------------ #

    def _handle_idle(self) -> None:
        """Transition to RAMP only when walk_distance is set and both feet confirmed."""
        if (self.walk_distance is not None and
                shared_state.left_foot_contact_state  == ContactState.CONTACT_CONFIRMED and
                shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED):
            shared_state.mission_state   = MissionState.RAMP
            shared_state.ramp_gain       = 0.0
            shared_state.step_count      = 0
            shared_state.steps_remaining = self._steps_total

    def _handle_ramp(self) -> None:
        """Increment ramp_gain. Transition to WALK when gain reaches 1.0."""
        shared_state.ramp_gain = min(1.0, shared_state.ramp_gain + RAMP_RATE)
        if shared_state.ramp_gain >= 1.0:
            shared_state.ramp_gain     = 1.0
            shared_state.mission_state = MissionState.WALK

    def _handle_walk(self) -> None:
        """Update steps_remaining. Transition to DECEL when 1 step left."""
        shared_state.steps_remaining = (
            self._steps_total - shared_state.step_count
        )
        if shared_state.steps_remaining <= 1:
            shared_state.mission_state = MissionState.DECEL

    def _handle_decel(self) -> None:
        """Update steps_remaining. Transition to STOP when no steps left."""
        shared_state.steps_remaining = max(
            0, self._steps_total - shared_state.step_count
        )
        if shared_state.steps_remaining == 0:
            shared_state.mission_state = MissionState.STOP

    def _handle_stop(self) -> None:
        """Decrement ramp_gain. Transition to IDLE when gain reaches 0."""
        shared_state.ramp_gain = max(0.0, shared_state.ramp_gain - STOP_RATE)
        if shared_state.ramp_gain <= 0.0:
            shared_state.ramp_gain     = 0.0
            shared_state.mission_state = MissionState.IDLE
