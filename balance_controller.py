"""
================================================================================
PROJECT SICLO1 - UNIFIED BALANCE CONTROLLER
================================================================================

Replaces active_balance.py with a 2-axis LIPM capture point controller.

THEORY
------
Linear Inverted Pendulum Model (LIPM):
    omega = sqrt(g / z_com)         # natural frequency
    CP    = com_xy + v_com_xy / omega   # capture point (2D)

LATERAL AXIS (X):
    CP error -> hip roll position targets on Hip_Inwards joints.
    target_roll = -(GAIN * e_x + KD * v_x), clipped and rate-limited.
    Walking: stance leg gets full target, swing leg gets -target * 0.5.
    Standing: both legs get target * 0.5.

SAGITTAL AXIS (Y):
    CP error -> pitch offset added to IK hip_pitch before WBC.
    pitch_offset = -(GAIN * e_y + KD * v_y), clipped and rate-limited.
    Emergency mode: when |e_y| > EMERGENCY_THRESHOLD, inject direct torque
    on hip forwards joints with hysteresis.

INPUTS (from shared_state):
    com_position, com_velocity, capture_point, foot positions,
    contact states, step_phase, stance_foot_world_pos

OUTPUTS (to shared_state):
    balance_hip_roll_left/right, sagittal_pitch_offset,
    emergency_sagittal_torque, diagnostics fields

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
import numpy as np
from typing import Dict

from shared_state import (
    shared_state,
    ContactState,
    StepPhase,
    URDF_JOINT_NAMES,
)


# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

G: float = 9.81  # m/s², gravitational acceleration

# ── Lateral axis gains ──────────────────────────────────────────────────────
LATERAL_ROLL_GAIN: float = 0.8      # rad/m, proportional gain on lateral CP error
LATERAL_KD: float = 0.1             # rad·s/m, derivative gain on lateral COM velocity
HIP_ROLL_MAX: float = 0.25          # rad (~14 deg), hard clip on hip roll position
HIP_ROLL_RATE_LIMIT: float = 0.03   # rad/cycle, max change per 100 Hz tick

# ── Sagittal axis gains ────────────────────────────────────────────────────
SAGITTAL_PITCH_GAIN: float = 1.2    # rad/m, proportional gain on sagittal CP error
SAGITTAL_KD: float = 0.3            # rad·s/m, derivative gain on sagittal COM velocity
PITCH_OFFSET_MAX: float = 0.15      # rad (~8.6 deg), hard clip on pitch offset
PITCH_RATE_LIMIT: float = 0.02      # rad/cycle, max change per 100 Hz tick

# ── Emergency sagittal recovery ─────────────────────────────────────────────
EMERGENCY_THRESHOLD: float = 0.08   # m, |e_y| above this activates emergency torque
EMERGENCY_KP: float = 40.0          # N·m/m, proportional gain on sagittal error
EMERGENCY_KD: float = 10.0          # N·m·s/m, derivative gain on sagittal velocity
EMERGENCY_TORQUE_MAX: float = 50.0  # N·m, hard clip on emergency torque per joint
EMERGENCY_HYSTERESIS: float = 0.02  # m, deadband below threshold before exiting emergency

# ── Safety bounds ───────────────────────────────────────────────────────────
COM_HEIGHT_MIN: float = 0.30  # m, minimum valid COM height for omega computation
COM_HEIGHT_MAX: float = 1.80  # m, maximum valid COM height for omega computation

# ── URDF joint name aliases ─────────────────────────────────────────────────
_L_HIP_FWD: str = URDF_JOINT_NAMES['L_HIP_FORWARDS']   # 'Left_Hip_Forwards'
_R_HIP_FWD: str = URDF_JOINT_NAMES['R_HIP_FORWARDS']   # 'Right_Hip_Fowards' (URDF typo)


# ============================================================================
# BALANCE CONTROLLER CLASS
# ============================================================================

class BalanceController:
    """
    Unified 2-axis balance controller using LIPM capture point.

    Lateral axis: CP error -> hip roll position targets.
    Sagittal axis: CP error -> pitch offset + emergency torque.
    """

    def __init__(self):
        self._prev_hip_roll_left: float = 0.0   # rad, previous cycle left hip roll
        self._prev_hip_roll_right: float = 0.0  # rad, previous cycle right hip roll
        self._prev_pitch_offset: float = 0.0    # rad, previous cycle pitch offset
        self._in_emergency: bool = False         # emergency sagittal mode flag

    # ------------------------------------------------------------------ #
    # PUBLIC                                                                #
    # ------------------------------------------------------------------ #

    def update(self) -> None:
        """Main entry — called once per 100 Hz cycle."""
        # Safety gate
        if shared_state.freeze_robot or shared_state.emergency_stop_triggered:
            self._write_zero_outputs()
            return

        # COM height validation
        h = shared_state.com_position[2]
        if not (COM_HEIGHT_MIN < h < COM_HEIGHT_MAX):
            self._write_zero_outputs()
            return

        # LIPM natural frequency
        omega = math.sqrt(G / h)  # rad/s

        # Capture point (read from shared_state, written by stability.py)
        cp = shared_state.capture_point  # 2D array [cp_x, cp_y]

        # Support centers
        lat_center = self._lateral_support_center()
        sag_center = self._sagittal_support_center()

        # Errors
        e_x = cp[0] - lat_center   # m, lateral CP error
        e_y = cp[1] - sag_center   # m, sagittal CP error
        vx = shared_state.com_velocity[0]  # m/s, lateral COM velocity
        vy = shared_state.com_velocity[1]  # m/s, sagittal COM velocity

        # Write diagnostics
        shared_state.capture_point_error_lateral = float(e_x)
        shared_state.capture_point_error_sagittal = float(e_y)

        # Axis controllers
        self._update_lateral(e_x, vx)
        self._update_sagittal(e_y, vy)

    def _update_lateral(self, e_x: float, vx: float) -> None:
        """Compute hip roll targets from lateral CP error."""
        # PD control: negative feedback to correct lateral lean
        target_roll = -(LATERAL_ROLL_GAIN * e_x + LATERAL_KD * vx)  # rad

        # Hard clip
        target_roll = max(-HIP_ROLL_MAX, min(HIP_ROLL_MAX, target_roll))

        # Determine walking vs standing
        walking_phases = (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )
        is_walking = shared_state.step_phase in walking_phases

        if is_walking:
            # Walking: stance leg gets full target, swing leg gets opposing half
            if shared_state.stance_side == "right":
                raw_right = target_roll
                raw_left = -target_roll * 0.5
            else:
                raw_left = target_roll
                raw_right = -target_roll * 0.5
        else:
            # Standing: distribute equally
            raw_left = target_roll * 0.5
            raw_right = target_roll * 0.5

        # Rate-limit each leg independently
        left_roll = self._rate_limit(raw_left, self._prev_hip_roll_left, HIP_ROLL_RATE_LIMIT)
        right_roll = self._rate_limit(raw_right, self._prev_hip_roll_right, HIP_ROLL_RATE_LIMIT)

        # Store for next cycle
        self._prev_hip_roll_left = left_roll
        self._prev_hip_roll_right = right_roll

        # Write to shared_state
        shared_state.balance_hip_roll_left = left_roll
        shared_state.balance_hip_roll_right = right_roll
        shared_state.balance_mode_lateral = "ACTIVE" if abs(e_x) > 1e-6 else "INACTIVE"

    def _update_sagittal(self, e_y: float, vy: float) -> None:
        """Compute pitch offset and emergency torque from sagittal CP error."""
        # PD pitch offset
        raw_offset = -(SAGITTAL_PITCH_GAIN * e_y + SAGITTAL_KD * vy)  # rad

        # Hard clip
        raw_offset = max(-PITCH_OFFSET_MAX, min(PITCH_OFFSET_MAX, raw_offset))

        # Rate-limit
        pitch_offset = self._rate_limit(raw_offset, self._prev_pitch_offset, PITCH_RATE_LIMIT)
        self._prev_pitch_offset = pitch_offset

        # Write pitch offset
        shared_state.sagittal_pitch_offset = pitch_offset

        # Emergency mode with hysteresis
        abs_ey = abs(e_y)
        if not self._in_emergency and abs_ey > EMERGENCY_THRESHOLD:
            self._in_emergency = True
        elif self._in_emergency and abs_ey < (EMERGENCY_THRESHOLD - EMERGENCY_HYSTERESIS):
            self._in_emergency = False

        if self._in_emergency:
            self._apply_emergency_torque(e_y, vy)
            shared_state.balance_mode_sagittal = "EMERGENCY"
        else:
            shared_state.emergency_sagittal_torque = {}
            shared_state.balance_mode_sagittal = "NORMAL" if abs_ey > 1e-6 else "INACTIVE"

    def _apply_emergency_torque(self, e_y: float, vy: float) -> None:
        """Inject direct hip-pitch torque for sagittal emergency recovery."""
        raw_torque = -(EMERGENCY_KP * e_y + EMERGENCY_KD * vy)  # N·m
        raw_torque = max(-EMERGENCY_TORQUE_MAX, min(EMERGENCY_TORQUE_MAX, raw_torque))

        # Split by contact state
        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED

        if l_conf and r_conf:
            l_share = 0.5
            r_share = 0.5
        elif l_conf:
            l_share = 1.0
            r_share = 0.0
        elif r_conf:
            l_share = 0.0
            r_share = 1.0
        else:
            l_share = 0.5
            r_share = 0.5

        shared_state.emergency_sagittal_torque = {
            _L_HIP_FWD: float(raw_torque * l_share),  # N·m
            _R_HIP_FWD: float(raw_torque * r_share),  # N·m
        }

    # ------------------------------------------------------------------ #
    # SUPPORT CENTER HELPERS                                                #
    # ------------------------------------------------------------------ #

    def _lateral_support_center(self) -> float:
        """Lateral (X) support center: stance foot during walking, midpoint when standing."""
        walking_phases = (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )
        if shared_state.step_phase in walking_phases:
            return float(shared_state.stance_foot_world_pos[0])

        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (shared_state.left_foot_position[0]
                    + shared_state.right_foot_position[0]) / 2.0
        elif l_conf:
            return float(shared_state.left_foot_position[0])
        elif r_conf:
            return float(shared_state.right_foot_position[0])
        return 0.0

    def _sagittal_support_center(self) -> float:
        """Sagittal (Y) support center: stance foot during walking, midpoint when standing."""
        walking_phases = (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )
        if shared_state.step_phase in walking_phases:
            return float(shared_state.stance_foot_world_pos[1])

        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (shared_state.left_foot_position[1]
                    + shared_state.right_foot_position[1]) / 2.0
        elif l_conf:
            return float(shared_state.left_foot_position[1])
        elif r_conf:
            return float(shared_state.right_foot_position[1])
        return 0.0

    # ------------------------------------------------------------------ #
    # UTILITY                                                               #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _rate_limit(target: float, prev: float, limit: float) -> float:
        """Clamp the change from prev to target within [-limit, +limit]."""
        delta = target - prev
        delta = max(-limit, min(limit, delta))
        return prev + delta

    def _write_zero_outputs(self) -> None:
        """Zero all balance outputs in shared_state."""
        shared_state.balance_hip_roll_left = 0.0
        shared_state.balance_hip_roll_right = 0.0
        shared_state.sagittal_pitch_offset = 0.0
        shared_state.emergency_sagittal_torque = {}
        shared_state.capture_point_error_lateral = 0.0
        shared_state.capture_point_error_sagittal = 0.0
        shared_state.balance_mode_lateral = "INACTIVE"
        shared_state.balance_mode_sagittal = "INACTIVE"
        # Reset internal state so next activation starts clean
        self._prev_hip_roll_left = 0.0
        self._prev_hip_roll_right = 0.0
        self._prev_pitch_offset = 0.0
        self._in_emergency = False

    def reset(self) -> None:
        """Reset internal state and zero all outputs."""
        self._write_zero_outputs()

    def get_diagnostics(self) -> Dict:
        """Return a dict of balance controller diagnostics."""
        return {
            'lateral_error': shared_state.capture_point_error_lateral,
            'sagittal_error': shared_state.capture_point_error_sagittal,
            'mode_lateral': shared_state.balance_mode_lateral,
            'mode_sagittal': shared_state.balance_mode_sagittal,
            'hip_roll_left': shared_state.balance_hip_roll_left,
            'hip_roll_right': shared_state.balance_hip_roll_right,
            'pitch_offset': shared_state.sagittal_pitch_offset,
            'in_emergency': self._in_emergency,
        }


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

_controller = BalanceController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_balance() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _controller.update()


def reset_balance() -> None:
    """Reset the balance controller state."""
    _controller.reset()


def get_balance_diagnostics() -> Dict:
    """Return balance controller diagnostics dict."""
    return _controller.get_diagnostics()
