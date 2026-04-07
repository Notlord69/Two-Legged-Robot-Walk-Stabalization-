"""
================================================================================
PROJECT SICLO1 — GAIT PLANNER  (gait_planner.py)
================================================================================

Compute Capture-Point-adjusted foot targets, run the parabolic swing arc,
and call kinematics.solve_ik() to produce joint angle targets.

Foot target (at toe-off):
    x_target = capture_point_x * STEP_TIMING_SCALE + STEP_LENGTH
    (STEP_LENGTH halved in DECEL state to absorb stopping)

Swing trajectory (parabolic arc):
    z_swing = SWING_HEIGHT * 4 * φ * (1 - φ)    # peaks at φ=0.5
    x_swing = x_stance + (x_target - x_stance) * φ

Phase advance per cycle:
    shared_state.swing_phase += dt / SWING_DURATION

At φ ≥ 1.0: foot placed, step_count++, active_swing_side flips, phase resets.

IK call (no modifications to kinematics.py):
    foot_xyz_rel = (x_swing - hip_x, 0.0, z_swing - hip_z)
    angles = kinematics.solve_ik(foot_xyz_rel, side)

INPUTS (shared_state):
    com_position, com_velocity, capture_point, swing_phase, swing_foot_x_stance,
    left/right_foot_position, active_swing_side, mission_state, ramp_gain,
    freeze_robot, link_positions, last_dt

OUTPUTS (shared_state):
    swing_phase, swing_foot_x_stance, swing_foot_target, left/right_foot_target,
    ik_left_angles, ik_right_angles, step_count, active_swing_side

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import numpy as np

import kinematics
from shared_state import shared_state, MissionState


# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH:       float = 0.12   # m, fixed sagittal advance per step (nominal)
STEP_TIMING_SCALE: float = 0.5    # dimensionless, blend factor for CP correction

SWING_HEIGHT:   float = 0.04   # m, peak foot clearance above ground at φ=0.5
SWING_DURATION: float = 0.40   # s, full swing phase (40 cycles at 100 Hz)

# Hip link names in shared_state.link_positions (verified HeartBeat.py 2026-04-05)
_LEFT_HIP_LINK:  str = "Left_Upper_Leg_1"
_RIGHT_HIP_LINK: str = "Right_Upper_Leg_1"


# ============================================================================
# HELPERS
# ============================================================================

def _swing_z(phi: float) -> float:
    """Parabolic foot height during swing.

    phi: normalized swing phase ∈ [0, 1]
    Returns z_foot (m) above ground; 0 at start and end, SWING_HEIGHT at mid.
    """
    return SWING_HEIGHT * 4.0 * phi * (1.0 - phi)


def _compute_x_target(capture_point_x: float, decel: bool = False) -> float:
    """Compute foot landing x-position from Capture Point.

    capture_point_x: x-component of CP in world frame (m)
    decel: True in DECEL state → halve STEP_LENGTH to absorb stopping impulse
    Returns target x (m, world frame).
    """
    step = STEP_LENGTH * 0.5 if decel else STEP_LENGTH
    return capture_point_x * STEP_TIMING_SCALE + step


# ============================================================================
# GAIT PLANNER CONTROLLER
# ============================================================================

class GaitPlannerController:
    """Per-cycle gait planner. Reads/writes shared_state."""

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        # Safety / gate
        if (shared_state.freeze_robot or
                shared_state.mission_state == MissionState.IDLE):
            return

        dt = shared_state.last_dt
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01   # fallback to 100 Hz nominal

        # Which leg is swinging?
        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK

        # Hip position in world frame
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # Foot stance position (position at toe-off, stored when swing starts)
        # At phase == 0.0 (swing start), snapshot current swing foot position
        phi = shared_state.swing_phase
        if phi == 0.0:
            foot_pos = (shared_state.left_foot_position
                        if side == "left"
                        else shared_state.right_foot_position)
            shared_state.swing_foot_x_stance = float(foot_pos[0])

        x_stance = shared_state.swing_foot_x_stance

        # Compute foot target (at toe-off, captured once per swing cycle)
        # For simplicity, recompute each cycle — CP may update mid-swing
        cp_x = float(getattr(shared_state, 'capture_point', np.zeros(2))[0])
        decel = (shared_state.mission_state == MissionState.DECEL)
        x_target = _compute_x_target(cp_x, decel=decel)
        shared_state.swing_foot_target = (x_target, 0.0, 0.0)

        if side == "left":
            shared_state.left_foot_target  = (x_target, 0.0, 0.0)
        else:
            shared_state.right_foot_target = (x_target, 0.0, 0.0)

        # Advance swing phase
        phi += dt / SWING_DURATION
        shared_state.swing_phase = phi

        # Compute swing foot position along arc
        phi_clamped = min(phi, 1.0)
        x_swing     = x_stance + (x_target - x_stance) * phi_clamped
        z_swing     = _swing_z(phi_clamped)

        # IK: foot position relative to hip-pitch joint
        foot_xyz_rel = (x_swing - hip_x, 0.0, z_swing - hip_z)
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)

        if side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

        # Step completion: φ ≥ 1.0
        if phi >= 1.0:
            shared_state.step_count    += 1
            shared_state.swing_phase    = 0.0
            # Flip swing side
            shared_state.active_swing_side = (
                "right" if side == "left" else "left"
            )


_gait_planner = GaitPlannerController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_gait_planner() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _gait_planner.update()
