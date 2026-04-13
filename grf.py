"""
================================================================================
PROJECT SICLO1 — GROUND REACTION FORCE CONTROLLER  (grf.py)
================================================================================

Virtual Spring-Damper Fz + Jacobian Transpose torque corrections.

Model:
    F_z = K_SPRING * (Z_REST - z_foot) - B_DAMPER * z_dot_foot

Sagittal 2-link Jacobian (hip-pitch + knee, no ankle):
    ∂z/∂θ_hip  = -(L_thigh*sin(θ_hip) + L_shank*sin(θ_hip + θ_knee))
    ∂z/∂θ_knee = -L_shank*sin(θ_hip + θ_knee)
    τ = Jᵀ * [0, F_z]ᵀ

URDF axis sign convention:
    Left  hip/knee  axis = -X  →  θ_geo = -q_urdf  →  τ_urdf = -τ_geo
    Right hip/knee  axis = +X  →  θ_geo =  q_urdf  →  τ_urdf =  τ_geo

Output torques are additive corrections layered on top of active_balance
target_torques. Scaled by shared_state.ramp_gain. Only applied to stance
(CONTACT_CONFIRMED) feet. Applied only when mission_state != IDLE.

INPUTS (shared_state):
    left/right_foot_position, left/right_foot_velocity,
    joint_positions, left/right_foot_contact_state,
    ramp_gain, mission_state, freeze_robot, emergency_stop_triggered

OUTPUTS (shared_state):
    grf_torque_correction  — Dict[str, float], URDF joint name keys, N·m

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
from typing import Dict

from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StepPhase,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
)

# Phases where only the stance leg receives GRF.  During LIFT/SWING/PLACE the
# swing-foot contact sensor may still read CONTACT_CONFIRMED due to sensor lag;
# we suppress GRF on the swing leg regardless of sensor state.
_STANCE_ONLY_PHASES = {StepPhase.LIFT, StepPhase.SWING, StepPhase.PLACE}


# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

# Leg geometry — left leg is canonical per kinematics.py
L_THIGH: float = DEFAULT_LINK_DATA['l_thigh']['length']  # m, hip-pitch axis to knee pivot
L_SHANK: float = DEFAULT_LINK_DATA['l_shank']['length']  # m, knee pivot to ankle pivot

# Spring-damper constants (sized for 8.1 kg robot, 5 cm max compression):
#   K_SPRING = m*g / δ_max  =  8.1 * 9.81 / 0.05  ≈  1589 N/m
#   B_DAMPER = 2 * sqrt(K * m) * ζ, ζ=0.7 (under-critically-damped for impact absorption)
Z_REST:   float = 0.75     # m,     nominal standing leg length (hip to foot, vertical)
K_SPRING: float = 1589.0   # N/m,   supports 8.1 kg with max 5 cm compression
B_DAMPER: float = 94.0     # N·s/m, ζ=0.7 critical damping ratio for impact absorption

# DECEL boost: increase K_SPRING by 20% to absorb stopping impulse
DECEL_SPRING_BOOST: float = 1.2   # dimensionless, applied to K_SPRING in DECEL state

ROBOT_MASS:   float = 8.0                       # kg, total robot mass (URDF-derived)
GRAVITY_COMP: float = ROBOT_MASS * 9.81 / 2.0   # N·per·leg, feedforward at standing


# ============================================================================
# INTERNAL HELPERS
# ============================================================================

def _spring_damper_fz(leg_ext: float, z_dot_foot: float,
                      k_spring: float) -> float:
    """Compute desired vertical support force via virtual spring-damper.

    leg_ext   : z_hip - z_foot (m); equals Z_REST at nominal standing posture.
                Positive = leg extended; negative = impossible geometry.
    z_dot_foot: foot z velocity in world frame (m/s), positive = moving up
    k_spring  : spring constant (N/m) — may be boosted for DECEL
    Returns F_z (N), positive = upward support.

    At equilibrium (leg_ext == Z_REST): F_z = GRAVITY_COMP (half body-weight).
    Compression (leg_ext < Z_REST): F_z > GRAVITY_COMP (extra support).
    Extension  (leg_ext > Z_REST): F_z < GRAVITY_COMP (less support).
    """
    return GRAVITY_COMP + k_spring * (Z_REST - leg_ext) - B_DAMPER * z_dot_foot


def _jacobian_torques(fz: float,
                      theta_hip_geo: float,
                      theta_knee_geo: float) -> tuple:
    """Sagittal Jacobian transpose: map F_z to (τ_hip_geo, τ_knee_geo).

    Uses geometric angles (positive = forward flex, axis-sign-agnostic).
    Caller applies URDF sign convention to convert to URDF torques.

    τ_hip  = (∂z/∂θ_hip)  * Fz
           = -(L_thigh*sin(θ_hip) + L_shank*sin(θ_hip+θ_knee)) * Fz
    τ_knee = (∂z/∂θ_knee) * Fz
           = -L_shank*sin(θ_hip+θ_knee) * Fz
    """
    sum_angle = theta_hip_geo + theta_knee_geo
    dz_dhip  = -(L_THIGH * math.sin(theta_hip_geo) +
                 L_SHANK * math.sin(sum_angle))
    dz_dknee = -L_SHANK * math.sin(sum_angle)
    return dz_dhip * fz, dz_dknee * fz


def _clip(joint_name: str, value: float) -> float:
    """Clip torque to URDF effort limit for the named joint."""
    lim = URDF_JOINT_LIMITS.get(joint_name)
    if lim is None:
        return value
    e = lim['effort']
    return max(-e, min(e, value))


def _compute_leg_correction(
    z_foot: float,
    z_hip: float,
    z_dot_foot: float,
    q_hip_urdf: float,
    q_knee_urdf: float,
    urdf_sign: float,
    hip_key: str,
    knee_key: str,
    k_spring: float,
    ramp_gain: float,
) -> Dict[str, float]:
    """Compute GRF torque corrections for one leg.

    z_foot    : foot z in world frame (m), from getLinkState
    z_hip     : hip link z in world frame (m), from link_positions
    z_dot_foot: foot z velocity (m/s)
    urdf_sign : +1.0 for right (axis=+X), -1.0 for left (axis=-X).
    URDF → geometric: θ_geo = urdf_sign * q_urdf
    Geometric → URDF: τ_urdf = urdf_sign * τ_geo

    Returns {hip_key: τ, knee_key: τ} — URDF-signed, clipped, gain-scaled.
    """
    theta_hip_geo  = urdf_sign * q_hip_urdf
    theta_knee_geo = urdf_sign * q_knee_urdf

    leg_ext = z_hip - z_foot  # m, actual leg extension; Z_REST at nominal stance
    fz = _spring_damper_fz(leg_ext, z_dot_foot, k_spring)
    tau_hip_geo, tau_knee_geo = _jacobian_torques(fz, theta_hip_geo, theta_knee_geo)

    tau_hip_urdf  = urdf_sign * tau_hip_geo
    tau_knee_urdf = urdf_sign * tau_knee_geo

    return {
        hip_key:  _clip(hip_key,  tau_hip_urdf  * ramp_gain),
        knee_key: _clip(knee_key, tau_knee_urdf * ramp_gain),
    }


# Fallback z_hip when link_positions not yet populated (first cycle).
# Produces leg_ext = Z_REST → spring term = 0 → F_z = GRAVITY_COMP. Safe default.
_Z_HIP_DEFAULT: float = Z_REST   # m


# ============================================================================
# GRF CONTROLLER
# ============================================================================

class GRFController:
    """Stateless per-cycle GRF controller. All state lives in shared_state."""

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        zero = {
            'Left_Hip_Forwards': 0.0,
            'Left_Knee':         0.0,
            'Right_Hip_Fowards': 0.0,
            'Right_Knee':        0.0,
        }

        # Safety gate: freeze, emergency stop, or gait not yet active
        if (shared_state.freeze_robot or
                shared_state.emergency_stop_triggered or
                shared_state.mission_state == MissionState.IDLE):
            shared_state.grf_torque_correction = zero
            return

        ramp_gain = shared_state.ramp_gain

        # DECEL: boost K_SPRING by 20% to absorb stopping impulse
        k_spring = (K_SPRING * DECEL_SPRING_BOOST
                    if shared_state.mission_state == MissionState.DECEL
                    else K_SPRING)

        jp = shared_state.joint_positions

        lp = shared_state.link_positions
        z_hip_left  = float(lp['Left_Upper_Leg_1'][2])  if 'Left_Upper_Leg_1'  in lp else _Z_HIP_DEFAULT
        z_hip_right = float(lp['Right_Upper_Leg_1'][2]) if 'Right_Upper_Leg_1' in lp else _Z_HIP_DEFAULT

        # Phase-aware eligibility: during LIFT/SWING/PLACE the swing leg is
        # suppressed even if its contact sensor still reads CONTACT_CONFIRMED.
        step_phase  = shared_state.step_phase
        stance_side = shared_state.stance_side
        left_eligible  = (step_phase not in _STANCE_ONLY_PHASES or stance_side == "left")
        right_eligible = (step_phase not in _STANCE_ONLY_PHASES or stance_side == "right")

        result: Dict[str, float] = {}

        # ── Left leg — axis = -X → urdf_sign = -1.0 ─────────────────────────
        if (left_eligible and
                shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED):
            result.update(_compute_leg_correction(
                z_foot      = float(shared_state.left_foot_position[2]),
                z_hip       = z_hip_left,
                z_dot_foot  = float(shared_state.left_foot_velocity[2]),
                q_hip_urdf  = jp.get('Left_Hip_Forwards', 0.0),
                q_knee_urdf = jp.get('Left_Knee', 0.0),
                urdf_sign   = -1.0,
                hip_key     = 'Left_Hip_Forwards',
                knee_key    = 'Left_Knee',
                k_spring    = k_spring,
                ramp_gain   = ramp_gain,
            ))
        else:
            result['Left_Hip_Forwards'] = 0.0
            result['Left_Knee']         = 0.0

        # ── Right leg — axis = +X → urdf_sign = +1.0 ────────────────────────
        if (right_eligible and
                shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED):
            result.update(_compute_leg_correction(
                z_foot      = float(shared_state.right_foot_position[2]),
                z_hip       = z_hip_right,
                z_dot_foot  = float(shared_state.right_foot_velocity[2]),
                q_hip_urdf  = jp.get('Right_Hip_Fowards', 0.0),
                q_knee_urdf = jp.get('Right_Knee', 0.0),
                urdf_sign   = +1.0,
                hip_key     = 'Right_Hip_Fowards',
                knee_key    = 'Right_Knee',
                k_spring    = k_spring,
                ramp_gain   = ramp_gain,
            ))
        else:
            result['Right_Hip_Fowards'] = 0.0
            result['Right_Knee']        = 0.0

        shared_state.grf_torque_correction = result


_grf_controller = GRFController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_grf() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _grf_controller.update()
