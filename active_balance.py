"""
================================================================================
PROJECT SICLO1 - ACTIVE BALANCE MODULE (Week 2)
================================================================================

Continuous Lateral COM Velocity Regulation via Layered Disturbance Strategy.

THEORY — Linear Inverted Pendulum Model (LIPM):
    The robot is modelled as a point mass at COM height h above the ankle pivot.
    Natural frequency:      ω₀ = √(g / h)
    Capture Point (CP):     x_cp = x_com + ẋ_com / ω₀
    If CP lies inside the support base → robot can recover without a step.

LAYERED TORQUE STRATEGY:
    ┌─────────────────────────────────────────────────────────────────┐
    │  |x_cp - x_center| < FOOT_WIDTH/4  →  ANKLE STRATEGY           │
    │      Small ankle torque re-centres COM.                         │
    │                                                                  │
    │  |x_cp - x_center| ≥ FOOT_WIDTH/4  →  HIP STRATEGY             │
    │  OR  lateral load acceleration spike detected                   │
    │      Hip torque shifts torso to catch the falling COM.          │
    │      5 kg backpack inertia (from URDF) is included.            │
    └─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  URDF SYNC  —  Siclo1.urdf                                                  │
│                                                                              │
│  Changes from pre-sync version                                               │
│  ─────────────────────────────                                               │
│  1. BASE_ROBOT_MASS  8.100127 kg  (sum of all URDF link masses)             │
│     Was: 29.0 kg (placeholder)                                               │
│                                                                              │
│  2. NOMINAL_COM_HEIGHT  0.8806 m  (URDF base_link inertial origin z)        │
│     Was: 1.00 m (THIGH + SHANK placeholder sum)                             │
│                                                                              │
│  3. LOAD_HEIGHT_ABOVE_COM  0.10 m  (backpack ~0.10 m above torso COM;       │
│     URDF has no separate payload link — derived from base_link geometry)    │
│     Was: 0.20 m                                                              │
│                                                                              │
│  4. J_TORSO_IYY  0.37776 kg·m²  (base_link <inertia iyy="0.37776"/>)       │
│     Replaces point-mass approximation BASE_ROBOT_MASS×h² in J_eff calc.    │
│     Iyy is the sagittal-plane (pitch) inertia — physically correct for      │
│     lateral hip torque in a planar LIPM.                                    │
│                                                                              │
│  5. FOOT_WIDTH  0.0286 m  (measured from URDF ankle Y positions:            │
│     Left ankle Y = 0.2250 m, Right ankle Y = 0.1964 m → Δ = 0.0286 m)    │
│     Was: 0.20 m  (was overestimated)                                        │
│                                                                              │
│  6. Output torque dict keys updated to URDF joint names:                    │
│       'Left_Ankle', 'Right_Ankle', 'Left_Hip_Forwards',                     │
│       'Right_Hip_Fowards'  (note URDF typo preserved)                       │
│     Were: 'l_ankle', 'r_ankle', 'l_hip', 'r_hip'                           │
└─────────────────────────────────────────────────────────────────────────────┘

INPUTS (from shared_state):
    com_position, com_velocity, current_load_mass, load_com_offset,
    left/right_foot_position, left/right_foot_contact_state,
    stability_status, last_dt, freeze_robot, emergency_stop_triggered

OUTPUTS (to shared_state):
    target_torques  — keys: 'Left_Ankle', 'Right_Ankle',
                             'Left_Hip_Forwards', 'Right_Hip_Fowards'
    capture_point, active_balance_mode, lateral_error

Author: Siclo1 Project Team
Date: February 2026 (URDF-synced March 2026)
================================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict
import warnings

from shared_state import (
    shared_state,
    Siclo1State,
    StabilityStatus,
    ContactState,
    DEFAULT_LINK_DATA,
    URDF_JOINT_NAMES,
)


# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

G: float = 9.81           # Gravitational acceleration (m/s²)
TARGET_DT: float = 0.01   # Nominal heartbeat period (s) — 100 Hz

JITTER_RATIO_THRESHOLD: float = 1.35   # 35% overrun triggers jitter compensation


# ============================================================================
# ROBOT GEOMETRY  —  URDF-SYNCED
# ============================================================================

# ── Segment lengths from URDF kinematic chain (neutral pose) ─────────────────
# Left  thigh: Left_Hip_Forwards → Left_Knee  |[0.031, 0.001, -0.052]| = 0.0607 m
# Left  shank: Left_Knee → Left_Ankle         |[0.100, 0.024, -0.679]| = 0.6870 m
# Right thigh: Right_Hip_Fowards → Right_Knee |[−0.106,−0.014,−0.009]| = 0.1072 m
# Right shank: Right_Knee → Right_Ankle       |[−0.025, 0.010,−0.759]| = 0.7592 m
_L_THIGH_LEN: float = DEFAULT_LINK_DATA['l_thigh']['length']   # 0.060661 m
_L_SHANK_LEN: float = DEFAULT_LINK_DATA['l_shank']['length']   # 0.686961 m

# NOMINAL_COM_HEIGHT: URDF base_link inertial origin z = 0.8806 m.
# This is the physical torso COM height and the correct LIPM pivot height.
NOMINAL_COM_HEIGHT: float = 0.8806   # metres (was 1.00 m)

# Foot lateral separation from URDF ankle Y-positions:
#   Left ankle Y ≈ 0.2250 m,  Right ankle Y ≈ 0.1964 m  →  Δ = 0.0286 m
# NOTE: this is narrower than the old 0.20 m default because the URDF legs
# are closely stacked.  The ankle strategy threshold is FOOT_WIDTH / 4.
FOOT_WIDTH: float = 0.0286   # metres (was 0.20 m)

# ── Masses  —  URDF-SYNCED ────────────────────────────────────────────────────
# Sum of all URDF <link><inertial><mass> values (no payload):
#   base_link 5.999 + left assembly 1.050 + right assembly 1.050 = 8.100 kg
BASE_ROBOT_MASS: float = 8.100127   # kg (was 29.0 kg)

# ── Backpack attachment height above torso COM  —  URDF-DERIVED ───────────────
# URDF has no separate payload link.  The backpack sits on top of base_link.
# base_link inertial origin z = 0.8806 m  (torso COM height).
# Backpack attachment ≈ torso_com_z + 0.10 m  (conservative shoulder estimate).
LOAD_HEIGHT_ABOVE_COM: float = 0.10   # metres (was 0.20 m)

# ── Torso inertia from URDF base_link  <inertia iyy="0.37776"/> ───────────────
# Iyy = sagittal-plane (pitch) inertia of the torso body about its own COM.
# Used in J_eff instead of the old point-mass approximation (BASE_ROBOT_MASS×h²).
# Old point-mass value was 8.10×0.88² ≈ 6.28 kg·m² — a 16× overestimate.
J_TORSO_IYY: float = 0.37776   # kg·m²  (base_link iyy from URDF)

# ── URDF joint name aliases for torque output dict keys ──────────────────────
_L_ANKLE_KEY:   str = URDF_JOINT_NAMES['L_ANKLE']       # 'Left_Ankle'
_R_ANKLE_KEY:   str = URDF_JOINT_NAMES['R_ANKLE']       # 'Right_Ankle'
_L_HIP_KEY:     str = URDF_JOINT_NAMES['L_HIP_FORWARDS'] # 'Left_Hip_Forwards'
_R_HIP_KEY:     str = URDF_JOINT_NAMES['R_HIP_FORWARDS'] # 'Right_Hip_Fowards'


# ============================================================================
# ACTIVE BALANCE CONFIGURATION
# ============================================================================

@dataclass
class ActiveBalanceConfig:
    """
    Tuning parameters for the active balance controller.
    All gains in SI units (torques N·m, lengths m).

    NOTE: kp_lateral, kd_lateral, and the torque limits are TUNING parameters
    — they are NOT derived from the URDF and may be adjusted freely.
    Only the geometric/inertial constants above are URDF-locked.
    """
    # PD gains — lateral velocity regulation
    kp_lateral: float = 40.0    # N·m/m
    kd_lateral: float = 8.0     # N·m·s/m

    jitter_damping_boost: float = 1.6

    # Ankle strategy
    ankle_torque_limit: float = 30.0   # N·m  (URDF Left_Ankle effort = 100 N·m — tuning is tighter)

    # Hip strategy
    hip_torque_limit: float = 60.0     # N·m  (URDF Left_Hip_Forwards effort = 100 N·m)
    hip_load_inertia_gain: float = 1.0

    lateral_accel_spike_threshold: float = 0.8   # m/s²
    strategy_divisor: float = 4.0

    com_height_min: float = 0.30
    com_height_max: float = 1.80

    split_torque_bilateral: bool = True


# ============================================================================
# ACTIVE BALANCE CONTROLLER
# ============================================================================

class ActiveBalanceController:
    """
    Continuous lateral COM velocity regulator for Siclo1.
    Physics-based torque controller using LIPM + layered ankle/hip strategy.
    """

    def __init__(self, config: ActiveBalanceConfig = None):
        self.config = config or ActiveBalanceConfig()
        self._prev_com_vel_lateral: float = 0.0
        self._prev_sim_time: float = 0.0
        self.capture_point: np.ndarray = np.zeros(2)
        self.lateral_error: float = 0.0
        self.active_mode: str = "INACTIVE"
        self.cycle_omega: float = 0.0

    # ------------------------------------------------------------------ #
    # PUBLIC API                                                           #
    # ------------------------------------------------------------------ #

    def update(self) -> Dict[str, float]:
        """
        Main entry point — called once per 100 Hz cycle.

        Returns:
            target_torques dict with URDF joint name keys:
                'Left_Ankle', 'Right_Ankle',
                'Left_Hip_Forwards', 'Right_Hip_Fowards'
        """
        zero_torques = self._zero_torques()

        # ── Safety gate ───────────────────────────────────────────────────────
        if shared_state.freeze_robot or shared_state.emergency_stop_triggered:
            self._write_to_shared_state(zero_torques, np.zeros(2), 0.0, "INACTIVE")
            return zero_torques

        # ── dt and jitter ─────────────────────────────────────────────────────
        dt = shared_state.last_dt
        if dt <= 0.0 or dt > 0.5:
            dt = TARGET_DT
        jitter_detected = (dt > TARGET_DT * JITTER_RATIO_THRESHOLD)

        # ── Read COM (written by stability.py) ───────────────────────────────
        com_pos = shared_state.com_position.copy()
        com_vel = shared_state.com_velocity.copy()

        h = com_pos[2]
        if not (self.config.com_height_min < h < self.config.com_height_max):
            self._write_to_shared_state(zero_torques, np.zeros(2), 0.0, "INACTIVE")
            return zero_torques

        # ── 1. LIPM natural frequency ─────────────────────────────────────────
        omega = np.sqrt(G / h)
        self.cycle_omega = omega

        # ── 2. Capture Point (lateral — X axis) ──────────────────────────────
        x_com = com_pos[0]
        xdot  = com_vel[0]
        x_cp  = x_com + xdot / omega
        cp_2d = np.array([x_cp, com_pos[1]])
        self.capture_point = cp_2d

        # ── 3. Support centre ─────────────────────────────────────────────────
        x_center = self._lateral_support_center()

        # ── 4. Lateral error ──────────────────────────────────────────────────
        lateral_error = x_cp - x_center
        self.lateral_error = lateral_error

        # ── 5. Lateral acceleration (spike detection) ─────────────────────────
        lat_accel = self._estimate_lateral_accel(xdot, dt)

        # ── 6. PD gains with optional jitter boost ────────────────────────────
        kp = self.config.kp_lateral
        kd = self.config.kd_lateral
        if jitter_detected:
            kd *= self.config.jitter_damping_boost

        # ── 7. Strategy selection ─────────────────────────────────────────────
        ankle_threshold = FOOT_WIDTH / self.config.strategy_divisor
        accel_spike     = abs(lat_accel) > self.config.lateral_accel_spike_threshold
        large_error     = abs(lateral_error) >= ankle_threshold

        if not large_error and not accel_spike:
            torques = self._ankle_strategy(lateral_error, xdot, kp, kd)
            mode = "ANKLE"
        elif large_error and not accel_spike:
            torques = self._hip_strategy(lateral_error, xdot, kp, kd, lat_accel)
            mode = "HIP"
        else:
            ankle_t = self._ankle_strategy(lateral_error, xdot, kp * 0.5, kd * 0.5)
            hip_t   = self._hip_strategy(lateral_error, xdot, kp, kd, lat_accel)
            torques = {k: ankle_t.get(k, 0.0) + hip_t.get(k, 0.0)
                       for k in (_L_ANKLE_KEY, _R_ANKLE_KEY, _L_HIP_KEY, _R_HIP_KEY)}
            mode = "COMBINED"

        # ── 8. Write results ──────────────────────────────────────────────────
        self._write_to_shared_state(torques, cp_2d, lateral_error, mode)
        return torques

    # ------------------------------------------------------------------ #
    # ANKLE STRATEGY                                                       #
    # ------------------------------------------------------------------ #

    def _ankle_strategy(self, lateral_error, xdot, kp, kd) -> Dict[str, float]:
        """
        Restore lateral balance via ankle plantarflexion/dorsiflexion.

        Sign convention:
            Positive torque → pushes COM in +X (rightward).
            To correct rightward lean (positive error): apply negative torque.

        T_ankle = -(kp·lateral_error + kd·ẋ)
        """
        raw_torque = -(kp * lateral_error + kd * xdot)
        l_share, r_share = self._torque_shares()
        limit = self.config.ankle_torque_limit

        return {
            _L_ANKLE_KEY: float(np.clip(raw_torque * l_share, -limit, limit)),
            _R_ANKLE_KEY: float(np.clip(raw_torque * r_share, -limit, limit)),
            _L_HIP_KEY:   0.0,
            _R_HIP_KEY:   0.0,
        }

    # ------------------------------------------------------------------ #
    # HIP STRATEGY  —  URDF INERTIA INTEGRATION                           #
    # ------------------------------------------------------------------ #

    def _hip_strategy(self, lateral_error, xdot, kp, kd, lat_accel) -> Dict[str, float]:
        """
        Inject hip torque to accelerate torso toward the falling side.

        URDF INERTIA INTEGRATION
        ─────────────────────────
        Old formula used a point-mass approximation:
            J_robot = BASE_ROBOT_MASS × h²   (≈ 6.28 kg·m² at h=0.88 m)
        This over-estimated J_robot by ~16× and under-estimated the inertia
        factor, leading to excessive hip torques.

        Updated formula uses the URDF base_link Iyy tensor directly:
            J_robot = J_TORSO_IYY = 0.37776 kg·m²   (from <inertia iyy="0.37776"/>)

        The 5 kg backpack adds rotational inertia about the torso COM:
            J_load = m_load × (h + LOAD_HEIGHT_ABOVE_COM)²
            where LOAD_HEIGHT_ABOVE_COM = 0.10 m  (0.10 m above torso COM)

        Effective inertia factor (normalised, dimensionless ≥ 1):
            J_eff = J_TORSO_IYY + hip_load_inertia_gain × J_load
            inertia_factor = J_eff / max(J_TORSO_IYY, 1e-6)

        The total hip torque is divided by inertia_factor so the hip does
        not over-swing when the payload is present.
        """
        m_load = shared_state.current_load_mass
        h      = shared_state.com_position[2]

        # URDF-sourced torso inertia (sagittal-plane, about torso COM)
        J_robot = J_TORSO_IYY                                            # 0.37776 kg·m²

        # Load contribution: backpack at (h + LOAD_HEIGHT_ABOVE_COM) from ankle
        J_load  = m_load * (h + LOAD_HEIGHT_ABOVE_COM) ** 2             # kg·m²

        J_eff   = J_robot + self.config.hip_load_inertia_gain * J_load
        inertia_factor = J_eff / max(J_robot, 1e-6)                     # ≥ 1.0

        # PD torque
        T_pd = -(kp * lateral_error + kd * xdot)

        # Feedforward load reaction (counter the backpack's lateral inertia)
        T_load_reaction = -m_load * lat_accel * LOAD_HEIGHT_ABOVE_COM

        # Combined, scaled by effective inertia
        T_total = (T_pd + T_load_reaction) / inertia_factor

        l_share, r_share = self._torque_shares()
        limit = self.config.hip_torque_limit

        return {
            _L_ANKLE_KEY: 0.0,
            _R_ANKLE_KEY: 0.0,
            _L_HIP_KEY:   float(np.clip(T_total * l_share, -limit, limit)),
            _R_HIP_KEY:   float(np.clip(T_total * r_share, -limit, limit)),
        }

    # ------------------------------------------------------------------ #
    # HELPERS                                                              #
    # ------------------------------------------------------------------ #

    def _lateral_support_center(self) -> float:
        l_conf = shared_state.left_foot_contact_state  == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (shared_state.left_foot_position[0] +
                    shared_state.right_foot_position[0]) / 2.0
        elif l_conf:
            return float(shared_state.left_foot_position[0])
        elif r_conf:
            return float(shared_state.right_foot_position[0])
        else:
            return 0.0

    def _torque_shares(self):
        l_conf = shared_state.left_foot_contact_state  == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (0.5, 0.5) if self.config.split_torque_bilateral else (0.5, 0.5)
        elif l_conf:
            return 1.0, 0.0
        elif r_conf:
            return 0.0, 1.0
        else:
            return 0.5, 0.5

    def _estimate_lateral_accel(self, xdot: float, dt: float) -> float:
        if self._prev_sim_time == 0.0:
            accel = 0.0
        else:
            accel = (xdot - self._prev_com_vel_lateral) / max(dt, 1e-6)
        self._prev_com_vel_lateral = xdot
        self._prev_sim_time        = shared_state.sim_time
        return accel

    def _zero_torques(self) -> Dict[str, float]:
        return {_L_ANKLE_KEY: 0.0, _R_ANKLE_KEY: 0.0,
                _L_HIP_KEY:   0.0, _R_HIP_KEY:   0.0}

    def _write_to_shared_state(self, torques, cp, lateral_error, mode):
        shared_state.target_torques      = torques
        # Do NOT overwrite shared_state.capture_point here.
        # stability.py (step 4 in pipeline) writes the authoritative 2D CP using
        # both X and Y velocity components.  active_balance only computed the X
        # component correctly (Y was set to com_pos[1] rather than com_y + vy/ω).
        # Preserving stability's value ensures gait_planner reads the correct CP.
        self.capture_point               = cp   # instance attribute — used for diagnostics
        shared_state.lateral_error       = lateral_error
        self.active_mode                 = mode
        shared_state.active_balance_mode = mode

    # ------------------------------------------------------------------ #
    # DIAGNOSTICS                                                          #
    # ------------------------------------------------------------------ #

    def get_diagnostics(self) -> dict:
        return {
            'mode':          self.active_mode,
            'omega_0':       round(self.cycle_omega, 4),
            'capture_point': self.capture_point.tolist(),
            'lateral_error': round(self.lateral_error, 5),
            'torques':       {k: round(v, 3)
                              for k, v in shared_state.target_torques.items()}
                             if hasattr(shared_state, 'target_torques') else {},
            'load_mass':     shared_state.current_load_mass,
            'jitter_active': shared_state.last_dt > TARGET_DT * JITTER_RATIO_THRESHOLD,
        }

    def print_diagnostics(self):
        d = self.get_diagnostics()
        print(f"\n[ACTIVE BALANCE @ t={shared_state.sim_time:.3f}s]")
        print(f"  Mode        : {d['mode']}")
        print(f"  ω₀          : {d['omega_0']:.3f} rad/s")
        print(f"  Capture Pt  : [{d['capture_point'][0]:.4f}, {d['capture_point'][1]:.4f}] m")
        print(f"  Lat. Error  : {d['lateral_error']:+.4f} m")
        print(f"  Torques     : {_L_ANKLE_KEY}={d['torques'].get(_L_ANKLE_KEY,0):+.1f}  "
              f"{_R_ANKLE_KEY}={d['torques'].get(_R_ANKLE_KEY,0):+.1f}  "
              f"{_L_HIP_KEY}={d['torques'].get(_L_HIP_KEY,0):+.1f}  "
              f"{_R_HIP_KEY}={d['torques'].get(_R_HIP_KEY,0):+.1f}  N·m")
        print(f"  Load        : {d['load_mass']:.1f} kg  "
              f"(J_TORSO_IYY={J_TORSO_IYY}, LOAD_H={LOAD_HEIGHT_ABOVE_COM})")
        print(f"  Jitter comp : {'YES' if d['jitter_active'] else 'no'}")


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

active_balance_controller = ActiveBalanceController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_active_balance() -> Dict[str, float]:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    return active_balance_controller.update()


def set_active_balance_config(config: ActiveBalanceConfig):
    active_balance_controller.config = config


def get_active_balance_diagnostics() -> dict:
    return active_balance_controller.get_diagnostics()


def reset_active_balance():
    active_balance_controller._prev_com_vel_lateral = 0.0
    active_balance_controller._prev_sim_time        = 0.0


# ============================================================================
# SELF-TEST
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("ACTIVE BALANCE MODULE — SELF TEST  (URDF-synced)")
    print("=" * 70)
    print(f"  BASE_ROBOT_MASS       = {BASE_ROBOT_MASS} kg  (was 29.0)")
    print(f"  NOMINAL_COM_HEIGHT    = {NOMINAL_COM_HEIGHT} m   (was 1.00)")
    print(f"  LOAD_HEIGHT_ABOVE_COM = {LOAD_HEIGHT_ABOVE_COM} m   (was 0.20)")
    print(f"  J_TORSO_IYY           = {J_TORSO_IYY} kg·m² (was point-mass)")
    print(f"  FOOT_WIDTH            = {FOOT_WIDTH} m (was 0.20)")
    print(f"  Torque keys: {_L_ANKLE_KEY}, {_R_ANKLE_KEY}, {_L_HIP_KEY}, {_R_HIP_KEY}")

    shared_state.reset()
    shared_state.base_position  = np.array([0.0, 0.0, NOMINAL_COM_HEIGHT])
    shared_state.com_position   = np.array([0.0, 0.0, NOMINAL_COM_HEIGHT])
    shared_state.com_velocity   = np.zeros(3)
    shared_state.current_load_mass = 0.0
    shared_state.last_dt        = TARGET_DT
    shared_state.sim_time       = 0.0

    shared_state.left_foot_position  = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position = np.array([ 0.1, 0.0, 0.0])
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)

    print("\nTest 1: Balanced — expect ANKLE mode, near-zero torques")
    torques = update_active_balance()
    active_balance_controller.print_diagnostics()
    assert active_balance_controller.active_mode == "ANKLE"
    assert abs(torques[_L_ANKLE_KEY]) < 1.0
    print("  ✓")

    print("\nTest 2: Small lean (+0.02 m) — expect ANKLE mode")
    shared_state.com_position = np.array([0.02, 0.0, NOMINAL_COM_HEIGHT])
    torques = update_active_balance()
    active_balance_controller.print_diagnostics()
    assert active_balance_controller.active_mode == "ANKLE"
    assert torques[_L_ANKLE_KEY] < 0.0
    print("  ✓")

    print("\nTest 3: Large lean (+0.08 m) — expect HIP / COMBINED mode")
    shared_state.com_position = np.array([0.08, 0.0, NOMINAL_COM_HEIGHT])
    shared_state.com_velocity = np.array([0.20, 0.0, 0.0])
    torques = update_active_balance()
    active_balance_controller.print_diagnostics()
    assert active_balance_controller.active_mode in ("HIP", "COMBINED")
    assert torques[_L_HIP_KEY] < 0.0
    print("  ✓")

    print("\nTest 4: 5 kg backpack — expect REDUCED hip torque (higher J_eff)")
    shared_state.current_load_mass = 5.0
    torques_loaded = update_active_balance()
    shared_state.current_load_mass = 0.0
    torques_unloaded = update_active_balance()
    assert abs(torques_loaded[_L_HIP_KEY]) <= abs(torques_unloaded[_L_HIP_KEY]) + 1e-6
    print(f"  Unloaded hip: {torques_unloaded[_L_HIP_KEY]:+.2f} N·m  "
          f"Loaded hip: {torques_loaded[_L_HIP_KEY]:+.2f} N·m  ✓")

    print("\nTest 5: Jitter compensation (last_dt = 18 ms)")
    shared_state.com_position = np.array([0.015, 0.0, NOMINAL_COM_HEIGHT])
    shared_state.com_velocity = np.array([0.05, 0.0, 0.0])
    shared_state.last_dt = 0.018
    torques_jitter = update_active_balance()
    shared_state.last_dt = TARGET_DT
    torques_normal = update_active_balance()
    assert abs(torques_jitter[_L_ANKLE_KEY]) >= abs(torques_normal[_L_ANKLE_KEY]) - 1e-6
    print("  ✓ Jitter compensation active")

    print("\nTest 6: freeze_robot — expect zero torques")
    shared_state.freeze_robot = True
    torques = update_active_balance()
    assert all(v == 0.0 for v in torques.values())
    shared_state.freeze_robot = False
    print("  ✓ Freeze gate working")

    print("\n" + "=" * 70)
    print("✓  ALL ACTIVE BALANCE TESTS PASSED  (URDF-synced)")
    print("=" * 70)
