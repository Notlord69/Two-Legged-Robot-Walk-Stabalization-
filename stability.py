"""
================================================================================
PROJECT SICLO1 - STABILITY MODULE
================================================================================

Monitors Center of Mass (COM) relative to support polygon.
Integrates Week 1 load offset (5 kg variable payload).

┌─────────────────────────────────────────────────────────────────────────────┐
│  URDF SYNC  —  Siclo1.urdf                                                  │
│                                                                              │
│  Changes from pre-sync version                                               │
│  ─────────────────────────────                                               │
│  1. joint_angles keys now use URDF names via URDF_JOINT_NAMES:              │
│       'Left_Hip_Forwards'  instead of 'l_hip'                               │
│       'Left_Knee'          instead of 'l_knee'                              │
│       'Right_Hip_Fowards'  instead of 'r_hip'   (note URDF typo preserved) │
│       'Right_Knee'         instead of 'r_knee'                              │
│                                                                              │
│  2. Axis-sign correction in forward_kinematics_2d():                        │
│       Left  hip/knee axis = −X  →  sin(−θ) applied (negate angle)          │
│       Right hip/knee axis = +X  →  sin(+θ) unchanged                       │
│                                                                              │
│  3. DEFAULT_LINK_DATA now sourced from shared_state.DEFAULT_LINK_DATA       │
│     (masses and lengths extracted from URDF <inertial> and joint origins).  │
└─────────────────────────────────────────────────────────────────────────────┘

INPUTS (from shared_state):
- joint_positions  — keyed by URDF joint names
- link_positions
- current_load_mass
- confirmed contact points

OUTPUTS (to shared_state):
- com_position
- stability_status (STABLE/MARGINAL/UNSTABLE)
- stability_margin
- current_safety_margin

Author: Siclo1 Project Team
Date: February 2026 (URDF-synced March 2026)

================================================================================
"""

import math
import time
import numpy as np
from shapely.geometry import Point, Polygon
from scipy.spatial import ConvexHull
from typing import Dict, List, Optional

from shared_state import (
    shared_state,
    StabilityStatus,
    LoadConfig,
    DEFAULT_LINK_DATA,
    URDF_JOINT_NAMES,
)


# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

G: float = 9.81          # m/s², standard gravitational acceleration
Z_COM_MIN: float = 0.05  # m, floor guard for omega_n — prevents sqrt domain error
                          # (robot at/below floor is already UNSTABLE; guard avoids crash)


# ============================================================================
# FORWARD KINEMATICS  (2-D sagittal-plane, URDF-aware)
# ============================================================================

def forward_kinematics_2d(joint_angles: Dict[str, float],
                          link_data: Dict) -> Dict[str, np.ndarray]:
    """
    Compute link COM positions from joint angles (2D simplified model).

    COORDINATE FRAME CONVENTION
    ───────────────────────────
    All positions are in the sagittal (X-Z) plane:
        +X = robot forward direction
        +Z = upward

    URDF AXIS-SIGN NOTES (critical — do not remove)
    ────────────────────────────────────────────────
    Left_Hip_Forwards  axis xyz="-1 0 0"  →  positive joint angle rotates
        the thigh BACKWARD in URDF convention.  To get forward flexion the
        controller sends a POSITIVE angle but FK must apply sin(-θ).
        RULE: left-side hip & knee angles are negated before sin/cos.

    Right_Hip_Fowards  axis xyz="1 0 0"   →  positive joint angle = forward
        flex as expected.  NO negation needed on the right side.

    Args:
        joint_angles : {URDF_joint_name: angle_rad}
        link_data    : {link_key: {'mass', 'length', 'com_local'}}

    Returns:
        {link_key: position_2d [x, z]}
    """
    torso_base = np.array([0.0, link_data['torso']['length']
                            if 'torso' in link_data else 0.8806])
    positions = {}

    # ── Torso COM ─────────────────────────────────────────────────────────────
    if 'torso' in link_data:
        positions['torso'] = torso_base + np.array([
            0.0,
            link_data['torso']['length'] * (link_data['torso']['com_local'] - 0.5)
        ])

    # ── Left leg ──────────────────────────────────────────────────────────────
    L_HIP = URDF_JOINT_NAMES['L_HIP_FORWARDS']   # 'Left_Hip_Forwards'
    L_KNEE = URDF_JOINT_NAMES['L_KNEE']           # 'Left_Knee'

    if L_HIP in joint_angles and 'l_thigh' in link_data:
        hip = torso_base.copy()

        # Axis = -X  →  negate angle so positive θ = forward step
        th_l = -joint_angles[L_HIP]

        knee_l = hip + np.array([
            link_data['l_thigh']['length'] * np.sin(th_l),
            -link_data['l_thigh']['length'] * np.cos(th_l)
        ])
        positions['l_thigh'] = hip + (knee_l - hip) * link_data['l_thigh']['com_local']

        if L_KNEE in joint_angles and 'l_shank' in link_data:
            # Axis = -X  →  negate angle
            kn_l = -joint_angles[L_KNEE]
            ankle_l = knee_l + np.array([
                link_data['l_shank']['length'] * np.sin(kn_l),
                -link_data['l_shank']['length'] * np.cos(kn_l)
            ])
            positions['l_shank'] = knee_l + (ankle_l - knee_l) * link_data['l_shank']['com_local']

    # ── Right leg ─────────────────────────────────────────────────────────────
    R_HIP = URDF_JOINT_NAMES['R_HIP_FORWARDS']   # 'Right_Hip_Fowards' (URDF typo)
    R_KNEE = URDF_JOINT_NAMES['R_KNEE']           # 'Right_Knee'

    if R_HIP in joint_angles and 'r_thigh' in link_data:
        hip_r = torso_base.copy()

        # Axis = +X  →  positive θ = forward flex  (no negation)
        th_r = joint_angles[R_HIP]

        knee_r = hip_r + np.array([
            link_data['r_thigh']['length'] * np.sin(th_r),
            -link_data['r_thigh']['length'] * np.cos(th_r)
        ])
        positions['r_thigh'] = hip_r + (knee_r - hip_r) * link_data['r_thigh']['com_local']

        if R_KNEE in joint_angles and 'r_shank' in link_data:
            # Axis = +X  →  no negation
            kn_r = joint_angles[R_KNEE]
            ankle_r = knee_r + np.array([
                link_data['r_shank']['length'] * np.sin(kn_r),
                -link_data['r_shank']['length'] * np.cos(kn_r)
            ])
            positions['r_shank'] = knee_r + (ankle_r - knee_r) * link_data['r_shank']['com_local']

    return positions


# ============================================================================
# PROFILING HELPER  (diagnostic — remove when root cause is identified)
# ============================================================================

def _emit_profile(t0: float, t1: float, t2: float, t3: float, t4: float,
                  n_contacts: int) -> None:
    """Print a one-line timing breakdown when total exceeds the threshold.

    Stages:
        COM       : compute_com_with_load + compute_com_velocity   (t0→t1)
        contacts  : get_confirmed_contact_points + compute_safety_margin (t1→t2)
        hull      : compute_support_polygon (ConvexHull + Shapely buffer) (t2→t3)
        distance  : polygon.exterior.distance(cp_point)            (t3→t4)
        total     : t0→t4

    Output example (all µs):
        [STAB-PROFILE] t=0.083 total=16142µs | COM=210 contacts=12 hull=15800 dist=120 | pts=6
    """
    total_us = (t4 - t0) * 1e6
    if total_us < StabilityMonitor.PROFILE_THRESHOLD_US:
        return
    com_us  = (t1 - t0) * 1e6
    cnt_us  = (t2 - t1) * 1e6
    hull_us = (t3 - t2) * 1e6
    dist_us = (t4 - t3) * 1e6
    print(
        f"[STAB-PROFILE] t={shared_state.sim_time:.3f}s "
        f"total={total_us:.0f}µs | "
        f"COM={com_us:.0f} contacts={cnt_us:.0f} hull={hull_us:.0f} dist={dist_us:.0f} | "
        f"pts={n_contacts}"
    )


# ============================================================================
# STABILITY MONITOR CLASS
# ============================================================================

class StabilityMonitor:
    """
    Monitors robot stability by comparing COM to support polygon.
    Includes Week 1 dynamic load offset.
    """

    def __init__(self, load_config: LoadConfig, link_data: Dict = None):
        self.load_config = load_config
        self.link_data = link_data if link_data is not None else DEFAULT_LINK_DATA
        self.prev_com: Optional[np.ndarray] = None
        self.prev_time: float = 0.0

    def compute_com_with_load(self) -> np.ndarray:
        """
        Compute COM including variable load (Week 1 requirement).

        Reads from shared_state:
        - link_positions (or computes from joint_positions)
        - current_load_mass
        - base_position

        Returns:
            com_position (3D: x, y, z)
        """
        total_mass = 0.0
        com_weighted = np.zeros(3)

        if len(shared_state.link_positions) > 0:
            for link_name, position in shared_state.link_positions.items():
                if link_name in self.link_data:
                    mass = self.link_data[link_name]['mass']
                    com_weighted += mass * position
                    total_mass += mass
        else:
            positions_2d = forward_kinematics_2d(
                shared_state.joint_positions,
                self.link_data
            )
            for link_name, pos_2d in positions_2d.items():
                if link_name in self.link_data:
                    mass = self.link_data[link_name]['mass']
                    # Sagittal plane: [x, z] → 3D [x, 0, z]
                    pos_3d = np.array([pos_2d[0], 0.0, pos_2d[1]])
                    com_weighted += mass * pos_3d
                    total_mass += mass

        # Load contribution (0–5 kg backpack)
        if shared_state.current_load_mass > 0:
            load_position = shared_state.base_position + self.load_config.load_attachment_point
            com_weighted += shared_state.current_load_mass * load_position
            total_mass   += shared_state.current_load_mass

        if total_mass > 0:
            return com_weighted / total_mass
        else:
            return shared_state.base_position.copy()

    def compute_com_velocity(self, com: np.ndarray, dt: float) -> np.ndarray:
        if self.prev_com is not None and dt > 0:
            velocity = (com - self.prev_com) / dt
        else:
            velocity = np.zeros(3)
        self.prev_com = com.copy()
        return velocity

    def compute_safety_margin(self) -> float:
        """
        Calculate load-dependent safety margin.

        Formula:
            load_ratio = current_load / max_load
            margin = nominal × (1 − load_ratio × (1 − scaling_factor))
        """
        load_ratio = shared_state.current_load_mass / self.load_config.max_load_mass
        margin_reduction = load_ratio * (1.0 - self.load_config.margin_scaling_factor)
        adjusted_margin = self.load_config.nominal_margin * (1.0 - margin_reduction)
        return max(0.005, adjusted_margin)

    def compute_support_polygon(self, contact_points: List[np.ndarray],
                                margin: float) -> Optional[Polygon]:
        if len(contact_points) == 0:
            return Polygon()

        pts_2d = np.array([p[:2] for p in contact_points])

        if len(pts_2d) <= 2:
            hull_pts = pts_2d
        else:
            try:
                hull = ConvexHull(pts_2d)
                hull_pts = pts_2d[hull.vertices]
            except Exception:
                hull_pts = pts_2d

        if len(hull_pts) < 3:
            return None

        poly = Polygon(hull_pts)
        if margin > 0 and not poly.is_empty:
            poly = poly.buffer(-margin)
        return poly

    # Threshold above which per-stage timing is logged (µs).
    # Set to 0 to log every cycle; set to float('inf') to disable.
    PROFILE_THRESHOLD_US: float = 5000.0   # 5 ms — half the 10 ms budget

    def check_stability(self, dt: float) -> StabilityStatus:
        """
        Main stability check.

        1. Compute COM with load
        2. Get confirmed contact points
        3. Build load-dependent support polygon
        4. Classify STABLE / MARGINAL / UNSTABLE

        Writes to shared_state:
            com_position, com_velocity, stability_status,
            stability_margin, current_safety_margin, support_polygon

        PROFILING: when total time exceeds PROFILE_THRESHOLD_US, emits a
        one-line breakdown showing which stage dominates.  Remove or set
        PROFILE_THRESHOLD_US = float('inf') after root cause is identified.
        """
        _t0 = time.perf_counter()

        com = self.compute_com_with_load()
        com_vel = self.compute_com_velocity(com, dt)
        shared_state.com_position = com
        shared_state.com_velocity = com_vel

        _t1 = time.perf_counter()

        contact_points = shared_state.get_confirmed_contact_points()
        safety_margin = self.compute_safety_margin()
        shared_state.current_safety_margin = safety_margin

        _t2 = time.perf_counter()

        polygon = self.compute_support_polygon(contact_points, safety_margin)
        shared_state.support_polygon = polygon

        _t3 = time.perf_counter()

        if polygon is None or polygon.is_empty or polygon.area < 1e-6:
            shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=0.0)
            _emit_profile(_t0, _t1, _t2, _t3, _t3, len(contact_points))
            return StabilityStatus.UNSTABLE

        # Capture Point — LIPM extrapolated COM (Option A).
        # CP = com_xy + v_com_xy / omega_n
        # omega_n = sqrt(g / z_com): natural frequency of the inverted pendulum.
        # At v=0 (standing still), CP = COM -> degrades to static check.
        z_com   = max(com[2], Z_COM_MIN)                  # guard: robot at/below floor
        omega_n = math.sqrt(G / z_com)                    # rad/s
        cp_xy   = com[:2] + com_vel[:2] / omega_n         # 2D capture point (X, Y)
        shared_state.capture_point = cp_xy                 # write for gait_planner + diagnostics

        cp_point = Point(cp_xy[0], cp_xy[1])
        if polygon.contains(cp_point):
            margin_distance = polygon.exterior.distance(cp_point)
            _t4 = time.perf_counter()
            _emit_profile(_t0, _t1, _t2, _t3, _t4, len(contact_points))
            if margin_distance > safety_margin * 0.5:
                shared_state.set_stability_status(StabilityStatus.STABLE, margin=margin_distance)
                return StabilityStatus.STABLE
            else:
                shared_state.set_stability_status(StabilityStatus.MARGINAL, margin=margin_distance)
                return StabilityStatus.MARGINAL
        else:
            margin_distance = -polygon.exterior.distance(cp_point)
            _t4 = time.perf_counter()
            _emit_profile(_t0, _t1, _t2, _t3, _t4, len(contact_points))
            shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=margin_distance)
            return StabilityStatus.UNSTABLE

    def update(self, dt: float) -> StabilityStatus:
        return self.check_stability(dt)


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

stability_monitor = StabilityMonitor(LoadConfig())


# ============================================================================
# PUBLIC API
# ============================================================================

def update_stability(dt: float = 0.01) -> StabilityStatus:
    """Public function called by HeartBeat.py."""
    return stability_monitor.update(dt)


def set_link_data(link_data: Dict):
    stability_monitor.link_data = link_data


def set_load_config(config: LoadConfig):
    stability_monitor.load_config = config


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("STABILITY MODULE TEST  (URDF-synced)")
    print("=" * 70)

    from shared_state import ContactState, URDF_JOINT_NAMES

    # Use URDF joint names
    shared_state.joint_positions = {
        URDF_JOINT_NAMES['L_HIP_FORWARDS']: 0.0,
        URDF_JOINT_NAMES['L_KNEE']:         0.3,
        URDF_JOINT_NAMES['R_HIP_FORWARDS']: 0.0,
        URDF_JOINT_NAMES['R_KNEE']:         0.3,
    }

    shared_state.base_position = np.array([0.0, 0.0, 0.8806])

    shared_state.left_foot_position  = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position = np.array([ 0.1, 0.0, 0.0])
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)

    print("\nTest 1: No load")
    shared_state.current_load_mass = 0.0
    status = update_stability(dt=0.01)
    print(f"  Status: {status.name}")
    print(f"  COM:    {np.round(shared_state.com_position, 4)}")
    print(f"  Margin: {shared_state.stability_margin:.4f} m")
    print(f"  Safety margin: {shared_state.current_safety_margin:.4f} m")

    print("\nTest 2: 5 kg load")
    shared_state.current_load_mass = 5.0
    status = update_stability(dt=0.01)
    print(f"  Status: {status.name}")
    print(f"  COM:    {np.round(shared_state.com_position, 4)}")
    print(f"  Margin: {shared_state.stability_margin:.4f} m")
    print(f"  Safety margin: {shared_state.current_safety_margin:.4f} m")
    print(f"  (Safety margin should be smaller than no-load case)")

    print("\nTest 3: Single foot contact")
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)
    status = update_stability(dt=0.01)
    print(f"  Status: {status.name}")

    print("\n" + "=" * 70)
    print("✓ Stability module (URDF-synced) test complete")
    print("=" * 70)
