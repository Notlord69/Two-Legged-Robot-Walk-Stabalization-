"""Kinematics for Siclo1 bipedal robot.

Pure math module — no PyBullet import. All sim calls go through sim/interface.py.
Left leg is the canonical segment-length reference (verified 2026-04-04).

Axis-sign convention (URDF):
  Left  hip/knee axis = -X  →  return negated geometric angles
  Right hip/knee axis = +X  →  return geometric angles unchanged
Ankle axis ≈ -Z (yaw, not sagittal pitch) → always 0.0 from IK.

Geometric note on proportions:
  L_THIGH = 390mm, L_SHANK = 360mm (1.083:1, ITER-003/004 2026-05-04).
  Knee stays within +/-pi/2 when d >= sqrt(L_THIGH^2+L_SHANK^2) ~= 0.5307 m.
"""
import math

# -- Canonical segment lengths (Left leg, ITER-003/004 patched 2026-05-04) ----
L_THIGH            = 0.390000  # m, hip-pitch axis to knee pivot
L_SHANK            = 0.360000  # m, knee pivot to ankle pivot
SINGULARITY_BUFFER = 0.005     # m, workspace annulus margin (avoid lock/collapse)
R_MIN = abs(L_THIGH - L_SHANK) + SINGULARITY_BUFFER  # m, 0.035000 inner bound
R_MAX = L_THIGH + L_SHANK - SINGULARITY_BUFFER        # m, 0.745000 outer bound

SWING_HEIGHT = 0.04  # m, default foot clearance above ground during swing


def clamp_foot_target(x_f: float, z_f: float) -> tuple:
    """Radially clamp (x_f, z_f) to the reachable annulus [R_MIN, R_MAX].

    x_f: sagittal forward offset from hip-pitch joint (m, positive = forward)
    z_f: vertical offset from hip-pitch joint (m, negative = below hip)
    Returns clamped (x_f, z_f) -- direction preserved, magnitude bounded.
    """
    d = math.sqrt(x_f * x_f + z_f * z_f)
    if d < 1e-9:                          # degenerate: foot at hip origin
        return 0.0, -R_MIN
    if d < R_MIN:
        scale = R_MIN / d
        return x_f * scale, z_f * scale
    if d > R_MAX:
        scale = R_MAX / d
        return x_f * scale, z_f * scale
    return x_f, z_f


def solve_ik(foot_xyz: tuple, side: str) -> tuple:
    """2-link planar IK in the sagittal plane.

    foot_xyz: (x, y, z) foot target relative to hip-pitch joint (m, world frame).
              The y component is ignored -- sagittal-plane solver.
    side:     'left' or 'right'

    Returns (hip_pitch, knee, ankle) in rad, URDF-signed:
      hip_pitch  positive = leg swings forward  (Left negated, Right kept)
      knee       positive = flexion             (Left negated, Right kept)
      ankle      0.0 (URDF axis ~= -Z = yaw, not sagittal pitch)

    Knee stays within +/-pi/2 when foot distance d >= sqrt(L_THIGH^2+L_SHANK^2) ~= 0.5307 m.

    Raises ValueError if side is not 'left' or 'right'.
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    x_f, _y, z_f = foot_xyz                    # project to sagittal plane
    x_f, z_f = clamp_foot_target(x_f, z_f)
    d = math.sqrt(x_f * x_f + z_f * z_f)

    # -- Knee angle (law of cosines, interior angle at knee) ------------------
    # gamma = pi  -> leg fully extended (straight)
    # gamma < pi  -> knee bent
    cos_gamma = (L_THIGH ** 2 + L_SHANK ** 2 - d ** 2) / (2.0 * L_THIGH * L_SHANK)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))  # clamp for floating-point safety
    gamma = math.acos(cos_gamma)
    theta_knee_geo = math.pi - gamma            # 0 = straight, positive = flexion (rad)

    # -- Hip pitch angle ------------------------------------------------------
    # alpha: angle from straight-down vertical to the hip->foot direction.
    # beta:  triangle angle at the hip vertex.
    alpha = math.atan2(x_f, -z_f)              # rad, positive = foot forward of vertical
    cos_beta = (L_THIGH ** 2 + d ** 2 - L_SHANK ** 2) / (2.0 * L_THIGH * d)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)                  # rad, always >= 0
    theta_hip_geo = alpha - beta                # rad, positive = leg forward

    # -- Ankle: axis ~= -Z (yaw) -> neutral -----------------------------------
    theta_ankle = 0.0                           # rad, yaw set by gait planner separately

    # -- Apply URDF axis-sign rule --------------------------------------------
    if side == "left":                          # Left axis = -X -> negate geometric angles
        return -theta_hip_geo, -theta_knee_geo, theta_ankle
    return theta_hip_geo, theta_knee_geo, theta_ankle   # Right axis = +X -> keep as-is


def swing_trajectory(phi: float, x_start: float, x_end: float, H: float) -> tuple:
    """Cycloidal foot trajectory -- zero velocity at liftoff and touchdown.

    phi:     normalized phase [0, 1] (0 = liftoff, 1 = touchdown)
    x_start: foot X at liftoff (m, world frame)
    x_end:   foot X at touchdown (m, world frame)
    H:       maximum swing clearance height (m, e.g. SWING_HEIGHT = 0.04 m)

    Returns (x, z) foot position (m). z = 0 at phi=0 and phi=1 by construction.
    Raises ValueError if phi is outside [0, 1].
    """
    if not 0.0 <= phi <= 1.0:
        raise ValueError(f"phi must be in [0, 1], got {phi!r}")
    two_pi_phi = 2.0 * math.pi * phi
    x = x_start + (x_end - x_start) * (phi - math.sin(two_pi_phi) / (2.0 * math.pi))
    z = H * (1.0 - math.cos(two_pi_phi)) / 2.0
    return x, z


def angular_momentum_correction(delta_theta_hip_swing: float,
                                m_leg: float,
                                m_total: float) -> float:
    """Feedforward torso pitch correction for angular momentum during swing.

    Derived from conservation of angular momentum:
      delta_theta_torso = -(m_leg / m_total) * delta_theta_hip_swing

    delta_theta_hip_swing: swing hip deviation from neutral (rad)
    m_leg:   swing leg mass (kg)
    m_total: total robot mass (kg, 8.0 kg nominal)

    Returns delta_theta_torso (rad) -- apply as feedforward on torso pitch joint.
    Raises ValueError if m_total <= 0.
    """
    if m_total <= 0.0:
        raise ValueError(f"m_total must be positive, got {m_total!r}")
    return -(m_leg / m_total) * delta_theta_hip_swing  # rad, feedforward torso pitch
