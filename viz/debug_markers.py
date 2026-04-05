"""IK debug visualisation for Siclo1.

Draws in the PyBullet GUI client (not the physics client).
All p.* calls go through sim.interface — never import pybullet here.

Draws:
  - Sagittal-plane semicircle arcs at R_min and R_max for each hip (light blue)
  - Green lines: hip → actual foot position (always drawn)
  - Red lines:   hip → foot target (skipped when target is (0,0,0))

Call update() from _sync_gui() only — never from the 100 Hz physics loop.
"""
import math
from sim.interface import add_debug_line
from kinematics import R_MIN, R_MAX

_ARC_SEGS     = 18                   # line segments per semicircle arc
_ARC_COLOR    = [0.6, 0.85, 1.0]    # light blue
_ACTUAL_COLOR = [0.0, 1.0, 0.0]     # green — hip → actual foot
_TARGET_COLOR = [1.0, 0.0, 0.0]     # red   — hip → foot target
_ZERO_VEC     = (0.0, 0.0, 0.0)     # sentinel: no active target


def _arc_points(hip_pos: tuple, radius: float,
                n_segs: int) -> list[tuple[float, float, float]]:
    """Return n_segs+1 world-space points for a sagittal-plane semicircle.

    The arc spans theta in [-pi/2, +pi/2] around the downward vertical:
      x = hip_x + radius * sin(theta)   (positive = forward)
      y = hip_y                          (constant: sagittal plane)
      z = hip_z - radius * cos(theta)   (theta=0 => straight below hip)

    hip_pos: (x, y, z) world position of the hip-pitch joint (m)
    radius:  arc radius (m)
    n_segs:  number of line segments (returns n_segs+1 points)
    """
    hx, hy, hz = hip_pos
    pts = []
    for i in range(n_segs + 1):
        theta = -math.pi / 2.0 + math.pi * i / n_segs
        pts.append((
            hx + radius * math.sin(theta),
            hy,
            hz - radius * math.cos(theta),
        ))
    return pts


class DebugVisualizer:
    """Manages PyBullet debug lines for IK workspace visualisation.

    Instantiate once after the GUI client is connected.
    Call update() from _sync_gui() every render tick.
    """

    def __init__(self, physics_client: int):
        self._pc = physics_client
        # Arc segment IDs: 4 arcs × _ARC_SEGS segments each
        # Order: L_Rmin, L_Rmax, R_Rmin, R_Rmax
        self._arc_ids: list[int] = [-1] * (4 * _ARC_SEGS)
        # Life-limited vector IDs
        self._left_actual_id:  int = -1
        self._right_actual_id: int = -1
        self._left_target_id:  int = -1
        self._right_target_id: int = -1

    def update(self, state, left_hip: tuple, right_hip: tuple) -> None:
        """Redraw all debug geometry for this render tick.

        state:     Siclo1State (reads left/right_foot_position and _target)
        left_hip:  world pos of left  hip-pitch joint (m)
        right_hip: world pos of right hip-pitch joint (m)
        """
        self._update_annulus(left_hip, right_hip)
        self._left_actual_id  = self._draw_vector(
            left_hip,  state.left_foot_position,
            _ACTUAL_COLOR, self._left_actual_id)
        self._right_actual_id = self._draw_vector(
            right_hip, state.right_foot_position,
            _ACTUAL_COLOR, self._right_actual_id)
        if state.left_foot_target != _ZERO_VEC:
            self._left_target_id = self._draw_vector(
                left_hip,  state.left_foot_target,
                _TARGET_COLOR, self._left_target_id)
        if state.right_foot_target != _ZERO_VEC:
            self._right_target_id = self._draw_vector(
                right_hip, state.right_foot_target,
                _TARGET_COLOR, self._right_target_id)

    # ── private helpers ───────────────────────────────────────────────────────

    def _draw_vector(self, from_pos: tuple, to_pos: tuple,
                     color: list, old_id: int) -> int:
        """Draw or replace one debug line. Returns the new item ID."""
        return add_debug_line(
            from_pos, to_pos, color,
            width=2.0,
            replace_id=old_id,
            physics_client=self._pc,
        )

    def _update_annulus(self, left_hip: tuple, right_hip: tuple) -> None:
        """Redraw the four semicircle arcs following the robot's hips."""
        arcs = [
            (left_hip,  R_MIN, 0),
            (left_hip,  R_MAX, _ARC_SEGS),
            (right_hip, R_MIN, 2 * _ARC_SEGS),
            (right_hip, R_MAX, 3 * _ARC_SEGS),
        ]
        for hip, radius, offset in arcs:
            pts = _arc_points(hip, radius, _ARC_SEGS)
            for i in range(_ARC_SEGS):
                idx = offset + i
                self._arc_ids[idx] = add_debug_line(
                    pts[i], pts[i + 1],
                    _ARC_COLOR,
                    width=1.0,
                    replace_id=self._arc_ids[idx],
                    physics_client=self._pc,
                )
