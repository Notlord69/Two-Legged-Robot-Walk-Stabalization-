"""Tests for viz/debug_markers.py — arc geometry and life-limited lines."""
import math
import pytest
from unittest.mock import patch, MagicMock, call


# ── Arc geometry (pure function) ─────────────────────────────────────────────

def test_arc_points_returns_n_segs_plus_one():
    from viz.debug_markers import _arc_points
    pts = _arc_points((0.0, 0.0, 1.0), 0.5, 18)
    assert len(pts) == 19  # n_segs + 1 boundary points


def test_arc_midpoint_is_straight_below_hip():
    from viz.debug_markers import _arc_points
    hip = (1.0, 2.0, 3.0)
    r = 0.5
    pts = _arc_points(hip, r, 18)
    mid = pts[9]  # midpoint: theta=0, straight down
    assert abs(mid[0] - hip[0]) < 1e-9   # x = hip_x
    assert abs(mid[1] - hip[1]) < 1e-9   # y = hip_y (sagittal plane)
    assert abs(mid[2] - (hip[2] - r)) < 1e-9  # z = hip_z - r


def test_arc_endpoints_are_at_hip_height():
    from viz.debug_markers import _arc_points
    hip = (0.0, 0.0, 1.0)
    r = 0.5
    pts = _arc_points(hip, r, 18)
    # theta = -pi/2: cos(-pi/2) = 0, so z = hip_z - r*0 = hip_z
    assert abs(pts[0][2] - hip[2]) < 1e-6
    # theta = +pi/2: same
    assert abs(pts[-1][2] - hip[2]) < 1e-6


def test_arc_all_points_at_correct_radius():
    from viz.debug_markers import _arc_points
    hip = (1.0, 2.0, 3.0)
    r = 0.6313
    pts = _arc_points(hip, r, 18)
    for pt in pts:
        dx = pt[0] - hip[0]
        dz = pt[2] - hip[2]
        dist = math.sqrt(dx * dx + dz * dz)
        assert abs(dist - r) < 1e-9


def test_arc_y_coordinate_equals_hip_y():
    from viz.debug_markers import _arc_points
    hip = (0.0, 0.75, 1.0)
    pts = _arc_points(hip, 0.5, 18)
    for pt in pts:
        assert abs(pt[1] - hip[1]) < 1e-9


# ── Life-limited line behaviour ───────────────────────────────────────────────

def _make_state(left_target=(0.0, 0.0, 0.0), right_target=(0.0, 0.0, 0.0)):
    """Return a minimal mock shared state for DebugVisualizer.update()."""
    s = MagicMock()
    s.left_foot_position  = (0.1, 0.0, 0.1)
    s.right_foot_position = (0.1, 0.5, 0.1)
    s.left_foot_target    = left_target
    s.right_foot_target   = right_target
    return s


def test_green_line_uses_replace_id_on_second_update():
    """Second update must pass the ID from the first as replaceItemUniqueId."""
    with patch('pybullet.addUserDebugLine', return_value=10) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0, warmup_cycles=0)
        mock_add.reset_mock()
        mock_add.return_value = 55  # first update returns ID 55 for each line

        vis.update(_make_state(), (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        mock_add.reset_mock()
        mock_add.return_value = 99

        vis.update(_make_state(), (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        # Every green-line call in the second update must include replaceItemUniqueId=55
        green_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [0.0, 1.0, 0.0]
        ]
        assert len(green_calls) == 2  # left + right actual
        for c in green_calls:
            assert c[1].get('replaceItemUniqueId') == 55


def test_red_line_not_drawn_when_target_is_zero_vector():
    """Red lines must be skipped when foot target is (0,0,0)."""
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0)
        mock_add.reset_mock()

        vis.update(_make_state(left_target=(0.0, 0.0, 0.0),
                               right_target=(0.0, 0.0, 0.0)),
                   (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        red_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [1.0, 0.0, 0.0]
        ]
        assert len(red_calls) == 0


def test_red_line_drawn_when_target_is_nonzero():
    """Red line must be drawn when foot target is set."""
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0, warmup_cycles=0)
        mock_add.reset_mock()

        vis.update(_make_state(left_target=(0.05, 0.0, -0.72)),
                   (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        red_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [1.0, 0.0, 0.0]
        ]
        assert len(red_calls) == 1  # left leg only
