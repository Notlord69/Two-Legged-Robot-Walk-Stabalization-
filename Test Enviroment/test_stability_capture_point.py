"""Tests for Capture Point stability classification in stability.py.

No PyBullet required. All shared_state fields are set manually.
Tests verify LIPM Capture Point (CP = com_xy + v_com_xy / omega_n) is used
instead of the static COM for polygon containment.
"""
import math
import numpy as np
import pytest
from shared_state import shared_state, ContactState, StabilityStatus


def _reset_for_stability():
    """Set up shared_state for a standing robot with both feet confirmed.

    Contact points are set explicitly (2 per foot) to form a valid rectangle
    polygon with >= 3 non-collinear points.  Using only foot_position fallback
    gives 2 collinear points — Shapely cannot build a polygon from those.

    Polygon corners (X, Y):
        (-0.1, +0.05)  (-0.1, -0.05)   <- left foot (front, back)
        (+0.1, +0.05)  (+0.1, -0.05)   <- right foot (front, back)
    COM at (0, 0) is well inside this rectangle.
    """
    shared_state.reset()

    # Robot standing upright: COM at nominal height, no load
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])
    shared_state.link_positions = {}          # empty -> FK path used
    shared_state.joint_positions = {}         # empty -> FK produces no links -> COM = base_position
    shared_state.current_load_mass = 0.0

    # Both feet confirmed on the ground
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_foot_position  = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position = np.array([ 0.1, 0.0, 0.0])

    # Explicit contact points — 2 per foot, forming a valid convex rectangle.
    # get_confirmed_contact_points() uses these when the list is non-empty.
    shared_state.left_contact_points = [
        np.array([-0.1,  0.05, 0.0]),   # left foot, forward edge
        np.array([-0.1, -0.05, 0.0]),   # left foot, rear edge
    ]
    shared_state.right_contact_points = [
        np.array([ 0.1,  0.05, 0.0]),   # right foot, forward edge
        np.array([ 0.1, -0.05, 0.0]),   # right foot, rear edge
    ]

    # Reset stability_monitor.prev_com to None so compute_com_velocity()
    # returns zeros on the first call, regardless of earlier test suite state.
    # HeartBeat tests call step() which writes prev_com; without this reset
    # the finite-difference velocity is non-zero and CP drifts outside polygon.
    from stability import stability_monitor
    stability_monitor.prev_com = None


def test_stable_when_standing_still():
    """Both feet confirmed, zero velocity -> CP = COM -> inside polygon -> STABLE."""
    from stability import stability_monitor, update_stability

    _reset_for_stability()
    # Zero velocity: prime prev_com so first finite-difference gives v=0
    stability_monitor.prev_com = np.array([0.0, 0.0, 0.8806])

    status = update_stability(dt=0.01)

    assert status == StabilityStatus.STABLE, (
        f"Expected STABLE, got {status.name}. "
        f"CP should equal COM at zero velocity."
    )
    # capture_point should be written and equal to COM xy
    assert shared_state.capture_point.shape == (2,)
    assert abs(shared_state.capture_point[0]) < 0.01   # near zero x
    assert abs(shared_state.capture_point[1]) < 0.01   # near zero y


def test_unstable_when_high_lateral_velocity():
    """High lateral velocity pushes CP outside support polygon -> UNSTABLE.

    At z_com = 0.8806 m:
        omega_n = sqrt(9.81 / 0.8806) ≈ 3.338 rad/s
        v_y = 2.0 m/s  ->  CP_y = 0 + 2.0 / 3.338 ≈ 0.599 m

    Support polygon spans Y: [-0.05, +0.05] m (foot half-width from contact points).
    CP_y ≈ 0.599 >> 0.05 -> CP is well outside -> UNSTABLE.
    """
    from stability import stability_monitor, update_stability

    _reset_for_stability()

    # Simulate lateral velocity by setting prev_com offset:
    # com_now = [0, 0, 0.8806], prev_com shifted so v_y = 2.0 m/s over dt=0.01 s
    dt = 0.01
    v_y = 2.0  # m/s, lateral — pushes CP far outside +-0.05 m polygon
    stability_monitor.prev_com = np.array([0.0, -v_y * dt, 0.8806])

    status = update_stability(dt=dt)

    assert status == StabilityStatus.UNSTABLE, (
        f"Expected UNSTABLE, got {status.name}. "
        f"CP_y ≈ {v_y / math.sqrt(9.81 / 0.8806):.3f} m should be outside +-0.05 m polygon."
    )
    # CP should reflect the extrapolation
    omega_n = math.sqrt(9.81 / 0.8806)
    expected_cp_y = 0.0 + v_y / omega_n
    assert abs(shared_state.capture_point[1] - expected_cp_y) < 0.05


def test_unstable_when_no_confirmed_contacts():
    """No confirmed contacts -> no support polygon -> UNSTABLE (existing behaviour)."""
    from stability import update_stability
    from shared_state import ContactState

    shared_state.reset()
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])
    shared_state.link_positions = {}
    # Both feet explicitly NOT confirmed
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)

    status = update_stability(dt=0.01)

    assert status == StabilityStatus.UNSTABLE, (
        f"Expected UNSTABLE with no contact polygon, got {status.name}"
    )
