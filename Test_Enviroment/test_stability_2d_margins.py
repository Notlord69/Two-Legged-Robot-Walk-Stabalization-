"""Tests for 2D stability margin decomposition in stability.py."""
import pytest
import numpy as np
from shared_state import shared_state, ContactState
from stability import update_stability


def _setup_standing():
    """Configure shared_state for a stable standing scenario with a valid polygon.

    Both feet confirmed with enough contact points to form a convex polygon
    (need >= 3 unique 2D points for ConvexHull).
    """
    shared_state.reset()
    shared_state.com_position = np.array([0.0, 0.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])

    # Both feet confirmed
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED

    # Provide 4 contact points forming a square around origin in X-Y plane.
    # This gives a well-defined convex hull with nonzero area.
    shared_state.left_contact_points = [
        np.array([-0.05, -0.05, 0.0]),
        np.array([-0.05,  0.05, 0.0]),
    ]
    shared_state.right_contact_points = [
        np.array([ 0.05, -0.05, 0.0]),
        np.array([ 0.05,  0.05, 0.0]),
    ]

    shared_state.left_foot_position = np.array([-0.05, 0.0, 0.0])
    shared_state.right_foot_position = np.array([0.05, 0.0, 0.0])


def test_lateral_margin_written():
    """If stability_margin > 0, stability_margin_lateral > 0.0."""
    _setup_standing()
    update_stability(dt=0.01)
    assert shared_state.stability_margin > 0, "Setup must produce a stable polygon"
    assert shared_state.stability_margin_lateral > 0.0


def test_sagittal_margin_written():
    """If stability_margin > 0, stability_margin_sagittal > 0.0."""
    _setup_standing()
    update_stability(dt=0.01)
    assert shared_state.stability_margin > 0, "Setup must produce a stable polygon"
    assert shared_state.stability_margin_sagittal > 0.0


def test_margins_zero_when_unstable():
    """CP outside polygon results in both margins at 0.0."""
    from stability import stability_monitor

    _setup_standing()

    # Give the stability monitor a previous COM far from origin so the
    # computed velocity pushes the capture point well outside the polygon.
    stability_monitor.prev_com = np.array([-5.0, -5.0, 0.8806])
    stability_monitor.prev_time = 0.0

    update_stability(dt=0.01)
    assert shared_state.stability_margin_lateral == 0.0
    assert shared_state.stability_margin_sagittal == 0.0
