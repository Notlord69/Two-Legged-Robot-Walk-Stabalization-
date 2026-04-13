"""
Tests for contact tick decay (Fix 1).

Problem: HeartBeat.read_sensors() hard-resets contact_ticks to 0 on any
frame with force ≤ 5 N.  A single PyBullet GJK glitch destroys 3 cycles of
accumulated contact evidence, cycling the foot back to TOUCH_EXPECTED and
collapsing the support polygon.

Fix: Replace the hard reset with a decay:
    force > threshold  →  ticks + 1
    force ≤ threshold  →  max(0, ticks - 1)

A single glitch decrements by 1.  The foot needs 3 consecutive zero-force
frames to fully lose contact confidence.
"""
import numpy as np
import pytest

from HeartBeat import _update_contact_ticks
from shared_state import shared_state, ContactState, ContactConfig
from perception import FootContactFSM


# ─── Unit tests for _update_contact_ticks ─────────────────────────────────────

def test_force_above_threshold_increments_ticks():
    """Force > threshold → ticks grows by 1."""
    assert _update_contact_ticks(2, 10.0, 5.0) == 3


def test_zero_force_decrements_ticks_by_one():
    """Single zero-force frame → ticks drops by exactly 1 (not reset to 0)."""
    assert _update_contact_ticks(3, 0.0, 5.0) == 2


def test_decay_floors_at_zero():
    """Decay never goes below zero."""
    assert _update_contact_ticks(0, 0.0, 5.0) == 0


def test_force_exactly_at_threshold_is_treated_as_no_contact():
    """Force == threshold is NOT counted as contact (strict >)."""
    assert _update_contact_ticks(3, 5.0, 5.0) == 2


def test_three_consecutive_zeros_clears_ticks():
    """Three consecutive zero-force frames fully clears the accumulator."""
    t = 3
    for _ in range(3):
        t = _update_contact_ticks(t, 0.0, 5.0)
    assert t == 0


# ─── Integration: perception FSM with decayed ticks ───────────────────────────

def test_single_glitch_gives_contact_tentative_not_touch_expected():
    """
    After 3 confirmed ticks followed by 1 zero-force glitch:
      OLD code: ticks=0  →  TOUCH_EXPECTED (foot near ground but ticks wiped)
      NEW code: ticks=2  →  CONTACT_TENTATIVE (contact confidence preserved)

    This test verifies the perception FSM correctly reads the decayed tick
    count (2) as CONTACT_TENTATIVE, proving the decay prevents the state
    regression to TOUCH_EXPECTED.
    """
    # Simulate state after decay: ticks dropped from 3 to 2 on a zero-force frame
    shared_state.left_contact_ticks = 2
    shared_state.left_foot_flat     = False   # no force → not flat
    shared_state.left_foot_position = np.array([0.0, 0.0, 0.01])  # near ground

    fsm = FootContactFSM('left', ContactConfig())
    state = fsm.update(
        height=0.01,         # below height_threshold=0.20 → would give TOUCH_EXPECTED if ticks=0
        velocity=np.zeros(3),
        force=0.0,
        current_time=0.0,
    )
    assert state == ContactState.CONTACT_TENTATIVE, (
        f"Expected CONTACT_TENTATIVE after single glitch with decay, got {state.name}. "
        f"Hard reset would give TOUCH_EXPECTED."
    )


def test_three_glitches_gives_touch_expected():
    """
    After three consecutive zero-force frames, ticks fully clears.
    The foot correctly falls back to TOUCH_EXPECTED (height < threshold).
    """
    shared_state.left_contact_ticks = 0
    shared_state.left_foot_flat     = False
    shared_state.left_foot_position = np.array([0.0, 0.0, 0.01])

    fsm = FootContactFSM('left', ContactConfig())
    state = fsm.update(
        height=0.01,
        velocity=np.zeros(3),
        force=0.0,
        current_time=0.0,
    )
    assert state == ContactState.TOUCH_EXPECTED, (
        f"Expected TOUCH_EXPECTED after ticks fully cleared, got {state.name}."
    )
