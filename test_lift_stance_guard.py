"""
Tests for LIFT → SWING stance-force guard (Fix 3).

Problem: _handle_lift exits to SWING when swing_force < threshold and
swing_vel_z < SETTLE_VEL_THRESHOLD, but does NOT verify the STANCE foot is
bearing load.  When a PyBullet GJK glitch drives both feet to F=0
simultaneously, the condition fires and the robot enters SWING while the
stance leg is also unloaded — causing immediate force collapse.

Fix: Gate the LIFT → SWING transition on stance_force >= UNLOAD_FORCE_THRESHOLD.
Timeout path: also abort to DS when stance_force < UNLOAD_FORCE_THRESHOLD.
"""
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState, StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    """Clean walking configuration: left swings, right is stance."""
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.LIFT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    # Nominal: swing (left) unloaded, stance (right) bearing load
    shared_state.left_foot_force           = 2.0   # N, below UNLOAD_FORCE_THRESHOLD=5.0
    shared_state.right_foot_force          = 30.0  # N, well above threshold
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.02])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.array([0.0, 0.0, 0.01])  # settled
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


# ── Nominal path (must still pass) ────────────────────────────────────────────

def test_lift_advances_to_swing_when_swing_unloaded_and_stance_bearing_load():
    """Existing behaviour preserved: swing unloaded + stance loaded → SWING."""
    _reset()
    # swing (left) = 2 N, stance (right) = 30 N — set by _reset()
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


# ── New guard: stance must be bearing load ────────────────────────────────────

def test_lift_blocked_when_stance_has_zero_force_even_if_swing_unloaded():
    """
    Both feet momentarily at F=0 (PyBullet GJK glitch):
    OLD code → advances to SWING (stance unloaded, wrong)
    NEW code → stays in LIFT until stance confirms load
    """
    _reset()
    shared_state.left_foot_force  = 2.0   # swing unloaded (passes old check)
    shared_state.right_foot_force = 0.0   # stance also has zero force (glitch)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT, (
        "Should hold LIFT when stance force = 0, even if swing force < threshold."
    )


def test_lift_advances_once_stance_force_recovers():
    """After the glitch clears and stance force returns, LIFT → SWING proceeds."""
    _reset()
    shared_state.left_foot_force  = 2.0   # swing unloaded
    shared_state.right_foot_force = 20.0  # stance bearing load again
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


# ── Timeout path with stance guard ────────────────────────────────────────────

def test_lift_timeout_aborts_to_ds_when_stance_is_unloaded():
    """
    LIFT_TIMEOUT with stance force = 0 → abort to DOUBLE_SUPPORT.
    Prevents advancing to SWING with an unloaded stance leg.
    """
    _reset()
    shared_state.step_phase_timer = 0.20   # > LIFT_TIMEOUT=0.15
    shared_state.left_foot_force  = 2.0    # swing unloaded
    shared_state.right_foot_force = 0.0    # stance unloaded (glitch/fall)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT, (
        "Timeout with unloaded stance must abort to DS, not advance to SWING."
    )


def test_lift_timeout_still_aborts_to_ds_when_swing_force_is_high():
    """Existing timeout behaviour preserved: swing force still high → abort."""
    _reset()
    shared_state.step_phase_timer = 0.20
    shared_state.left_foot_force  = 20.0   # swing still loaded
    shared_state.right_foot_force = 30.0   # stance loaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_lift_timeout_advances_to_swing_when_conditions_are_met():
    """Timeout with swing unloaded AND stance loaded → still advances to SWING."""
    _reset()
    shared_state.step_phase_timer = 0.20
    shared_state.left_foot_force  = 2.0    # swing unloaded
    shared_state.right_foot_force = 30.0   # stance loaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING
