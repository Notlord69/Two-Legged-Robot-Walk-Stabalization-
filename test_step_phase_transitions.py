"""Tests for nominal phase transitions in the 5-state gait FSM."""
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, Siclo1State, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    """Return shared_state to a clean walking configuration."""
    shared_state.freeze_robot              = False
    shared_state.emergency_stop_triggered  = False
    shared_state.mission_state             = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.step_count                = 0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0   # N, well above threshold
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])  # right foot locked
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


# ── DOUBLE_SUPPORT → COM_SHIFT ────────────────────────────────────────────────

def test_ds_does_not_advance_before_min_time():
    _reset()
    shared_state.step_phase_timer = 0.05   # < DS_MIN_TIME=0.10
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_ds_advances_to_com_shift_when_both_confirmed_and_min_time_elapsed():
    _reset()
    shared_state.step_phase_timer = 0.11   # > DS_MIN_TIME
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_ds_timer_resets_on_transition_to_com_shift():
    _reset()
    shared_state.step_phase_timer = 0.11
    gait_planner.update_gait_planner()
    # After one update the timer was already incremented by dt=0.01 before check
    # then reset to 0.0 on transition, then +dt re-added — so timer ≈ dt
    assert shared_state.step_phase_timer < 0.02


def test_ds_blocks_when_foot_not_confirmed():
    _reset()
    shared_state.step_phase_timer = 0.5
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


# ── COM_SHIFT → LIFT ─────────────────────────────────────────────────────────

def test_com_shift_advances_to_lift_when_cp_close_and_stable():
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    # CP near stance foot (right foot at x=0.1)
    shared_state.capture_point     = np.array([0.11, 0.0])  # |0.11-0.10| = 0.01 < 0.03
    shared_state.stability_status  = StabilityStatus.STABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_com_shift_blocked_when_cp_far():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.50, 0.0])   # far from stance x=0.10
    shared_state.stability_status = StabilityStatus.STABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_com_shift_blocked_when_unstable_even_if_cp_close():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.UNSTABLE
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_com_shift_timer_resets_on_transition_to_lift():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.STABLE
    shared_state.step_phase_timer = 0.5
    gait_planner.update_gait_planner()
    assert shared_state.step_phase_timer < 0.02


# ── LIFT → SWING ─────────────────────────────────────────────────────────────

def test_lift_advances_to_swing_when_unloaded_and_settled():
    _reset()
    shared_state.step_phase              = StepPhase.LIFT
    shared_state.left_foot_force         = 2.0    # < UNLOAD_FORCE_THRESHOLD=5.0
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])  # < 0.05 m/s
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_lift_blocked_when_force_still_high():
    _reset()
    shared_state.step_phase       = StepPhase.LIFT
    shared_state.left_foot_force  = 20.0   # > 5.0 N
    shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_blocked_when_velocity_not_settled():
    _reset()
    shared_state.step_phase              = StepPhase.LIFT
    shared_state.left_foot_force         = 2.0
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.20])  # > 0.05
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_snapshots_swing_foot_x_stance_on_transition():
    _reset()
    shared_state.step_phase          = StepPhase.LIFT
    shared_state.left_foot_position  = np.array([0.05, 0.0, 0.0])
    shared_state.left_foot_force     = 2.0
    shared_state.left_foot_velocity  = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.swing_phase == 0.0
    # x_stance snapped from left_foot_position[0] at transition
    assert abs(shared_state.swing_foot_x_stance - 0.05) < 1e-9
