"""Tests for hard safety guards in the 5-state gait FSM."""
import numpy as np
import pytest
from shared_state import (
    shared_state, ContactState, MissionState,
    StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.freeze_robot                = False
    shared_state.timing_violation_this_cycle = False
    shared_state.emergency_stop_triggered    = False
    shared_state.mission_state               = MissionState.WALK
    shared_state.ramp_gain                 = 1.0
    shared_state.last_dt                   = 0.01
    shared_state.step_phase                = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer          = 0.20   # past DS_MIN_TIME
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.capture_point             = np.array([0.0, 0.0])
    shared_state.stability_status          = StabilityStatus.STABLE
    shared_state.left_foot_contact_state   = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state  = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force           = 30.0
    shared_state.right_foot_force          = 30.0
    shared_state.left_foot_position        = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position       = np.array([ 0.1, 0.0, 0.0])
    shared_state.left_foot_velocity        = np.zeros(3)
    shared_state.right_foot_velocity       = np.zeros(3)
    shared_state.stance_foot_world_pos     = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions            = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }


def test_freeze_robot_blocks_all_phases():
    for phase in StepPhase:
        _reset()
        shared_state.step_phase   = phase
        shared_state.freeze_robot = True
        before = shared_state.step_phase
        gait_planner.update_gait_planner()
        assert shared_state.step_phase == before, (
            f"freeze_robot did not block phase {phase.name}"
        )


def test_idle_mission_blocks_all_phases():
    for phase in StepPhase:
        _reset()
        shared_state.step_phase    = phase
        shared_state.mission_state = MissionState.IDLE
        before = shared_state.step_phase
        gait_planner.update_gait_planner()
        assert shared_state.step_phase == before, (
            f"IDLE mission did not block phase {phase.name}"
        )


def test_unstable_blocks_com_shift_exit():
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])   # CP close to stance x=0.10
    shared_state.stability_status = StabilityStatus.UNSTABLE
    gait_planner.update_gait_planner()
    # Must NOT advance to LIFT
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_marginal_stability_does_not_block_com_shift_exit():
    """MARGINAL is not UNSTABLE — exit is allowed."""
    _reset()
    shared_state.step_phase       = StepPhase.COM_SHIFT
    shared_state.capture_point    = np.array([0.11, 0.0])
    shared_state.stability_status = StabilityStatus.MARGINAL
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_ds_freeze_when_timeout():
    _reset()
    shared_state.step_phase       = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 2.05   # > DS_TIMEOUT=2.0
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True


def test_place_freeze_when_timeout_and_no_force():
    """Foot missed ground at PLACE timeout → freeze."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51   # > PLACE_TIMEOUT=0.5
    shared_state.left_foot_force  = 1.0   # below UNLOAD_FORCE_THRESHOLD
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True
