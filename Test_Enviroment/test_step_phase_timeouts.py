"""Tests for phase timeout routing in the 5-state gait FSM."""
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
    shared_state.step_phase_timer          = 0.0
    shared_state.active_swing_side         = "left"
    shared_state.stance_side               = "right"
    shared_state.swing_phase               = 0.0
    shared_state.swing_foot_x_stance       = -0.1
    shared_state.step_count                = 0
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


def test_com_shift_timeout_high_force_aborts_to_ds():
    """COM_SHIFT timeout: swing force high → abort → DOUBLE_SUPPORT."""
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    shared_state.step_phase_timer  = 1.01   # > COM_SHIFT_TIMEOUT=1.0
    shared_state.capture_point     = np.array([0.50, 0.0])   # CP far → no normal exit
    shared_state.left_foot_force   = 20.0   # swing foot still loaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_com_shift_timeout_low_force_proceeds_to_lift():
    """COM_SHIFT timeout: swing force low → proceed → LIFT."""
    _reset()
    shared_state.step_phase        = StepPhase.COM_SHIFT
    shared_state.step_phase_timer  = 1.01
    shared_state.capture_point     = np.array([0.50, 0.0])
    shared_state.left_foot_force   = 2.0   # swing foot unloaded
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.LIFT


def test_lift_timeout_high_force_aborts_to_ds():
    """LIFT timeout: swing force high → abort → DOUBLE_SUPPORT."""
    _reset()
    shared_state.step_phase       = StepPhase.LIFT
    shared_state.step_phase_timer = 0.16   # > LIFT_TIMEOUT=0.15
    shared_state.left_foot_force  = 20.0
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_lift_timeout_low_force_proceeds_to_swing():
    """LIFT timeout: swing force low, stance loaded → proceed → SWING."""
    _reset()
    shared_state.step_phase         = StepPhase.LIFT
    shared_state.step_phase_timer   = 0.16
    shared_state.left_foot_force    = 2.0    # N, < SWING_UNLOAD_THRESHOLD=5 N
    shared_state.right_foot_force   = 65.0   # N, > STANCE_LOAD_THRESHOLD=60 N
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING


def test_swing_timeout_forces_place_and_logs_error():
    """SWING timeout (> 1.5 × SWING_DURATION=0.75s) → PLACE + ERR_PHASE_TIMEOUT."""
    _reset()
    shared_state.step_phase       = StepPhase.SWING
    shared_state.step_phase_timer = 0.76   # > 0.50 * 1.5 = 0.75
    shared_state.swing_phase      = 0.50   # mid-swing, not yet at PLACE_ENTRY_PHI
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    err_before = shared_state._error_write_idx
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.PLACE
    assert shared_state._error_write_idx == err_before + 1


def test_place_timeout_high_force_completes_step():
    """PLACE timeout: swing force high → contact was real → complete step."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51   # > PLACE_TIMEOUT=0.5
    shared_state.left_foot_force  = 20.0   # N, > SWING_UNLOAD_THRESHOLD=5 N
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT
    assert shared_state.step_count == 1


def test_place_timeout_low_force_freezes():
    """PLACE timeout: swing force low → foot missed ground → freeze."""
    _reset()
    shared_state.step_phase       = StepPhase.PLACE
    shared_state.step_phase_timer = 0.51
    shared_state.left_foot_force  = 1.0   # below threshold
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    shared_state.left_foot_velocity = np.zeros(3)
    gait_planner.update_gait_planner()
    assert shared_state.freeze_robot is True
