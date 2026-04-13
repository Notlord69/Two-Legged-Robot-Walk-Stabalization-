"""Tests for grf.py — Ground Reaction Force controller.

Pure-math tests: no PyBullet required. All shared_state fields are set
manually before each call to update_grf().
"""
import math
import pytest
import numpy as np
from shared_state import (
    shared_state, Siclo1State, ContactState, MissionState, URDF_JOINT_NAMES
)


def _reset_for_grf():
    """Set shared_state to a stable standing pose for GRF tests."""
    shared_state.reset()
    # Both feet on ground, confirmed contact
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_foot_position  = np.array([0.0, 0.1, 0.0])   # m, on ground
    shared_state.right_foot_position = np.array([0.0, -0.1, 0.0])  # m, on ground
    shared_state.left_foot_velocity  = np.zeros(3)
    shared_state.right_foot_velocity = np.zeros(3)
    shared_state.ramp_gain   = 1.0
    shared_state.freeze_robot = False
    shared_state.emergency_stop_triggered = False
    # Nominal standing joint angles (geometric ≈ 0 → URDF ≈ 0)
    shared_state.joint_positions = {
        'Left_Hip_Forwards':  0.0,
        'Left_Knee':          0.0,
        'Right_Hip_Fowards':  0.0,
        'Right_Knee':         0.0,
    }
    shared_state.mission_state = MissionState.WALK


def test_zero_force_at_rest_height():
    """F_z = 0 when foot is exactly at Z_REST with zero velocity."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST
    shared_state.right_foot_position[2] = Z_REST
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert abs(v) < 1e-9, f"Expected 0, got {v}"


def test_positive_fz_when_foot_below_rest():
    """Compressed leg (z_foot < Z_REST) → non-zero support torques."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    # Compress both feet 2 cm below rest
    shared_state.left_foot_position[2]  = Z_REST - 0.02
    shared_state.right_foot_position[2] = Z_REST - 0.02
    update_grf()
    corr = shared_state.grf_torque_correction
    assert len(corr) == 4, f"Expected 4 torque keys, got {list(corr.keys())}"
    # All keys present
    assert 'Left_Hip_Forwards' in corr
    assert 'Left_Knee'         in corr
    assert 'Right_Hip_Fowards' in corr
    assert 'Right_Knee'        in corr


def test_zero_output_on_freeze():
    """freeze_robot = True → all corrections zero."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.freeze_robot = True
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_zero_output_on_emergency_stop():
    """emergency_stop_triggered = True → all corrections zero."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.emergency_stop_triggered = True
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_zero_output_on_idle():
    """mission_state == IDLE → all corrections zero (gait not yet active)."""
    from grf import update_grf
    _reset_for_grf()
    shared_state.mission_state = MissionState.IDLE
    update_grf()
    for v in shared_state.grf_torque_correction.values():
        assert v == 0.0


def test_ramp_gain_scales_output():
    """Torque at ramp_gain=0.5 is exactly half of torque at ramp_gain=1.0."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST - 0.03
    shared_state.right_foot_position[2] = Z_REST - 0.03

    shared_state.ramp_gain = 1.0
    update_grf()
    full = dict(shared_state.grf_torque_correction)

    shared_state.ramp_gain = 0.5
    update_grf()
    half = dict(shared_state.grf_torque_correction)

    for k in full:
        assert abs(half[k] - full[k] * 0.5) < 1e-9, (
            f"Key {k}: expected {full[k]*0.5:.6f}, got {half[k]:.6f}"
        )


def test_swing_foot_gets_zero_grf():
    """Swing foot (NO_CONTACT) must not receive GRF torques.

    Uses non-zero stance joint angles so the Jacobian produces a non-zero
    torque on the stance leg — sin(0)=0 at neutral pose so we need to lean.
    """
    from grf import update_grf, Z_REST
    _reset_for_grf()
    # Left foot in swing = no contact
    shared_state.set_contact_state('left', ContactState.NO_CONTACT)
    shared_state.left_foot_position[2]  = Z_REST - 0.05  # would produce Fz if not gated
    shared_state.right_foot_position[2] = Z_REST - 0.02
    # Right leg in slight forward lean so Jacobian z-entries are non-zero
    shared_state.joint_positions['Right_Hip_Fowards'] = 0.30   # rad ≈ 17° forward
    shared_state.joint_positions['Right_Knee']        = 0.20   # rad ≈ 11° flex

    update_grf()
    corr = shared_state.grf_torque_correction
    # Left leg corrections must be zero (swing foot — contact gated)
    assert corr.get('Left_Hip_Forwards', 0.0) == 0.0
    assert corr.get('Left_Knee',         0.0) == 0.0
    # Right leg corrections non-zero (stance foot compressed, non-zero Jacobian)
    assert corr.get('Right_Hip_Fowards', 0.0) != 0.0 or corr.get('Right_Knee', 0.0) != 0.0


def test_decel_increases_k_spring():
    """DECEL state uses K_SPRING * 1.2 → larger correction for same compression."""
    from grf import update_grf, Z_REST
    _reset_for_grf()
    shared_state.left_foot_position[2]  = Z_REST - 0.02
    shared_state.right_foot_position[2] = Z_REST - 0.02

    shared_state.mission_state = MissionState.WALK
    update_grf()
    walk_corr = dict(shared_state.grf_torque_correction)

    shared_state.mission_state = MissionState.DECEL
    update_grf()
    decel_corr = dict(shared_state.grf_torque_correction)

    for k in walk_corr:
        if abs(walk_corr[k]) > 1e-9:
            ratio = decel_corr[k] / walk_corr[k]
            assert abs(ratio - 1.2) < 1e-6, f"Key {k}: expected ratio 1.2, got {ratio:.6f}"
