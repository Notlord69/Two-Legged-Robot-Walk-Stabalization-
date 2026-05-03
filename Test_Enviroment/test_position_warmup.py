"""Tests for POSITION_CONTROL warmup — enter/restore mode methods and warmup structure."""
import inspect
import pytest


def test_enter_position_mode_exists():
    """PyBulletRobot must have enter_position_mode method."""
    from HeartBeat import PyBulletRobot
    assert hasattr(PyBulletRobot, 'enter_position_mode'), \
        "PyBulletRobot missing enter_position_mode"


def test_enter_position_mode_uses_position_control():
    """enter_position_mode must call POSITION_CONTROL with maxVelocity."""
    from HeartBeat import PyBulletRobot
    src = inspect.getsource(PyBulletRobot.enter_position_mode)
    assert 'POSITION_CONTROL' in src, "must use POSITION_CONTROL"
    assert 'maxVelocity' in src, "must set maxVelocity for gentle settling"


def test_restore_torque_mode_exists():
    """PyBulletRobot must have restore_torque_mode method."""
    from HeartBeat import PyBulletRobot
    assert hasattr(PyBulletRobot, 'restore_torque_mode'), \
        "PyBulletRobot missing restore_torque_mode"


def test_restore_torque_mode_uses_velocity_control_force_zero():
    """restore_torque_mode must re-disable motors with VELOCITY_CONTROL force=0."""
    from HeartBeat import PyBulletRobot
    src = inspect.getsource(PyBulletRobot.restore_torque_mode)
    assert 'VELOCITY_CONTROL' in src, "must use VELOCITY_CONTROL to hand joints to WBC"
    assert 'force=0' in src, "must set force=0 to disable motor constraint"


def test_warmup_calls_enter_position_mode():
    """_warmup must call enter_position_mode before the loop."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'enter_position_mode' in src, \
        "_warmup must call enter_position_mode"


def test_warmup_calls_restore_torque_mode():
    """_warmup must call restore_torque_mode after the loop."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'restore_torque_mode' in src, \
        "_warmup must call restore_torque_mode"


def test_warmup_does_not_call_wbc_step():
    """_warmup must not call _wbc_step — POSITION_CONTROL handles joints during warmup."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert '_wbc_step' not in src, \
        "_warmup must not call _wbc_step during POSITION_CONTROL warmup"


def test_warmup_does_not_call_apply_control():
    """_warmup must not call apply_control — POSITION_CONTROL is set once, not per-cycle."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'apply_control' not in src, \
        "_warmup must not call apply_control during POSITION_CONTROL warmup"
