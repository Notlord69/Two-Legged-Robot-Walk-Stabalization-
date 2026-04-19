"""Integration smoke test: verify gait wiring in HeartBeat initialises cleanly.

Requires PyBullet. Does NOT test physics — only that:
1. Siclo1Controller accepts walk_distance kwarg without error.
2. After init, mission_state == IDLE and ramp_gain == 0.0.
3. step() runs without exception for 10 cycles with walk_distance=None.
"""
import pytest
from shared_state import shared_state, MissionState


@pytest.fixture(scope="module")
def controller():
    from HeartBeat import Siclo1Controller
    ctrl = Siclo1Controller(use_gui=False, walk_distance=None)
    yield ctrl
    ctrl.finalize_telemetry()
    ctrl.shutdown()


def test_controller_accepts_walk_distance_none(controller):
    assert controller is not None


def test_mission_state_starts_idle(controller):
    assert shared_state.mission_state == MissionState.IDLE


def test_ramp_gain_starts_zero(controller):
    assert shared_state.ramp_gain == 0.0


def test_grf_torque_correction_exists(controller):
    # After init + warmup, grf_torque_correction is a dict
    assert isinstance(shared_state.grf_torque_correction, dict)


def test_ten_steps_run_without_exception(controller):
    for _ in range(10):
        controller.step()
