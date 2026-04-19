"""Tests for mission.py state machine — no PyBullet required."""
import math
import pytest
from shared_state import shared_state, Siclo1State, ContactState, MissionState, RecoveryAction


def _reset_for_mission():
    shared_state.reset()
    shared_state.mission_state   = MissionState.IDLE
    shared_state.ramp_gain       = 0.0
    shared_state.step_count      = 0
    shared_state.steps_remaining = 0
    shared_state.freeze_robot    = False
    shared_state.emergency_stop_triggered = False
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)


def test_stays_idle_without_walk_distance():
    """No walk_distance → mission stays IDLE indefinitely."""
    from mission import MissionController
    mc = MissionController(walk_distance=None)
    _reset_for_mission()
    for _ in range(100):
        mc.update()
    assert shared_state.mission_state == MissionState.IDLE
    assert shared_state.ramp_gain == 0.0


def test_stays_idle_until_both_feet_confirmed():
    """With walk_distance set but feet not CONFIRMED → stay IDLE."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    # Only left foot confirmed
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    mc.update()
    assert shared_state.mission_state == MissionState.IDLE


def test_transitions_idle_to_ramp_when_both_feet_confirmed():
    """Both feet CONFIRMED + walk_distance set → IDLE transitions to RAMP."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()
    assert shared_state.mission_state == MissionState.RAMP


def test_ramp_increments_ramp_gain():
    """RAMP state increments ramp_gain by 1/50 per cycle."""
    from mission import MissionController, RAMP_RATE
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    assert shared_state.mission_state == MissionState.RAMP
    before = shared_state.ramp_gain
    mc.update()  # RAMP cycle 1
    after = shared_state.ramp_gain
    assert abs(after - before - RAMP_RATE) < 1e-9


def test_ramp_transitions_to_walk_at_full_gain():
    """After 50 RAMP cycles, ramp_gain reaches 1.0 and state → WALK."""
    from mission import MissionController
    mc = MissionController(walk_distance=1.0)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()
    assert abs(shared_state.ramp_gain - 1.0) < 1e-9
    assert shared_state.mission_state == MissionState.WALK


def test_steps_remaining_computed_correctly():
    """steps_remaining = ceil(distance / STEP_LENGTH) - step_count."""
    from mission import MissionController, STEP_LENGTH
    mc = MissionController(walk_distance=0.5)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()  # RAMP → WALK
    assert shared_state.mission_state == MissionState.WALK
    expected_total = math.ceil(0.5 / STEP_LENGTH)
    assert shared_state.steps_remaining == expected_total


def test_walk_transitions_to_decel_at_one_step_remaining():
    """WALK → DECEL when steps_remaining == 1."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)  # exactly 1 step
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    mc.update()  # IDLE → RAMP
    for _ in range(50):
        mc.update()  # reach WALK
    # steps_remaining should be 1 for 0.12 m / 0.12 m = 1 step
    assert shared_state.mission_state == MissionState.WALK
    assert shared_state.steps_remaining == 1
    mc.update()  # WALK → DECEL
    assert shared_state.mission_state == MissionState.DECEL


def test_decel_transitions_to_stop_at_zero_steps():
    """DECEL → STOP when steps_remaining == 0."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    # Force into DECEL with all steps completed (step_count == _steps_total)
    shared_state.mission_state   = MissionState.DECEL
    shared_state.step_count      = 1   # walk_distance=0.12 → _steps_total=1
    shared_state.ramp_gain       = 1.0
    mc.update()
    assert shared_state.mission_state == MissionState.STOP


def test_stop_decrements_ramp_gain():
    """STOP state decrements ramp_gain by 1/20 per cycle."""
    from mission import MissionController, STOP_RATE
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.mission_state   = MissionState.STOP
    shared_state.ramp_gain       = 1.0
    mc.update()
    assert abs(shared_state.ramp_gain - (1.0 - STOP_RATE)) < 1e-9


def test_stop_transitions_to_idle_at_zero_gain():
    """After 20 STOP cycles, ramp_gain → 0.0 and state → IDLE."""
    from mission import MissionController
    mc = MissionController(walk_distance=0.12)
    _reset_for_mission()
    shared_state.mission_state = MissionState.STOP
    shared_state.ramp_gain     = 1.0
    for _ in range(20):
        mc.update()
    assert abs(shared_state.ramp_gain) < 1e-9
    assert shared_state.mission_state == MissionState.IDLE


def test_emergency_stop_exits_immediately():
    """emergency_stop_triggered = True → mission goes to IDLE, ramp_gain = 0."""
    from mission import MissionController
    mc = MissionController(walk_distance=2.0)
    _reset_for_mission()
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain     = 1.0
    shared_state.emergency_stop_triggered = True
    mc.update()
    assert shared_state.ramp_gain == 0.0
    assert shared_state.mission_state == MissionState.IDLE


# ── Task 4 tests — recovery IDLE guard ───────────────────────────────────── #

def _reset_for_recovery():
    """Shared setup: step_duration = 4 s (> 3 s threshold), both feet unconfirmed.
    Priorities 1, 4, 5 are suppressed so only Priority 3 (timeout) can fire."""
    shared_state.reset()
    shared_state.current_step_start_time = 0.0
    shared_state.sim_time = 4.0           # step_duration = 4.0 > timeout_threshold
    # Both feet unconfirmed — would trigger Priority 3 if mission is not IDLE
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)
    # Priority 1 suppressed: is_unstable = False  (done by reset())
    # Priority 4 suppressed: slip flags = False    (done by reset())
    # Priority 5 suppressed: stability_status = STABLE (done by reset())


def test_recovery_timeout_blocked_when_idle():
    """Priority 3 timeout must NOT fire when mission_state == IDLE.
    A standing robot that was never commanded to walk must not self-destruct."""
    from recovery import RecoveryController
    _reset_for_recovery()
    shared_state.mission_state = MissionState.IDLE
    controller = RecoveryController()
    action, _ = controller.evaluate()
    assert action == RecoveryAction.NONE


def test_recovery_timeout_fires_when_walking():
    """Priority 3 timeout still fires during WALK — existing behaviour preserved."""
    from recovery import RecoveryController
    _reset_for_recovery()
    shared_state.mission_state = MissionState.WALK
    controller = RecoveryController()
    action, _ = controller.evaluate()
    assert action == RecoveryAction.REPOSITION
