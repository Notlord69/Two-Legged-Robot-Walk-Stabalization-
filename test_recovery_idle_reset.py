"""Tests for IDLE step-timer reset in recovery.py.

No PyBullet required. All shared_state fields set manually.

Verifies:
  1. EMERGENCY_STOP never fires during a long IDLE (timer kept fresh).
  2. step_duration is near-zero on the first WALK cycle after a long IDLE
     (timer was reset on the last IDLE cycle, so WALK starts clean).
"""
import numpy as np
import pytest
from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StabilityStatus,
    RecoveryAction,
    RecoveryConfig,
)


def _reset_for_idle():
    """Shared_state for a robot standing in IDLE with both feet confirmed."""
    shared_state.reset()
    shared_state.mission_state = MissionState.IDLE
    shared_state.sim_time = 0.0
    shared_state.current_step_start_time = 0.0

    # Worst case: stability reports UNSTABLE (no contacts confirmed yet)
    shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=-0.05)
    # Both feet confirmed (normal standing)
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    # No slipping
    shared_state.set_slip_detection('left',  False)
    shared_state.set_slip_detection('right', False)


def test_no_emergency_stop_after_long_idle():
    """EMERGENCY_STOP must NOT fire after sim_time >> 3.0 s in IDLE.

    Scenario: robot stands in IDLE for 10 s. Timer has drifted:
        step_duration = 10.0 - 0.0 = 10.0 s >> 3.0 s threshold.
    Without the fix, Priority 1 (is_unstable + timeout) fires immediately.
    With the fix, current_step_start_time is reset each cycle → duration ≈ 0.
    """
    from recovery import update_recovery

    _reset_for_idle()
    # Simulate 10 s of sim_time passing with no timer reset
    shared_state.sim_time = 10.0
    shared_state.current_step_start_time = 0.0   # stale — the bug scenario

    update_recovery()

    assert shared_state.recovery_action != RecoveryAction.EMERGENCY_STOP, (
        f"EMERGENCY_STOP fired during IDLE after 10 s. "
        f"action={shared_state.recovery_action.name}, "
        f"reason='{shared_state.recovery_reason}'"
    )
    # Timer must have been reset to sim_time
    assert abs(shared_state.current_step_start_time - 10.0) < 1e-6, (
        f"Timer not reset: current_step_start_time={shared_state.current_step_start_time}"
    )


def test_step_duration_near_zero_on_first_walk_cycle():
    """step_duration must be near-zero on the first WALK cycle after long IDLE.

    Scenario:
      1. IDLE for 10 s — last IDLE cycle resets timer to sim_time=10.0.
      2. sim_time advances by one dt (10.01 s), mission_state flips to WALK.
      3. First WALK cycle: step_duration = 10.01 - 10.0 = 0.01 s << 3.0 s.
    Without the fix, step_duration = 10.01 - 0.0 = 10.01 s → immediate EMERGENCY_STOP.
    """
    from recovery import update_recovery

    _reset_for_idle()
    shared_state.sim_time = 10.0
    shared_state.current_step_start_time = 0.0   # stale

    # Simulate the last IDLE cycle (resets the timer)
    update_recovery()
    assert abs(shared_state.current_step_start_time - 10.0) < 1e-6

    # Advance one dt and flip to WALK
    shared_state.sim_time = 10.01
    shared_state.mission_state = MissionState.WALK
    # Both feet confirmed — no contact-loss trigger
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)

    update_recovery()

    duration = shared_state.get_step_duration()
    assert duration < 0.1, (
        f"step_duration on first WALK cycle too large: {duration:.3f} s. "
        f"Watchdog will fire immediately."
    )
    assert shared_state.recovery_action != RecoveryAction.EMERGENCY_STOP, (
        f"EMERGENCY_STOP fired on first WALK cycle. action={shared_state.recovery_action.name}"
    )
