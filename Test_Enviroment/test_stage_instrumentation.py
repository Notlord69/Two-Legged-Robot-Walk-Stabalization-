import numpy as np
import pytest


def test_stage_names_count():
    from shared_state import STAGE_NAMES
    assert len(STAGE_NAMES) == 12


def test_stage_names_content():
    from shared_state import STAGE_NAMES
    expected = (
        'sensors', 'link_positions', 'perception', 'stability',
        'active_balance', 'grf', 'gait_planner', 'mission',
        'wbc', 'recovery', 'apply_control', 'step_sim',
    )
    assert STAGE_NAMES == expected


def test_stage_times_shape():
    from shared_state import Siclo1State
    s = Siclo1State()
    assert s._stage_times.shape == (12,)
    assert s._stage_times.dtype == np.float64


def test_stage_times_init_zero():
    from shared_state import Siclo1State
    s = Siclo1State()
    assert np.all(s._stage_times == 0.0)


def test_stage_times_reset():
    from shared_state import Siclo1State
    s = Siclo1State()
    s._stage_times[:] = 1.0
    s.reset()
    assert np.all(s._stage_times == 0.0)


def test_stage_times_monotonic():
    """Values written sequentially must be non-decreasing."""
    import time
    from shared_state import Siclo1State
    s = Siclo1State()
    for i in range(12):
        s._stage_times[i] = time.perf_counter()
    for i in range(1, 12):
        assert s._stage_times[i] >= s._stage_times[i - 1]
