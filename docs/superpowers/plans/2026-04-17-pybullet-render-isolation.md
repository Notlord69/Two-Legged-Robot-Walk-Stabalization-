# PyBullet Render Isolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate GUI-induced timing violations by moving PyBullet's `p.GUI` into a subprocess isolated from the 100 Hz physics loop, and add per-stage timing instrumentation to `step()`.

**Architecture:** Physics stays on `p.DIRECT` in the main process. A `VizBridge` class owns a `multiprocessing.shared_memory` pose buffer and a daemon subprocess (`viz/gui_worker.py`) that owns `p.GUI`. The 100 Hz loop writes pose data into the buffer with no locking; the subprocess reads it at `viz_fps` Hz. `GUISyncThread`, `PoseSnapshot`, `_push_gui_snapshot`, and `_sync_gui` are removed from `HeartBeat.py`.

**Tech Stack:** Python 3.10+, PyBullet, `multiprocessing.shared_memory` (stdlib 3.8+), numpy, pytest

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `shared_state.py` | Modify | Add `STAGE_NAMES` tuple + `_stage_times` ndarray to `Siclo1State` |
| `HeartBeat.py` | Modify | Stage timestamps in `step()`; replace `GUISyncThread` with `VizBridge`; remove `PoseSnapshot`, `_push_gui_snapshot`, `_sync_gui` |
| `main.py` | Modify | Update `--hold` logic: `gui_client` → `_viz_bridge.is_alive` |
| `sim/viz_bridge.py` | Create | `VizBridge` class: shared_memory allocation, subprocess launch, `push_pose` |
| `viz/gui_worker.py` | Create | Subprocess entry point: `p.GUI` owner, pose mirror loop |
| `Test Enviroment/test_stage_instrumentation.py` | Create | Tests for `STAGE_NAMES` and `_stage_times` |
| `Test Enviroment/test_viz_bridge.py` | Create | Tests for `VizBridge` layout, push, and lifecycle |
| `Test Enviroment/test_gui_worker.py` | Create | Tests for `gui_worker._init_pybullet` and `_render_loop` |
| `Test Enviroment/test_gui_sync_thread.py` | Delete | `GUISyncThread` removed |
| `Test Enviroment/test_gui_sync_fix.py` | Delete | Superseded by new tests |

---

## Task 1: Per-stage instrumentation — `shared_state.py`

**Files:**
- Modify: `shared_state.py:36-42` (after ERR constants block)
- Modify: `shared_state.py:370-371` (end of `Siclo1State.__init__`)
- Modify: `shared_state.py:545-546` (end of `reset()`)
- Create: `Test Enviroment/test_stage_instrumentation.py`

- [ ] **Step 1.1: Write the failing tests**

Create `Test Enviroment/test_stage_instrumentation.py`:

```python
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
```

- [ ] **Step 1.2: Run tests to verify they fail**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_stage_instrumentation.py" -v
```

Expected: `ImportError: cannot import name 'STAGE_NAMES' from 'shared_state'`

- [ ] **Step 1.3: Add `STAGE_NAMES` after the ERR constants in `shared_state.py`**

After line 42 (`ERR_PHASE_TIMEOUT = 6`), insert:

```python

# ============================================================================
# PER-STAGE TIMING CONSTANTS
# ============================================================================

STAGE_NAMES: tuple = (
    'sensors', 'link_positions', 'perception', 'stability',
    'active_balance', 'grf', 'gait_planner', 'mission',
    'wbc', 'recovery', 'apply_control', 'step_sim',
)  # 12 checkpoints matching step() stage order in HeartBeat.py
```

- [ ] **Step 1.4: Add `_stage_times` to `Siclo1State.__init__`**

After line 371 (`self._error_write_idx = 0`), insert:

```python

        # Per-stage elapsed times from cycle_start (seconds).
        # Written in-place by HeartBeat.step() at each stage boundary.
        # Zero allocation; never grows.
        self._stage_times: np.ndarray = np.zeros(len(STAGE_NAMES), dtype=np.float64)
```

- [ ] **Step 1.5: Reset `_stage_times` in `reset()`**

After line 546 (`self._error_write_idx = 0` inside `reset()`), insert:

```python
            self._stage_times[:] = 0.0
```

- [ ] **Step 1.6: Run tests to verify they pass**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_stage_instrumentation.py" -v
```

Expected: 6 PASSED

- [ ] **Step 1.7: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add shared_state.py "Test Enviroment/test_stage_instrumentation.py" && git commit -m "feat: add STAGE_NAMES and _stage_times to shared_state for per-stage timing"
```

---

## Task 2: Per-stage timestamps in `HeartBeat.step()`

**Files:**
- Modify: `HeartBeat.py:38-50` (add `STAGE_NAMES` to import)
- Modify: `HeartBeat.py:804-906` (add 12 timestamp writes in `step()`)

- [ ] **Step 2.1: Add `STAGE_NAMES` to the `shared_state` import in `HeartBeat.py`**

In `HeartBeat.py`, find the `from shared_state import (` block and add `STAGE_NAMES`:

```python
from shared_state import (
    shared_state,
    Siclo1State,
    TelemetryRingBuffer,
    SystemStatus,
    ContactState,
    MissionState,
    URDF_JOINT_NAMES,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
    ERR_TIMING_VIOLATION,
    ERR_MID_CYCLE_OVERRUN,
    STAGE_NAMES,
)
```

- [ ] **Step 2.2: Add timestamp writes inside `step()`**

In `HeartBeat.py`, inside the `step()` method, add `shared_state._stage_times[i] = time.perf_counter() - self.heartbeat.cycle_start` after each stage. Replace the body of `step()` from the `# 2. Sensors` comment through `# 14. Physics step` with the version below (everything else in the method stays unchanged):

```python
        # 2. Sensors
        self.pybullet.read_sensors()
        shared_state._stage_times[0] = time.perf_counter() - self.heartbeat.cycle_start

        if shared_state.freeze_robot:
            return False

        # 3. Link positions → avoids FK fallback
        self.pybullet.update_link_positions()
        shared_state._stage_times[1] = time.perf_counter() - self.heartbeat.cycle_start

        # 4. Perception
        perception.update_perception()
        shared_state._stage_times[2] = time.perf_counter() - self.heartbeat.cycle_start

        # 5. Stability
        stability.update_stability(dt=TARGET_DT)
        shared_state._stage_times[3] = time.perf_counter() - self.heartbeat.cycle_start

        # ── TIMING GUARD (mid-cycle) ────────────────────────────────────
        elapsed = time.perf_counter() - self.heartbeat.cycle_start
        if elapsed > OVERRUN_LIMIT:
            shared_state.add_error_code(ERR_MID_CYCLE_OVERRUN)
            shared_state.timing_violation_this_cycle = True

        # 6. Active balance
        active_balance.update_active_balance()
        shared_state._stage_times[4] = time.perf_counter() - self.heartbeat.cycle_start

        # 7. GRF — virtual spring-damper torque corrections
        grf.update_grf()
        shared_state._stage_times[5] = time.perf_counter() - self.heartbeat.cycle_start

        # 8. Gait Planner — swing arc + IK angle targets
        gait_planner.update_gait_planner()
        shared_state._stage_times[6] = time.perf_counter() - self.heartbeat.cycle_start

        # 9. Mission — state machine, ramp_gain, step counting
        self._mission.update()
        shared_state._stage_times[7] = time.perf_counter() - self.heartbeat.cycle_start

        # 10. WBC — IK angles → additive joint PD torques
        self._wbc_step()
        shared_state._stage_times[8] = time.perf_counter() - self.heartbeat.cycle_start

        # 11. Emergency gate
        if shared_state.emergency_stop_triggered:
            return False

        # 12. Recovery
        recovery.update_recovery()
        shared_state._stage_times[9] = time.perf_counter() - self.heartbeat.cycle_start

        # 13. Control
        self.pybullet.apply_control()
        shared_state._stage_times[10] = time.perf_counter() - self.heartbeat.cycle_start

        # 14. Physics step — DIRECT only (no render stall)
        sim.interface.step_simulation(self.physics_client)
        shared_state._stage_times[11] = time.perf_counter() - self.heartbeat.cycle_start
```

- [ ] **Step 2.3: Add worst-stage logging after `end_cycle`**

Find the comment `# 17. Write telemetry` in `step()`. Just before it (after `violation, comp_time = self.heartbeat.end_cycle()`), add:

```python
        # Per-stage worst logging (text buffer only — CSV schema unchanged)
        _prev = 0.0
        _worst_ms = 0.0
        _worst_stage = ''
        for _i in range(12):
            _t = shared_state._stage_times[_i]
            _delta = (_t - _prev) * 1000.0
            if _delta > _worst_ms:
                _worst_ms = _delta
                _worst_stage = STAGE_NAMES[_i]
            _prev = _t
        if _worst_ms > 3.0:
            self._telemetry_thread.log(
                f"[STAGE] worst={_worst_stage}  {_worst_ms:.2f} ms"
            )
```

- [ ] **Step 2.4: Run full test suite to confirm no regressions**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/" -v --tb=short 2>&1 | tail -20
```

Expected: all previously passing tests still pass.

- [ ] **Step 2.5: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add HeartBeat.py && git commit -m "feat: add per-stage timing timestamps to HeartBeat.step()"
```

---

## Task 3: `VizBridge` — constructor and `push_pose`

**Files:**
- Create: `sim/viz_bridge.py`
- Create: `Test Enviroment/test_viz_bridge.py` (layout + push tests only)

- [ ] **Step 3.1: Write the failing tests**

Create `Test Enviroment/test_viz_bridge.py`:

```python
"""Tests for VizBridge — shared_memory layout and push_pose behaviour.
Lifecycle tests (start/stop/is_alive) are in Task 4 below.
"""
import numpy as np
import pytest
import time


JOINT_NAMES = [
    'Left_Hip_Forwards', 'Left_Knee', 'Left_Ankle',
    'Right_Hip_Fowards', 'Right_Knee', 'Right_Ankle',
]
N = len(JOINT_NAMES)
URDF_PATH = '/fake/Siclo1.urdf'


@pytest.fixture
def bridge():
    from sim.viz_bridge import VizBridge
    b = VizBridge(joint_names=JOINT_NAMES, urdf_path=URDF_PATH)
    yield b
    b.stop()


class TestSharedMemoryLayout:
    def test_buffer_size(self, bridge):
        """Buffer must be exactly (1 + 3 + 4 + N) × 8 bytes."""
        assert bridge._shm.size == (1 + 3 + 4 + N) * 8

    def test_initial_seq_is_negative(self, bridge):
        """seq_num = -1 signals 'not ready' before start()."""
        assert bridge._arr[0] == -1.0

    def test_initial_active_false(self, bridge):
        assert bridge._active is False


class TestPushPose:
    def test_increments_seq(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[0] == pytest.approx(1.0)

    def test_writes_base_position(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((1.1, 2.2, 3.3), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[1] == pytest.approx(1.1)
        assert bridge._arr[2] == pytest.approx(2.2)
        assert bridge._arr[3] == pytest.approx(3.3)

    def test_writes_base_orientation(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((0.0, 0.0, 0.0), (0.1, 0.2, 0.3, 0.9),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[4] == pytest.approx(0.1)
        assert bridge._arr[5] == pytest.approx(0.2)
        assert bridge._arr[6] == pytest.approx(0.3)
        assert bridge._arr[7] == pytest.approx(0.9)

    def test_writes_joints_in_order(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        joints = {j: float(i) * 0.1 for i, j in enumerate(JOINT_NAMES)}
        bridge.push_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), joints)
        for i, jname in enumerate(JOINT_NAMES):
            assert bridge._arr[8 + i] == pytest.approx(joints[jname])

    def test_seq_incremented_last(self, bridge):
        """seq_num must be written after joint data (write-barrier semantic)."""
        bridge._active = True
        bridge._arr[0] = 5.0
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {'Left_Hip_Forwards': 0.5})
        assert bridge._arr[0] == pytest.approx(6.0)  # incremented by 1

    def test_noop_when_inactive(self, bridge):
        """push_pose must not modify buffer when _active is False."""
        bridge._active = False
        bridge.push_pose((9.9, 9.9, 9.9), (9.9, 9.9, 9.9, 9.9), {})
        assert bridge._arr[0] == -1.0   # unchanged
        assert bridge._arr[1] == 0.0    # unchanged

    def test_nonblocking(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        t0 = time.perf_counter()
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert time.perf_counter() - t0 < 0.001  # < 1 ms
```

- [ ] **Step 3.2: Run tests to verify they fail**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_viz_bridge.py" -v
```

Expected: `ModuleNotFoundError: No module named 'sim.viz_bridge'`

- [ ] **Step 3.3: Create `sim/viz_bridge.py` with constructor and `push_pose`**

```python
"""Subprocess-based GUI bridge: zero GIL contention, pure process isolation.

The 100 Hz physics loop calls push_pose() to write pose data into a shared
memory buffer. A daemon subprocess (viz/gui_worker.py) reads the buffer at
viz_fps Hz and mirrors the pose to a p.GUI window.

All p.* calls stay in the subprocess — never in this file.
"""
import multiprocessing
import time
import numpy as np
from multiprocessing.shared_memory import SharedMemory
from typing import Optional


class VizBridge:
    """Owns the shared_memory pose buffer and the GUI subprocess lifecycle.

    Buffer layout (float64 array):
        [0]      seq_num      — monotonic counter; GUI skips if unchanged
        [1:4]    base_pos     — (x, y, z) world position, metres
        [4:8]    base_orn     — quaternion (x, y, z, w)
        [8:8+N]  joint_angles — one value per joint in joint_names order
    """

    def __init__(self, joint_names: list, urdf_path: str,
                 viz_fps: int = 30, spawn_z: float = 0.8806) -> None:
        self._joint_names = list(joint_names)
        self._urdf_path = urdf_path
        self._viz_fps = viz_fps
        self._spawn_z = spawn_z
        self._n = len(self._joint_names)

        # Allocate shared memory: [seq(1) + pos(3) + orn(4) + joints(N)] × 8 bytes
        _n_floats = 1 + 3 + 4 + self._n
        self._shm = SharedMemory(create=True, size=_n_floats * 8)
        self._arr = np.ndarray((_n_floats,), dtype=np.float64, buffer=self._shm.buf)
        self._arr[:] = 0.0
        self._arr[0] = -1.0  # ready sentinel: -1 = subprocess not yet ready

        self._process: Optional[multiprocessing.Process] = None
        self._active: bool = False

    def push_pose(self, base_pos, base_orn,
                  joint_positions: dict) -> None:
        """Write latest pose into shared buffer. Non-blocking, no allocation.

        base_pos:        (x, y, z) or array-like, metres, world frame
        base_orn:        (x, y, z, w) or array-like, quaternion
        joint_positions: {joint_name: angle_rad}

        If subprocess has died, sets _active=False silently.
        """
        if not self._active:
            return
        if self._process is not None and not self._process.is_alive():
            self._active = False
            return
        arr = self._arr
        # Write pose fields first (seq_num incremented last — write barrier)
        arr[1] = base_pos[0]
        arr[2] = base_pos[1]
        arr[3] = base_pos[2]
        arr[4] = base_orn[0]
        arr[5] = base_orn[1]
        arr[6] = base_orn[2]
        arr[7] = base_orn[3]
        for i, jname in enumerate(self._joint_names):
            arr[8 + i] = joint_positions.get(jname, 0.0)
        arr[0] += 1.0  # seq_num: increment last
```

- [ ] **Step 3.4: Run tests to verify they pass**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_viz_bridge.py" -v
```

Expected: all 10 tests PASSED

- [ ] **Step 3.5: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add sim/viz_bridge.py "Test Enviroment/test_viz_bridge.py" && git commit -m "feat: add VizBridge constructor and push_pose with shared_memory buffer"
```

---

## Task 4: `VizBridge` — subprocess lifecycle (`start`, `stop`, `is_alive`)

**Files:**
- Modify: `sim/viz_bridge.py` (add `start`, `stop`, `is_alive`)
- Modify: `Test Enviroment/test_viz_bridge.py` (append lifecycle tests)

- [ ] **Step 4.1: Write failing lifecycle tests**

Append to `Test Enviroment/test_viz_bridge.py`:

```python

class TestLifecycle:
    def test_start_timeout_sets_inactive(self):
        """If subprocess never signals ready, _active stays False."""
        from sim.viz_bridge import VizBridge
        from unittest.mock import MagicMock, patch

        b = VizBridge(['Left_Hip_Forwards'], '/fake/path.urdf')
        mock_proc = MagicMock()
        mock_proc.is_alive.return_value = True

        ctx_mock = MagicMock()
        ctx_mock.Process.return_value = mock_proc

        with patch('sim.viz_bridge.multiprocessing.get_context', return_value=ctx_mock):
            # _timeout=0.01 → 10 ms timeout; arr[0] stays -1 → never ready
            b.start(_timeout=0.01)

        assert b._active is False
        b.stop()

    def test_stop_unlinks_shm(self):
        """stop() must unlink shared memory so it can't be re-opened by name."""
        from sim.viz_bridge import VizBridge
        from multiprocessing.shared_memory import SharedMemory

        b = VizBridge(['Left_Hip_Forwards'], '/fake/path.urdf')
        shm_name = b._shm.name
        b.stop()

        with pytest.raises(Exception):
            SharedMemory(name=shm_name)

    def test_is_alive_false_after_stop(self):
        from sim.viz_bridge import VizBridge
        from unittest.mock import MagicMock

        b = VizBridge(['Left_Hip_Forwards'], '/fake/path.urdf')
        mock_proc = MagicMock()
        mock_proc.is_alive.return_value = False
        b._process = mock_proc
        b._active = True
        b.stop()

        assert b.is_alive is False

    def test_is_alive_true_when_process_running(self):
        from sim.viz_bridge import VizBridge
        from unittest.mock import MagicMock

        b = VizBridge(['Left_Hip_Forwards'], '/fake/path.urdf')
        mock_proc = MagicMock()
        mock_proc.is_alive.return_value = True
        b._process = mock_proc

        assert b.is_alive is True
        b.stop()
```

- [ ] **Step 4.2: Run tests to verify they fail**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_viz_bridge.py::TestLifecycle" -v
```

Expected: `AttributeError: 'VizBridge' object has no attribute 'start'`

- [ ] **Step 4.3: Add `start`, `stop`, and `is_alive` to `sim/viz_bridge.py`**

Append to the `VizBridge` class (after `push_pose`):

```python
    def start(self, _timeout: float = 5.0) -> None:
        """Launch GUI subprocess. Spin-waits up to _timeout seconds for ready signal.

        On timeout: logs warning, leaves _active=False (physics runs headlessly).
        _timeout is exposed for testing; production code uses the default 5.0 s.
        """
        from viz import gui_worker  # deferred: keeps physics-process imports clean

        ctx = multiprocessing.get_context('spawn')  # 'spawn' avoids fork+OpenGL issues on WSL2
        self._process = ctx.Process(
            target=gui_worker.main,
            args=(
                self._shm.name, self._n, self._joint_names,
                self._urdf_path, self._viz_fps, self._spawn_z,
            ),
            daemon=True,
        )
        self._process.start()

        # Spin-wait: arr[0] changes from -1 to ≥0 once the subprocess is ready
        deadline = time.perf_counter() + _timeout
        while time.perf_counter() < deadline:
            if self._arr[0] >= 0.0:
                self._active = True
                return
            time.sleep(0.05)

        print("[VizBridge][WARN] GUI subprocess did not become ready within "
              f"{_timeout:.1f} s — running headlessly")
        self._active = False

    def stop(self) -> None:
        """Terminate subprocess and release shared memory."""
        if self._process is not None and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.kill()
        try:
            self._shm.close()
        except Exception:
            pass
        try:
            self._shm.unlink()
        except Exception:
            pass
        self._active = False

    @property
    def is_alive(self) -> bool:
        """True if the GUI subprocess is running."""
        return self._process is not None and self._process.is_alive()
```

- [ ] **Step 4.4: Run all `test_viz_bridge.py` tests**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_viz_bridge.py" -v
```

Expected: all 14 tests PASSED

- [ ] **Step 4.5: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add sim/viz_bridge.py "Test Enviroment/test_viz_bridge.py" && git commit -m "feat: add VizBridge subprocess lifecycle (start/stop/is_alive)"
```

---

## Task 5: `viz/gui_worker.py`

**Files:**
- Create: `viz/gui_worker.py`
- Create: `Test Enviroment/test_gui_worker.py`

- [ ] **Step 5.1: Write failing tests**

Create `Test Enviroment/test_gui_worker.py`:

```python
"""Tests for viz/gui_worker.py.

_init_pybullet and _render_loop are tested with mocked pybullet.
The infinite loop is exited via StopIteration injected into time.sleep.
"""
import numpy as np
import pytest
from unittest.mock import patch, MagicMock, call
from multiprocessing.shared_memory import SharedMemory


def _make_shm(n_joints: int):
    """Create shared memory + numpy view with sentinel seq_num=-1."""
    n_floats = 1 + 3 + 4 + n_joints
    shm = SharedMemory(create=True, size=n_floats * 8)
    arr = np.ndarray((n_floats,), dtype=np.float64, buffer=shm.buf)
    arr[:] = 0.0
    arr[0] = -1.0
    return shm, arr


class TestInitPybullet:
    def test_sets_ready_flag(self):
        """After _init_pybullet, arr[0] must be 0.0 (ready signal)."""
        joint_names = ['Left_Hip_Forwards', 'Left_Knee']
        shm, arr = _make_shm(len(joint_names))
        try:
            with patch('viz.gui_worker.p') as mock_p:
                mock_p.connect.return_value = 0
                mock_p.GUI = 1
                mock_p.URDF_USE_INERTIA_FROM_FILE = 0
                mock_p.COV_ENABLE_GUI = 0
                mock_p.loadURDF.return_value = 42
                mock_p.getNumJoints.return_value = 2
                mock_p.getJointInfo.side_effect = [
                    (0, b'Left_Hip_Forwards', *([None] * 10), b'link_a', None),
                    (1, b'Left_Knee',         *([None] * 10), b'link_b', None),
                ]

                from viz.gui_worker import _init_pybullet
                client, robot_id, joint_ids, period = _init_pybullet(
                    shm.name, len(joint_names), joint_names,
                    '/fake/Siclo1.urdf', 30, 0.8806,
                )

            assert arr[0] == 0.0  # ready signal set
            assert robot_id == 42
            assert joint_ids == {'Left_Hip_Forwards': 0, 'Left_Knee': 1}
            assert period == pytest.approx(1.0 / 30)
        finally:
            shm.close()
            shm.unlink()

    def test_builds_joint_id_map(self):
        """_init_pybullet must map joint names to pybullet joint indices."""
        joint_names = ['Left_Ankle']
        shm, arr = _make_shm(len(joint_names))
        try:
            with patch('viz.gui_worker.p') as mock_p:
                mock_p.connect.return_value = 0
                mock_p.GUI = 1
                mock_p.URDF_USE_INERTIA_FROM_FILE = 0
                mock_p.COV_ENABLE_GUI = 0
                mock_p.loadURDF.return_value = 1
                mock_p.getNumJoints.return_value = 1
                mock_p.getJointInfo.return_value = (
                    0, b'Left_Ankle', *([None] * 10), b'link_c', None
                )
                from viz.gui_worker import _init_pybullet
                _, _, joint_ids, _ = _init_pybullet(
                    shm.name, 1, joint_names, '/fake/Siclo1.urdf', 30, 0.8806,
                )
            assert joint_ids == {'Left_Ankle': 0}
        finally:
            shm.close()
            shm.unlink()


class TestRenderLoop:
    def test_applies_pose_when_seq_changes(self):
        joint_names = ['Left_Hip_Forwards']
        shm, arr = _make_shm(1)
        try:
            arr[0] = 1.0   # seq_num = 1 (new data; last_seq starts at -1)
            arr[1], arr[2], arr[3] = 0.1, 0.2, 0.88
            arr[4], arr[5], arr[6], arr[7] = 0.0, 0.0, 0.0, 1.0
            arr[8] = 0.5   # Left_Hip_Forwards angle

            with patch('viz.gui_worker.p') as mock_p, \
                 patch('viz.gui_worker.time') as mock_time:
                mock_time.perf_counter.return_value = 0.0
                mock_time.sleep.side_effect = StopIteration  # exit after 1 iter

                from viz.gui_worker import _render_loop
                try:
                    _render_loop(0, 1, {'Left_Hip_Forwards': 0},
                                 arr, 1.0 / 30, joint_names, 1)
                except StopIteration:
                    pass

            mock_p.resetBasePositionAndOrientation.assert_called_once_with(
                1, [0.1, 0.2, 0.88], [0.0, 0.0, 0.0, 1.0], physicsClientId=0
            )
            mock_p.resetJointState.assert_called_once_with(
                1, 0, 0.5, 0.0, physicsClientId=0
            )
            mock_p.stepSimulation.assert_called_once_with(physicsClientId=0)
        finally:
            shm.close()
            shm.unlink()

    def test_skips_frame_when_seq_unchanged(self):
        """After seq_num is consumed in iter 1, iter 2 must not call reset*.

        Two iterations: first applies (seq 1 != last_seq -1), second skips
        (seq 1 == last_seq 1). resetBasePositionAndOrientation called once total.
        """
        joint_names = ['Left_Hip_Forwards']
        shm, arr = _make_shm(1)
        try:
            arr[0] = 1.0   # seq_num = 1; never changes between iterations
            arr[1], arr[2], arr[3] = 0.1, 0.2, 0.88
            arr[4], arr[5], arr[6], arr[7] = 0.0, 0.0, 0.0, 1.0
            arr[8] = 0.5

            sleep_calls = [0]
            def counting_sleep(t):
                sleep_calls[0] += 1
                if sleep_calls[0] >= 2:
                    raise StopIteration  # exit after 2 iterations

            with patch('viz.gui_worker.p') as mock_p, \
                 patch('viz.gui_worker.time') as mock_time:
                mock_time.perf_counter.return_value = 0.0
                mock_time.sleep.side_effect = counting_sleep

                from viz.gui_worker import _render_loop
                try:
                    _render_loop(0, 1, {'Left_Hip_Forwards': 0},
                                 arr, 1.0 / 30, joint_names, 1)
                except StopIteration:
                    pass

            # Exactly one apply (first iter), zero on second (same seq)
            assert mock_p.resetBasePositionAndOrientation.call_count == 1
            assert mock_p.stepSimulation.call_count == 1
        finally:
            shm.close()
            shm.unlink()
```

- [ ] **Step 5.2: Run tests to verify they fail**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_gui_worker.py" -v
```

Expected: `ModuleNotFoundError: No module named 'viz.gui_worker'`

- [ ] **Step 5.3: Create `viz/gui_worker.py`**

```python
"""GUI subprocess entry point — never imported by the physics process.

Runs p.GUI in a completely separate OS process, reading pose data from a
shared_memory buffer written by VizBridge.push_pose() at 100 Hz.
The physics process's GIL is never shared; render stalls cannot cause violations.

Public API:
    main(...)          — entry point called by VizBridge.start()
    _init_pybullet(...)— init phase (separated for testability)
    _render_loop(...)  — infinite mirror loop (separated for testability)
"""
import os
import sys
import time
import numpy as np
from multiprocessing.shared_memory import SharedMemory

import pybullet as p
import pybullet_data


def _init_pybullet(shm_name: str, n_joints: int, joint_names: list,
                   urdf_path: str, viz_fps: int,
                   spawn_z: float) -> tuple:
    """Connect p.GUI, load world and robot, build joint map, signal ready.

    Returns: (client_id, robot_id, joint_ids, period)
        client_id  — PyBullet GUI client integer
        robot_id   — URDF body ID in the GUI client
        joint_ids  — {joint_name: pybullet_joint_index}
        period     — seconds per render frame (1 / viz_fps)
    """
    shm = SharedMemory(name=shm_name)
    arr = np.ndarray((1 + 3 + 4 + n_joints,), dtype=np.float64, buffer=shm.buf)

    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf", physicsClientId=client)
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(urdf_path)))
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, spawn_z],
        physicsClientId=client,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    p.setRealTimeSimulation(0, physicsClientId=client)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5, cameraYaw=90, cameraPitch=-20,
        cameraTargetPosition=[0.0, 0.0, 0.5],
        physicsClientId=client,
    )

    # Build joint name → index map
    joint_ids = {}
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        joint_ids[info[1].decode('utf-8')] = i

    # Signal ready to VizBridge.start() spin-wait: arr[0] was -1, now 0
    arr[0] = 0.0

    return client, robot_id, joint_ids, 1.0 / viz_fps


def _render_loop(client: int, robot_id: int, joint_ids: dict,
                 arr: np.ndarray, period: float,
                 joint_names: list, n_joints: int) -> None:
    """Infinite render loop. Runs until killed by parent (daemon process).

    Reads seq_num each frame; skips p.reset* calls if seq_num is unchanged
    (no-op frame). Camera interaction is handled natively by p.GUI event loop.
    """
    last_seq = -1.0

    while True:
        t0 = time.perf_counter()
        seq = arr[0]

        if seq != last_seq and seq >= 0.0:
            base_pos = [arr[1], arr[2], arr[3]]
            base_orn = [arr[4], arr[5], arr[6], arr[7]]
            p.resetBasePositionAndOrientation(
                robot_id, base_pos, base_orn, physicsClientId=client
            )
            for i, jname in enumerate(joint_names):
                jid = joint_ids.get(jname)
                if jid is not None:
                    p.resetJointState(
                        robot_id, jid, arr[8 + i], 0.0, physicsClientId=client
                    )
            p.stepSimulation(physicsClientId=client)
            last_seq = seq

        elapsed = time.perf_counter() - t0
        rem = period - elapsed
        if rem > 0.0:
            time.sleep(rem)


def main(shm_name: str, n_joints: int, joint_names: list,
         urdf_path: str, viz_fps: int, spawn_z: float) -> None:
    """Subprocess entry point called by VizBridge.start().

    Runs forever until killed by parent (daemon=True in VizBridge).
    """
    shm = SharedMemory(name=shm_name)
    arr = np.ndarray((1 + 3 + 4 + n_joints,), dtype=np.float64, buffer=shm.buf)

    client, robot_id, joint_ids, period = _init_pybullet(
        shm_name, n_joints, joint_names, urdf_path, viz_fps, spawn_z
    )
    _render_loop(client, robot_id, joint_ids, arr, period, joint_names, n_joints)
```

- [ ] **Step 5.4: Run `test_gui_worker.py` to verify tests pass**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_gui_worker.py" -v
```

Expected: all 4 tests PASSED

- [ ] **Step 5.5: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add viz/gui_worker.py "Test Enviroment/test_gui_worker.py" && git commit -m "feat: add viz/gui_worker subprocess entry point with render loop"
```

---

## Task 6: Wire `VizBridge` into `HeartBeat.py` and `main.py`

**Files:**
- Modify: `HeartBeat.py` (remove `GUISyncThread`, `PoseSnapshot`, `_push_gui_snapshot`, `_sync_gui`; add `VizBridge`)
- Modify: `main.py` (update `--hold` to use `_viz_bridge.is_alive`)

- [ ] **Step 6.1: Update imports in `HeartBeat.py`**

Replace the `import threading` line and add the VizBridge import. Find:

```python
import threading
```

Replace with:

```python
import threading
from sim.viz_bridge import VizBridge
```

- [ ] **Step 6.2: Remove `PoseSnapshot` dataclass from `HeartBeat.py`**

Delete the entire `PoseSnapshot` dataclass (lines 464–475 in the original):

```python
@dataclass
class PoseSnapshot:
    """Frozen robot state written by the 100 Hz loop; read by GUISyncThread.
    ...
    """
    base_pos:       tuple
    base_orn:       tuple
    joint_states:   dict
    link_positions: dict
    mission_state:  MissionState
    emergency_stop: bool
```

- [ ] **Step 6.3: Remove `GUISyncThread` class from `HeartBeat.py`**

Delete the entire `GUISyncThread` class (lines 483–584). It starts with:

```python
class GUISyncThread(threading.Thread):
```

and ends with:

```python
        except Exception:
            pass  # GUI mirror is non-critical
```

- [ ] **Step 6.4: Replace GUI client setup in `Siclo1Controller.__init__`**

Find this block (around line 609):

```python
        # 2. Optional GUI viewer
        self.gui_client: Optional[int] = None
        if use_gui:
            try:
                self.gui_client = p.connect(p.GUI)
                time.sleep(1.0)  # X-server buffer — wait for window to appear (WSL)
                p.removeAllUserDebugItems(physicsClientId=self.gui_client)  # clear comms pipe before spawn
            except Exception:
                self.gui_client = None
```

Replace with:

```python
        # 2. GUI subprocess placeholder (VizBridge started after warmup)
        self.gui_client: Optional[int] = None  # kept for API compatibility; always None
        self._viz_bridge: Optional[VizBridge] = None
```

- [ ] **Step 6.5: Remove the GUI world-mirror block from `__init__`**

Find and delete the entire `# 7. If GUI, mirror the scene` block (from `self._gui_robot_id: int = -1` through the closing `except` of the GUI load try). Replace the entire block with:

```python
        # 7. GUI robot loaded in subprocess — nothing to do in main process
```

- [ ] **Step 6.6: Replace `GUISyncThread` construction with `VizBridge` construction**

Find the debug visualiser + GUISyncThread block (around lines 724–739):

```python
        # Debug visualiser — GUI mode only
        if self.gui_client is not None:
            from viz.debug_markers import DebugVisualizer
            self._visualizer = DebugVisualizer(self.gui_client)

        # 11. GUISyncThread — display only (no recording)
        if self.gui_client is not None and self._gui_robot_id >= 0:
            self._gui_sync_thread = GUISyncThread(
                gui_client=self.gui_client,
                gui_robot_id=self._gui_robot_id,
                joint_list=self.pybullet._joint_list,
                viz_fps=30,
                visualizer=self._visualizer,
                left_hip_link=self._left_hip_link,
                right_hip_link=self._right_hip_link,
            )
            self._gui_sync_thread.start()
```

Replace with:

```python
        # 11. VizBridge — subprocess GUI (separate OS process, zero GIL contention)
        if use_gui:
            self._viz_bridge = VizBridge(
                joint_names=list(URDF_JOINT_NAMES.values()),
                urdf_path=urdf_file,
                viz_fps=max(1, 100 // self.viz_decimation),
                spawn_z=URDF_SPAWN_Z,
            )
            self._viz_bridge.start()
```

- [ ] **Step 6.7: Replace GUI sync call in `step()`**

Find (around line 902):

```python
        # 18. Optional GUI sync (decimated — every viz_decimation cycles)
        if (self._gui_sync_thread is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._push_gui_snapshot()
```

Replace with:

```python
        # 18. Optional GUI sync (decimated — every viz_decimation cycles)
        if (self._viz_bridge is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._viz_bridge.push_pose(
                shared_state.base_position,
                shared_state.base_orientation,
                shared_state.joint_positions,
            )
```

- [ ] **Step 6.8: Remove `_push_gui_snapshot` and `_sync_gui` methods**

Delete the entire `_push_gui_snapshot` method (lines 909–937) and the entire `_sync_gui` method (lines 940–967).

- [ ] **Step 6.9: Update `shutdown()` to use `_viz_bridge`**

Find in `shutdown()`:

```python
        # Step 2: stop GUI sync thread.
        if self._gui_sync_thread is not None and self._gui_sync_thread.is_alive():
            self._gui_sync_thread.stop()
            self._gui_sync_thread.join(timeout=2.0)
```

Replace with:

```python
        # Step 2: stop GUI subprocess.
        if self._viz_bridge is not None:
            self._viz_bridge.stop()
```

Also find and remove the gui_client disconnect block in `shutdown()`:

```python
        if self.gui_client is not None:
            try:
                p.disconnect(physicsClientId=self.gui_client)
            except Exception:
                pass
```

Delete it entirely (gui_client is always None in subprocess architecture).

- [ ] **Step 6.10: Update `--hold` logic in `main.py`**

Find in `main.py`:

```python
        if args.hold:
            if controller.gui_client is not None:
                print("[Siclo1] --hold active. Inspect final pose. Ctrl-C to exit.")
                try:
                    while p.isConnected(physicsClientId=controller.gui_client):
                        p.stepSimulation(physicsClientId=controller.physics_client)
                        time.sleep(0.01)  # 100 Hz physics keep; non-blocking GUI
                except KeyboardInterrupt:
                    print("\n[Siclo1] Hold ended.")
            else:
                print("[Siclo1] --hold ignored: no GUI client active (use --gui)")
```

Replace with:

```python
        if args.hold:
            if controller._viz_bridge is not None and controller._viz_bridge.is_alive:
                print("[Siclo1] --hold active. Inspect final pose. Ctrl-C to exit.")
                try:
                    while controller._viz_bridge.is_alive:
                        sim.interface.step_simulation(controller.physics_client)
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    print("\n[Siclo1] Hold ended.")
            else:
                print("[Siclo1] --hold ignored: no GUI active (use --gui)")
```

Also add `import sim.interface` to `main.py`'s imports (it currently imports `from HeartBeat import Siclo1Controller` — add `import sim.interface` after that line).

- [ ] **Step 6.11: Run the new test files to confirm wiring is correct**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/test_viz_bridge.py" "Test Enviroment/test_gui_worker.py" "Test Enviroment/test_stage_instrumentation.py" -v
```

Expected: all tests PASSED

- [ ] **Step 6.12: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add HeartBeat.py main.py && git commit -m "feat: wire VizBridge into HeartBeat; remove GUISyncThread and PoseSnapshot"
```

---

## Task 7: Delete obsolete tests and run full suite

**Files:**
- Delete: `Test Enviroment/test_gui_sync_thread.py`
- Delete: `Test Enviroment/test_gui_sync_fix.py`

- [ ] **Step 7.1: Delete obsolete test files**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && rm "Test Enviroment/test_gui_sync_thread.py" "Test Enviroment/test_gui_sync_fix.py"
```

- [ ] **Step 7.2: Run the full test suite**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && python -m pytest "Test Enviroment/" -v --tb=short 2>&1 | tail -30
```

Expected: all previously passing tests pass; new tests pass; no reference to `GUISyncThread` or `PoseSnapshot` in failures.

- [ ] **Step 7.3: Commit**

```bash
cd "/home/notlord/ros2_ws/Siclo1_V1" && git add -u && git commit -m "chore: delete obsolete GUISyncThread tests (replaced by test_viz_bridge + test_gui_worker)"
```
