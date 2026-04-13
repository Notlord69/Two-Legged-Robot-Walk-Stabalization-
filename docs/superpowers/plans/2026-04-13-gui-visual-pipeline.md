# GUI Visual Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix GUI-mode timing violations that terminate the simulation early, and add a post-run visual analysis pipeline (3D replay + time-series plots).

**Architecture:** Physics stays on `p.DIRECT` at 100 Hz. The GUI client is mirror-only (no physics stepping). Pose snapshots are logged per-cycle to a pre-allocated numpy array written to disk at shutdown. Two standalone scripts — `replay.py` and `analyze.py` — handle all post-run visual work.

**Tech Stack:** Python 3.10+, PyBullet, NumPy, Matplotlib, OpenCV (replay --record only)

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `HeartBeat.py` | Modify | `HeartbeatController` strict flag; warmup parity; `GUI_CONNECT_SETTLE_S`; `GUI_SYNC_FPS`; remove `p.stepSimulation` from `GUISyncThread`; integrate `PoseLogger` into `step()` and `shutdown()` |
| `pose_logger.py` | Create | Pre-allocated numpy ring; `record()` + `save()` |
| `replay.py` | Create | Post-run 3D pose playback; optional MP4 via `VideoRecorder` |
| `analyze.py` | Create | Post-run matplotlib plots from `telemetry.csv` |
| `test_heartbeat_strict.py` | Create | Tests for `HeartbeatController` strict flag |
| `test_gui_sync_fix.py` | Create | Test `GUISyncThread` never calls `p.stepSimulation` |
| `test_pose_logger.py` | Create | Tests for `PoseLogger` |
| `test_replay.py` | Create | Headless replay tests (no display needed) |
| `test_analyze.py` | Create | Plot generation tests (Agg backend, no display) |

---

### Task 1: HeartbeatController strict flag

**Files:**
- Modify: `HeartBeat.py` — `HeartbeatController.__init__` and `end_cycle()`
- Create: `test_heartbeat_strict.py`

**Background:** In GUI mode (WSL2), `p.connect(p.GUI)` adds overhead that causes the 100 Hz cycle to exceed 10 ms. Currently every violation calls `shared_state.increment_timing_violations()` and adds `ERR_TIMING_VIOLATION`, which can feed recovery logic and freeze the robot. The `strict=False` flag lets violations be counted internally without propagating to `shared_state`.

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_heartbeat_strict.py`:

```python
"""Tests for HeartbeatController strict/non-strict timing modes."""
from unittest.mock import patch, MagicMock, call
import pytest


def test_strict_mode_propagates_violation_to_shared_state():
    """In strict mode, a violation must call increment_timing_violations."""
    from HeartBeat import HeartbeatController
    # 1 µs target — any real computation exceeds it, guaranteeing a violation.
    hb = HeartbeatController(target_dt=0.000001, strict=True)
    hb.start_cycle()
    with patch('HeartBeat.shared_state') as mock_ss:
        mock_ss.increment_timing_violations = MagicMock()
        mock_ss.add_error_code = MagicMock()
        violation, _ = hb.end_cycle()
    assert violation is True
    mock_ss.increment_timing_violations.assert_called_once()


def test_non_strict_mode_does_not_propagate_violation():
    """In non-strict mode, a violation must NOT call increment_timing_violations."""
    from HeartBeat import HeartbeatController
    hb = HeartbeatController(target_dt=0.000001, strict=False)
    hb.start_cycle()
    with patch('HeartBeat.shared_state') as mock_ss:
        mock_ss.increment_timing_violations = MagicMock()
        mock_ss.add_error_code = MagicMock()
        violation, _ = hb.end_cycle()
    assert violation is True
    mock_ss.increment_timing_violations.assert_not_called()
    mock_ss.add_error_code.assert_not_called()


def test_non_strict_still_counts_violation_internally():
    """Internal _violations_count increments in both modes."""
    from HeartBeat import HeartbeatController
    for strict in (True, False):
        hb = HeartbeatController(target_dt=0.000001, strict=strict)
        hb.start_cycle()
        with patch('HeartBeat.shared_state'):
            violation, _ = hb.end_cycle()
        assert hb._violations_count == 1, f"failed for strict={strict}"


def test_strict_defaults_to_true():
    """HeartbeatController() with no strict arg behaves as strict=True."""
    from HeartBeat import HeartbeatController, TARGET_DT
    hb = HeartbeatController(target_dt=TARGET_DT)
    assert hb.strict is True
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest test_heartbeat_strict.py -v
```

Expected: 4 FAILED (AttributeError or TypeError — `strict` parameter doesn't exist yet)

- [ ] **Step 3: Add `strict` parameter to `HeartbeatController`**

In `HeartBeat.py`, locate the `HeartbeatController` class (line ~98). Make two edits:

**Edit `__init__`** — add `strict` parameter:

```python
def __init__(self, target_dt: float = TARGET_DT, strict: bool = True):
    self.strict = strict          # False in GUI mode: violations counted but not propagated
    self.target_dt = target_dt
    # ... rest unchanged ...
```

**Edit `end_cycle`** — gate the shared_state calls on `self.strict`:

Find this block in `end_cycle()`:
```python
if violation:
    self._violations_count += 1
    shared_state.increment_timing_violations()
    shared_state.add_error_code(ERR_TIMING_VIOLATION)
```

Replace with:
```python
if violation:
    self._violations_count += 1
    if self.strict:
        # Propagate to shared_state only in strict mode — feeds recovery logic.
        # GUI mode (strict=False) counts internally but never freezes the robot.
        shared_state.increment_timing_violations()
        shared_state.add_error_code(ERR_TIMING_VIOLATION)
```

- [ ] **Step 4: Pass `strict=not use_gui` when constructing HeartbeatController**

In `Siclo1Controller.__init__` (around line ~631), find:
```python
self.heartbeat = HeartbeatController(target_dt=TARGET_DT)
```

Replace with:
```python
self.heartbeat = HeartbeatController(target_dt=TARGET_DT, strict=not use_gui)
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
python3 -m pytest test_heartbeat_strict.py -v
```

Expected: 4 PASSED

- [ ] **Step 6: Commit**

```bash
git add HeartBeat.py test_heartbeat_strict.py
git commit -m "fix: add strict flag to HeartbeatController — GUI mode no longer propagates timing violations to shared_state"
```

---

### Task 2: GUISyncThread fix + warmup parity + constants

**Files:**
- Modify: `HeartBeat.py` — `GUISyncThread.run()`, `Siclo1Controller.__init__`
- Create: `test_gui_sync_fix.py`

**Background:** `GUISyncThread.run()` calls `p.stepSimulation(gui_client)` at 30 fps. PyBullet uses an internal mutex across all clients in the same process; this call competes with the main thread's physics step and causes 10+ ms stalls. The GUI client only needs pose resets — no physics stepping. Also, GUI warmup was 5 cycles (vs 50 headless); 5 steps is insufficient for foot contact confirmation, so the robot falls immediately.

- [ ] **Step 1: Write the failing test**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_gui_sync_fix.py`:

```python
"""Tests for GUISyncThread mutex-contention fix."""
from unittest.mock import MagicMock, patch
import time


def test_gui_sync_thread_never_calls_step_simulation():
    """GUISyncThread.run() must not call p.stepSimulation on any client.

    This is the primary fix for GUI-mode timing violations. PyBullet's
    internal mutex means p.stepSimulation(gui_client) blocks the 100 Hz loop.
    The GUI client only needs pose resets — not physics stepping.
    """
    from HeartBeat import GUISyncThread, PoseSnapshot, MissionState

    mock_p = MagicMock()

    with patch('HeartBeat.p', mock_p):
        thread = GUISyncThread(
            gui_client=0,
            gui_robot_id=0,
            joint_list=[],
            viz_fps=200,       # fast so test finishes in <0.1 s
            visualizer=None,
            left_hip_link='left',
            right_hip_link='right',
        )
        snap = PoseSnapshot(
            base_pos=(0.0, 0.0, 0.88),
            base_orn=(0.0, 0.0, 0.0, 1.0),
            joint_states={},
            link_positions={},
            mission_state=MissionState.IDLE,
            emergency_stop=False,
        )
        thread.push_pose(snap)
        thread.start()
        time.sleep(0.05)
        thread.stop()
        thread.join(timeout=1.0)

    mock_p.stepSimulation.assert_not_called()


def test_gui_sync_fps_constant_exists_and_is_reasonable():
    """GUI_SYNC_FPS must exist and be between 1 and 30 Hz."""
    import HeartBeat
    assert hasattr(HeartBeat, 'GUI_SYNC_FPS')
    assert 1 <= HeartBeat.GUI_SYNC_FPS <= 30
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest test_gui_sync_fix.py -v
```

Expected: 2 FAILED — `p.stepSimulation` is still called, `GUI_SYNC_FPS` doesn't exist yet

- [ ] **Step 3: Add `GUI_CONNECT_SETTLE_S` and `GUI_SYNC_FPS` constants**

In `HeartBeat.py`, find the constants block (around line ~63). Add after the existing constants:

```python
GUI_CONNECT_SETTLE_S: float = 1.0  # s — X-server buffer wait after p.connect(p.GUI) in WSL2
GUI_SYNC_FPS:         int   = 10   # Hz — GUI mirror rate; lower = less PyBullet mutex pressure
```

- [ ] **Step 4: Replace `time.sleep(1.0)` with the named constant**

Find in `Siclo1Controller.__init__` (around line ~625):
```python
self.gui_client = p.connect(p.GUI)
time.sleep(1.0)  # X-server buffer — wait for window to appear (WSL)
```

Replace with:
```python
self.gui_client = p.connect(p.GUI)
time.sleep(GUI_CONNECT_SETTLE_S)  # X-server buffer — wait for window (WSL2)
```

- [ ] **Step 5: Remove `p.stepSimulation` from `GUISyncThread.run()`**

Find the `run()` method in `GUISyncThread` (around line ~555):

```python
def run(self) -> None:
    """~viz_fps Hz render loop. Mirrors pose to the GUI window."""
    period = 1.0 / self._viz_fps
    while not self._stop_event.is_set():
        loop_start = time.perf_counter()

        with self._lock:
            snapshot = self._slot

        if snapshot is not None:
            self._mirror_pose(snapshot)
            p.stepSimulation(physicsClientId=self._gui_client)
            self._video_frame_count += 1

        elapsed = time.perf_counter() - loop_start
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)
```

Replace with:

```python
def run(self) -> None:
    """~viz_fps Hz render loop. Mirrors pose to the GUI window.

    p.stepSimulation is intentionally absent — the GUI client never advances
    physics. It only shows whatever the DIRECT client has already computed.
    Removing the step call eliminates the primary PyBullet mutex contention
    that caused >10 ms stalls in the 100 Hz physics loop.
    """
    period = 1.0 / self._viz_fps
    while not self._stop_event.is_set():
        loop_start = time.perf_counter()

        with self._lock:
            snapshot = self._slot

        if snapshot is not None:
            self._mirror_pose(snapshot)
            self._video_frame_count += 1

        elapsed = time.perf_counter() - loop_start
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)
```

- [ ] **Step 6: Use `GUI_SYNC_FPS` when constructing `GUISyncThread`**

Find in `Siclo1Controller.__init__` (around line ~741):
```python
self._gui_sync_thread = GUISyncThread(
    gui_client=self.gui_client,
    gui_robot_id=self._gui_robot_id,
    joint_list=self.pybullet._joint_list,
    viz_fps=30,
    visualizer=self._visualizer,
    left_hip_link=self._left_hip_link,
    right_hip_link=self._right_hip_link,
)
```

Replace `viz_fps=30` with `viz_fps=GUI_SYNC_FPS`.

- [ ] **Step 7: Fix warmup parity — GUI mode gets 50 cycles, same as headless**

Find in `Siclo1Controller.__init__` (around line ~730):
```python
warmup_cycles = 5 if self.use_gui else 50
```

Replace with:
```python
warmup_cycles = 50  # same in GUI and headless — 50 cycles needed for contact confirmation
```

- [ ] **Step 8: Run tests to verify they pass**

```bash
python3 -m pytest test_gui_sync_fix.py -v
```

Expected: 2 PASSED

- [ ] **Step 9: Commit**

```bash
git add HeartBeat.py test_gui_sync_fix.py
git commit -m "fix: remove p.stepSimulation from GUISyncThread; raise GUI warmup to 50 cycles; add GUI_SYNC_FPS/GUI_CONNECT_SETTLE_S constants"
```

---

### Task 3: PoseLogger

**Files:**
- Create: `pose_logger.py`
- Create: `test_pose_logger.py`

**Background:** Captures full robot pose (base position/orientation + 6 joint angles + forces + status) per cycle into a pre-allocated numpy array. Zero file I/O during the run — `save()` writes the array once at shutdown.

**Data layout (19 float64 per row):**

| Index | Field |
|-------|-------|
| 0 | sim_time (s) |
| 1–3 | base_pos (x, y, z) m |
| 4–7 | base_orn (qx, qy, qz, qw) |
| 8 | Left_Hip_Forwards (rad) |
| 9 | Left_Knee (rad) |
| 10 | Left_Ankle (rad) |
| 11 | Right_Hip_Fowards (rad) — note: URDF typo, "Fowards" not "Forwards" |
| 12 | Right_Knee (rad) |
| 13 | Right_Ankle (rad) |
| 14 | left_foot_force (N) |
| 15 | right_foot_force (N) |
| 16 | stability_status (StabilityStatus.value int) |
| 17 | mission_state (MissionState.value int) |
| 18 | spare (0.0) |

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_pose_logger.py`:

```python
"""Tests for PoseLogger — pre-allocated pose ring buffer."""
import os
import tempfile
import numpy as np
import pytest


_SAMPLE_JOINTS = {
    'Left_Hip_Forwards': 0.5,
    'Left_Knee':         0.6,
    'Left_Ankle':        0.7,
    'Right_Hip_Fowards': 0.8,   # URDF typo: "Fowards"
    'Right_Knee':        0.9,
    'Right_Ankle':       1.0,
}


def test_record_fills_correct_columns():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=10)
    logger.record(
        sim_time=0.01,
        base_pos=(1.0, 2.0, 3.0),
        base_orn=(0.1, 0.2, 0.3, 0.9),
        joint_positions=_SAMPLE_JOINTS,
        left_force=45.0,
        right_force=50.0,
        stability_status=1,
        mission_state=2,
    )
    row = logger._buf[0]
    assert row[0]  == pytest.approx(0.01)   # sim_time
    assert row[1]  == pytest.approx(1.0)    # base_pos.x
    assert row[2]  == pytest.approx(2.0)    # base_pos.y
    assert row[3]  == pytest.approx(3.0)    # base_pos.z
    assert row[4]  == pytest.approx(0.1)    # qx
    assert row[5]  == pytest.approx(0.2)    # qy
    assert row[6]  == pytest.approx(0.3)    # qz
    assert row[7]  == pytest.approx(0.9)    # qw
    assert row[8]  == pytest.approx(0.5)    # Left_Hip_Forwards
    assert row[9]  == pytest.approx(0.6)    # Left_Knee
    assert row[10] == pytest.approx(0.7)    # Left_Ankle
    assert row[11] == pytest.approx(0.8)    # Right_Hip_Fowards
    assert row[12] == pytest.approx(0.9)    # Right_Knee
    assert row[13] == pytest.approx(1.0)    # Right_Ankle
    assert row[14] == pytest.approx(45.0)   # left_force
    assert row[15] == pytest.approx(50.0)   # right_force
    assert row[16] == pytest.approx(1.0)    # stability_status
    assert row[17] == pytest.approx(2.0)    # mission_state
    assert row[18] == pytest.approx(0.0)    # spare


def test_record_handles_missing_joints_gracefully():
    """Unknown/absent joint names default to 0.0 — no KeyError."""
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=1)
    logger.record(
        sim_time=0.0, base_pos=(0, 0, 0), base_orn=(0, 0, 0, 1),
        joint_positions={},  # empty — all joints absent
        left_force=0.0, right_force=0.0, stability_status=0, mission_state=0,
    )
    row = logger._buf[0]
    for col in range(8, 14):   # joint columns 8–13
        assert row[col] == pytest.approx(0.0), f"col {col} should be 0.0"


def test_save_writes_npy_with_correct_shape():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=5)
    for i in range(3):
        logger.record(
            sim_time=float(i) * 0.01,
            base_pos=(0.0, 0.0, 0.88), base_orn=(0.0, 0.0, 0.0, 1.0),
            joint_positions={}, left_force=0.0, right_force=0.0,
            stability_status=0, mission_state=0,
        )
    with tempfile.TemporaryDirectory() as tmpdir:
        path = logger.save(tmpdir)
        assert os.path.isfile(path)
        loaded = np.load(path)
        assert loaded.shape == (3, 19)
        assert loaded.dtype == np.float64


def test_save_returns_poses_npy_path():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=2)
    logger.record(0.0, (0, 0, 0), (0, 0, 0, 1), {}, 0.0, 0.0, 0, 0)
    with tempfile.TemporaryDirectory() as tmpdir:
        path = logger.save(tmpdir)
        assert path == os.path.join(tmpdir, 'poses.npy')


def test_buffer_overflow_is_silent():
    """When buffer is full, extra records are silently dropped."""
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=2)
    for _ in range(5):
        logger.record(0.0, (0, 0, 0), (0, 0, 0, 1), {}, 0.0, 0.0, 0, 0)
    assert logger._idx == 2   # capped at max_cycles


def test_index_advances_per_record():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=10)
    for i in range(4):
        logger.record(
            float(i) * 0.01, (0, 0, 0), (0, 0, 0, 1),
            {}, 0.0, 0.0, 0, 0,
        )
    assert logger._idx == 4
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest test_pose_logger.py -v
```

Expected: 6 FAILED — `pose_logger` module not found

- [ ] **Step 3: Implement `pose_logger.py`**

Create `/home/notlord/ros2_ws/Siclo1_V1/pose_logger.py`:

```python
"""
================================================================================
PROJECT SICLO1 — POSE LOGGER
================================================================================

Pre-allocated numpy ring buffer that captures full robot pose per cycle.
Zero file I/O during the run — save() writes poses.npy once at shutdown.

Layout per row (19 × float64):
  [0]     sim_time (s)
  [1–3]   base_pos (x, y, z) m
  [4–7]   base_orn (qx, qy, qz, qw)
  [8]     Left_Hip_Forwards (rad)
  [9]     Left_Knee (rad)
  [10]    Left_Ankle (rad)
  [11]    Right_Hip_Fowards (rad)   — note: URDF typo preserved
  [12]    Right_Knee (rad)
  [13]    Right_Ankle (rad)
  [14]    left_foot_force (N)
  [15]    right_foot_force (N)
  [16]    stability_status (StabilityStatus.value)
  [17]    mission_state (MissionState.value)
  [18]    spare (0.0)

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import os
import numpy as np

_COLS: int = 19  # float64 columns per row

# Joint column indices — must match _WBC_*_JOINTS order in HeartBeat.py
_COL_LEFT_HIP   = 8
_COL_LEFT_KNEE  = 9
_COL_LEFT_ANKLE = 10
_COL_RIGHT_HIP  = 11
_COL_RIGHT_KNEE = 12
_COL_RIGHT_ANKLE = 13

_JOINT_COLS = [
    ('Left_Hip_Forwards', _COL_LEFT_HIP),
    ('Left_Knee',         _COL_LEFT_KNEE),
    ('Left_Ankle',        _COL_LEFT_ANKLE),
    ('Right_Hip_Fowards', _COL_RIGHT_HIP),   # URDF typo: "Fowards"
    ('Right_Knee',        _COL_RIGHT_KNEE),
    ('Right_Ankle',       _COL_RIGHT_ANKLE),
]


class PoseLogger:
    """Pre-allocated numpy ring buffer for per-cycle robot pose capture.

    record() is a single numpy row write — zero allocation, zero file I/O.
    save() is called once at shutdown; writes poses.npy to the session folder.
    """

    def __init__(self, max_cycles: int = 20_000) -> None:
        self._buf: np.ndarray = np.zeros((max_cycles, _COLS), dtype=np.float64)
        self._idx: int = 0
        self._max: int = max_cycles

    def record(
        self,
        sim_time:         float,
        base_pos:         tuple,
        base_orn:         tuple,
        joint_positions:  dict,
        left_force:       float,
        right_force:      float,
        stability_status: int,
        mission_state:    int,
    ) -> None:
        """Write one pose row into the pre-allocated buffer.

        Silently drops the record if the buffer is full.
        Missing joint names default to 0.0 — no KeyError.
        """
        if self._idx >= self._max:
            return
        row = self._buf[self._idx]
        row[0]  = sim_time
        row[1]  = base_pos[0]
        row[2]  = base_pos[1]
        row[3]  = base_pos[2]
        row[4]  = base_orn[0]   # qx
        row[5]  = base_orn[1]   # qy
        row[6]  = base_orn[2]   # qz
        row[7]  = base_orn[3]   # qw
        for jname, col in _JOINT_COLS:
            row[col] = joint_positions.get(jname, 0.0)
        row[14] = left_force
        row[15] = right_force
        row[16] = float(stability_status)
        row[17] = float(mission_state)
        row[18] = 0.0           # spare
        self._idx += 1

    def save(self, session_path: str) -> str:
        """Write recorded rows to poses.npy in session_path.

        Returns the absolute path to the written file.
        Saves only the rows that were actually recorded (_idx rows).
        Call once at shutdown, after the run loop completes.
        """
        path = os.path.join(session_path, 'poses.npy')
        np.save(path, self._buf[:self._idx])
        return path
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest test_pose_logger.py -v
```

Expected: 6 PASSED

- [ ] **Step 5: Commit**

```bash
git add pose_logger.py test_pose_logger.py
git commit -m "feat: add PoseLogger — pre-allocated numpy ring buffer for per-cycle pose capture"
```

---

### Task 4: Integrate PoseLogger into HeartBeat.py

**Files:**
- Modify: `HeartBeat.py` — `Siclo1Controller.__init__`, `step()`, `_push_gui_snapshot()`, `shutdown()`

**Background:** `step()` needs to call `p.getBasePositionAndOrientation` once per cycle and share the result between `PoseLogger.record()` and `_push_gui_snapshot()`. Currently `_push_gui_snapshot()` calls `p.getBasePositionAndOrientation` itself; after this task it receives the values as parameters, eliminating a duplicate call.

- [ ] **Step 1: Add `PoseLogger` import and construction in `__init__`**

At the top of `HeartBeat.py`, after the existing local imports, add:

```python
from pose_logger import PoseLogger
```

In `Siclo1Controller.__init__`, after the `TelemetryThread` is started (around line ~710), add:

```python
# 12b. Pose logger — pre-allocated ring, written to disk at shutdown
self._pose_logger = PoseLogger(max_cycles=50_000)  # 500 s @ 100 Hz
```

Note: the `VideoRecorder` block (lines ~752-761) stays unchanged (disabled).

- [ ] **Step 2: Add `p.getBasePositionAndOrientation` call in `step()` and pass to `PoseLogger`**

In `Siclo1Controller.step()`, find step 17 (write telemetry, around line ~912):

```python
# 17. Write telemetry (zero-alloc: reuse scratch row)
row = shared_state._telem_row
row[0] = shared_state.sim_time
```

Insert **before** this block:

```python
# Read base pose once — reused by PoseLogger and _push_gui_snapshot.
_base_pos, _base_orn = p.getBasePositionAndOrientation(
    self.pybullet.robot_id, physicsClientId=self.physics_client)

# Pose log — zero I/O, single numpy row write
self._pose_logger.record(
    sim_time=shared_state.sim_time,
    base_pos=_base_pos,
    base_orn=_base_orn,
    joint_positions=shared_state.joint_positions,
    left_force=shared_state.left_foot_force,
    right_force=shared_state.right_foot_force,
    stability_status=shared_state.stability_status.value,
    mission_state=shared_state.mission_state.value,
)
```

- [ ] **Step 3: Pass `_base_pos/_base_orn` to `_push_gui_snapshot` to avoid duplicate call**

In `step()`, find step 18 (around line ~930):

```python
# 18. Optional GUI sync (decimated — every viz_decimation cycles)
if (self._gui_sync_thread is not None and
        shared_state.cycle_count % self.viz_decimation == 0):
    self._push_gui_snapshot()
```

Replace with:

```python
# 18. Optional GUI sync (decimated — every viz_decimation cycles)
if (self._gui_sync_thread is not None and
        shared_state.cycle_count % self.viz_decimation == 0):
    self._push_gui_snapshot(_base_pos, _base_orn)
```

- [ ] **Step 4: Update `_push_gui_snapshot` signature to accept base pose**

Find `_push_gui_snapshot` (around line ~938):

```python
def _push_gui_snapshot(self) -> None:
    """Build a PoseSnapshot from current physics state and push to GUISyncThread.
    ...
    """
    if self._gui_sync_thread is None or self._gui_robot_id < 0:
        return
    try:
        pos, orn = p.getBasePositionAndOrientation(
            self.pybullet.robot_id, physicsClientId=self.physics_client)
        joint_states = {
```

Replace with:

```python
def _push_gui_snapshot(self, base_pos: tuple, base_orn: tuple) -> None:
    """Build a PoseSnapshot from current physics state and push to GUISyncThread.

    base_pos, base_orn: pre-read from the DIRECT client in step() — reused
    here to avoid a duplicate p.getBasePositionAndOrientation call.
    """
    if self._gui_sync_thread is None or self._gui_robot_id < 0:
        return
    try:
        joint_states = {
```

And in the `PoseSnapshot(...)` construction inside `_push_gui_snapshot`, replace `pos` and `orn` with `base_pos` and `base_orn`:

```python
snap = PoseSnapshot(
    base_pos=base_pos,
    base_orn=base_orn,
    joint_states=joint_states,
    link_positions=dict(shared_state.link_positions),
    mission_state=shared_state.mission_state,
    emergency_stop=shared_state.emergency_stop_triggered,
)
```

- [ ] **Step 5: Call `PoseLogger.save()` in `shutdown()`**

In `Siclo1Controller.shutdown()`, find the first line (around line ~1069):

```python
def shutdown(self) -> None:
    # Step 1: stop VideoRecorder (disabled — self._recorder is None when off)
```

After the VideoRecorder block (or immediately after `# Step 1`), add:

```python
# Step 1b: save pose log to session folder
if hasattr(self, '_pose_logger') and hasattr(self, '_telemetry_thread'):
    try:
        path = self._pose_logger.save(self._telemetry_thread.session_path)
        print(f"[Siclo1] Pose log saved: {path}  ({self._pose_logger._idx} frames)")
    except Exception as exc:
        print(f"[Siclo1][WARN] Pose log save failed: {exc}")
```

- [ ] **Step 6: Smoke-test the integration (headless)**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 main.py --duration 100 2>&1 | tail -5
```

Expected output includes: `[Siclo1] Pose log saved: sessions/.../poses.npy  (100 frames)`

- [ ] **Step 7: Run all existing tests to confirm no regressions**

```bash
python3 -m pytest test_heartbeat_strict.py test_gui_sync_fix.py test_pose_logger.py test_grf.py test_stance_anchor.py test_step_phase_guards.py -v
```

Expected: all PASSED

- [ ] **Step 8: Commit**

```bash
git add HeartBeat.py
git commit -m "feat: integrate PoseLogger into HeartBeat — per-cycle pose capture; save poses.npy at shutdown"
```

---

### Task 5: replay.py

**Files:**
- Create: `replay.py`
- Create: `test_replay.py`

**Background:** Standalone post-run 3D replay. Reads `poses.npy`, steps PyBullet with pose resets (no control logic), optionally captures frames via `VideoRecorder`. No imports from `HeartBeat.py` or any control module.

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_replay.py`:

```python
"""Tests for replay.py — post-run 3D session replay."""
import os
import tempfile
import numpy as np
import pytest


def _write_poses_npy(tmpdir: str, n_frames: int = 5) -> str:
    """Write a minimal poses.npy for testing (identity pose, z=0.88)."""
    poses = np.zeros((n_frames, 19), dtype=np.float64)
    for i in range(n_frames):
        poses[i, 0] = i * 0.01   # sim_time
        poses[i, 3] = 0.8806     # base_pos.z — valid spawn height
        poses[i, 7] = 1.0        # base_orn.qw — identity quaternion
    path = os.path.join(tmpdir, 'poses.npy')
    np.save(path, poses)
    return tmpdir


def test_replay_headless_completes_without_error():
    """Headless replay of 3 frames must finish without raising."""
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=3)
        from replay import replay
        replay(tmpdir, speed=1000.0, record=False, headless=True)


def test_replay_raises_file_not_found_if_poses_missing():
    """FileNotFoundError with 'poses.npy' in message when file is absent."""
    with tempfile.TemporaryDirectory() as tmpdir:
        from replay import replay
        with pytest.raises(FileNotFoundError, match='poses.npy'):
            replay(tmpdir, speed=1.0, record=False, headless=True)


def test_replay_loads_correct_frame_count():
    """Replay must process exactly the number of rows in poses.npy."""
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=7)
        from replay import _load_poses
        poses = _load_poses(tmpdir)
        assert poses.shape == (7, 19)


def test_replay_speed_multiplier_accepted():
    """--speed values above and below 1.0 must not raise."""
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=2)
        from replay import replay
        for speed in (0.1, 1.0, 10.0, 500.0):
            replay(tmpdir, speed=speed, record=False, headless=True)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest test_replay.py -v
```

Expected: 4 FAILED — `replay` module not found

- [ ] **Step 3: Implement `replay.py`**

Create `/home/notlord/ros2_ws/Siclo1_V1/replay.py`:

```python
"""
================================================================================
PROJECT SICLO1 — POST-RUN 3D REPLAY
================================================================================

Reads poses.npy from a session folder and plays back the robot in 3D.
Optionally records walk.mp4 via VideoRecorder (TinyRenderer, no GPU needed).

No imports from HeartBeat.py or any control module (stability, gait, WBC).
Only uses: sim/interface.py, recorder.py, pybullet, numpy.

Usage:
    python3 replay.py sessions/2026-04-13_14-30-00/
    python3 replay.py sessions/2026-04-13_14-30-00/ --speed 0.5
    python3 replay.py sessions/2026-04-13_14-30-00/ --record
    python3 replay.py sessions/2026-04-13_14-30-00/ --headless --record

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import argparse
import os
import time
import numpy as np
import pybullet as p
import pybullet_data

# ── Constants ─────────────────────────────────────────────────────────────────

TARGET_DT: float = 0.01       # s — matches HeartBeat.py physics step

URDF_SPAWN_Z: float = 0.8806  # m — URDF-aligned spawn height

# Joint names in the order they appear in poses.npy columns 8–13.
# Must match _JOINT_COLS in pose_logger.py.
_REPLAY_JOINT_NAMES = [
    'Left_Hip_Forwards',   # col 8
    'Left_Knee',           # col 9
    'Left_Ankle',          # col 10
    'Right_Hip_Fowards',   # col 11 — URDF typo preserved
    'Right_Knee',          # col 12
    'Right_Ankle',         # col 13
]

# poses.npy column slices
_COL_TIME  = 0
_COL_POS   = slice(1, 4)
_COL_ORN   = slice(4, 8)
_COL_J_START = 8
_COL_J_END   = 14


# ── Public helpers ─────────────────────────────────────────────────────────────

def _load_poses(session_path: str) -> np.ndarray:
    """Load poses.npy from session_path. Raises FileNotFoundError if absent."""
    path = os.path.join(session_path, 'poses.npy')
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"poses.npy not found in {session_path}. "
            "Run with --gui to generate pose logs."
        )
    return np.load(path)


def _build_joint_map(robot_id: int, client: int) -> dict:
    """Return {joint_name: joint_index} for the 6 active replay joints."""
    joint_map = {}
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        name = info[1].decode('utf-8')
        if name in _REPLAY_JOINT_NAMES:
            joint_map[name] = i
    return joint_map


# ── Main replay function ───────────────────────────────────────────────────────

def replay(
    session_path: str,
    speed:        float = 1.0,
    record:       bool  = False,
    headless:     bool  = False,
) -> None:
    """Play back a session in PyBullet.

    session_path : folder containing poses.npy (and optionally telemetry.csv)
    speed        : playback speed multiplier (1.0 = real-time, 0.5 = half speed)
    record       : write walk.mp4 to session_path via VideoRecorder
    headless     : use p.DIRECT instead of p.GUI (combine with record for MP4-only)
    """
    poses = _load_poses(session_path)
    n_frames = len(poses)
    print(f"[replay] {n_frames} frames loaded from {session_path}")

    client = p.connect(p.DIRECT if headless else p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.loadURDF('plane.urdf', physicsClientId=client)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path   = os.path.join(current_dir, 'Siclo1.urdf')
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"Siclo1.urdf not found at {urdf_path}")

        p.setAdditionalSearchPath(os.path.dirname(urdf_path))
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0.0, 0.0, URDF_SPAWN_Z],
            physicsClientId=client,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        joint_map = _build_joint_map(robot_id, client)

        if not headless:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5, cameraYaw=90, cameraPitch=-20,
                cameraTargetPosition=[0.0, 0.0, 0.5],
                physicsClientId=client,
            )

        # Optional video recording — TinyRenderer, software-only, no GPU needed
        recorder = None
        if record:
            from recorder import VideoRecorder
            recorder = VideoRecorder(
                physics_client=client,
                session_path=session_path,
            )
            recorder.start()
            print(f"[replay] Recording to {recorder.video_path}")

        frame_dt = TARGET_DT / max(speed, 1e-6)  # s per frame at chosen speed

        for i, row in enumerate(poses):
            t_frame = time.perf_counter()

            base_pos = tuple(float(v) for v in row[_COL_POS])
            base_orn = tuple(float(v) for v in row[_COL_ORN])

            p.resetBasePositionAndOrientation(
                robot_id, base_pos, base_orn, physicsClientId=client)

            angles = row[_COL_J_START:_COL_J_END]
            for j, jname in enumerate(_REPLAY_JOINT_NAMES):
                jid = joint_map.get(jname)
                if jid is not None:
                    p.resetJointState(
                        robot_id, jid, float(angles[j]), 0.0,
                        physicsClientId=client,
                    )

            p.stepSimulation(physicsClientId=client)

            elapsed   = time.perf_counter() - t_frame
            remaining = frame_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

        if recorder is not None:
            video_path = recorder.stop()
            recorder.join(timeout=5.0)
            if os.path.isfile(video_path):
                print(f"[replay] Video saved: {video_path}")
            else:
                print(f"[replay][WARN] walk.mp4 not found at {video_path}")

        if not headless:
            print("[replay] Playback complete. Press Ctrl-C to close.")
            try:
                while p.isConnected(physicsClientId=client):
                    time.sleep(0.05)
            except KeyboardInterrupt:
                pass

    finally:
        if p.isConnected(physicsClientId=client):
            p.disconnect(physicsClientId=client)


# ── CLI entry point ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Siclo1 post-run 3D session replay'
    )
    parser.add_argument('session', help='Path to session folder (contains poses.npy)')
    parser.add_argument('--speed', type=float, default=1.0, metavar='X',
                        help='Playback speed multiplier (default: 1.0 = real-time)')
    parser.add_argument('--record', action='store_true',
                        help='Record walk.mp4 to the session folder')
    parser.add_argument('--headless', action='store_true',
                        help='Use p.DIRECT (no window); use with --record for MP4-only')
    args = parser.parse_args()
    replay(
        session_path=args.session,
        speed=args.speed,
        record=args.record,
        headless=args.headless,
    )


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest test_replay.py -v
```

Expected: 4 PASSED

- [ ] **Step 5: Commit**

```bash
git add replay.py test_replay.py
git commit -m "feat: add replay.py — post-run 3D pose playback with optional MP4 recording"
```

---

### Task 6: analyze.py

**Files:**
- Create: `analyze.py`
- Create: `test_analyze.py`

**Background:** Reads the existing `telemetry.csv` (16-column format, already produced by `TelemetryThread`) and generates 4 matplotlib PNG figures. No PyBullet dependency. Uses `matplotlib.use('Agg')` when `--show` is not requested so it runs fully headless.

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_analyze.py`:

```python
"""Tests for analyze.py — post-run telemetry plot generator."""
import os
import tempfile
import pytest


_CSV_HEADER = (
    "timestamp_s,cycle,error_code,com_x,com_y,com_z,"
    "left_contact,right_contact,stability_status,"
    "left_force_n,right_force_n,stability_margin_m,"
    "compute_us,extra_0,extra_1,extra_2"
)


def _write_telemetry_csv(tmpdir: str, n_rows: int = 10) -> str:
    """Write a minimal telemetry.csv for testing."""
    rows = []
    for i in range(n_rows):
        rows.append(','.join([
            f'{i * 0.01:.6g}', f'{i}', '0',       # timestamp, cycle, error
            f'{i * 0.001:.6g}', '0.0', '0.88',    # com x, y, z
            '3', '3', '1',                          # contacts (CONFIRMED), stab (STABLE)
            '39.0', '39.0', '0.02',                 # forces, margin
            '4500', '0', '0', '0',                  # compute, extras
        ]))
    path = os.path.join(tmpdir, 'telemetry.csv')
    with open(path, 'w') as f:
        f.write(_CSV_HEADER + '\n')
        f.write('\n'.join(rows) + '\n')
    return tmpdir


def test_analyze_creates_four_png_files():
    """analyze() must produce exactly these 4 PNGs in the session folder."""
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_telemetry_csv(tmpdir)
        from analyze import analyze
        analyze(tmpdir, show=False)
        for name in ('com_trajectory.png', 'contact_forces.png',
                     'timing.png', 'stability.png'):
            assert os.path.isfile(os.path.join(tmpdir, name)), \
                f"Expected {name} to exist after analyze()"


def test_analyze_raises_if_csv_missing():
    """FileNotFoundError with 'telemetry.csv' in message when file absent."""
    with tempfile.TemporaryDirectory() as tmpdir:
        from analyze import analyze
        with pytest.raises(FileNotFoundError, match='telemetry.csv'):
            analyze(tmpdir, show=False)


def test_analyze_handles_violation_rows():
    """Rows with error_code > 0 (timing violations) must not crash analyze()."""
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, 'telemetry.csv')
        with open(path, 'w') as f:
            f.write(_CSV_HEADER + '\n')
            # Two normal rows, one with error_code=1 (ERR_TIMING_VIOLATION)
            for err in ('0', '1', '0'):
                f.write(f'0.01,1,{err},0.0,0.0,0.88,3,3,1,39.0,39.0,0.02,5000,0,0,0\n')
        from analyze import analyze
        analyze(tmpdir, show=False)
        assert os.path.isfile(os.path.join(tmpdir, 'timing.png'))


def test_analyze_handles_single_row_csv():
    """A CSV with only one data row must not crash (avoids 1-D array issues)."""
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, 'telemetry.csv')
        with open(path, 'w') as f:
            f.write(_CSV_HEADER + '\n')
            f.write('0.01,1,0,0.0,0.0,0.88,3,3,1,39.0,39.0,0.02,4500,0,0,0\n')
        from analyze import analyze
        analyze(tmpdir, show=False)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest test_analyze.py -v
```

Expected: 4 FAILED — `analyze` module not found

- [ ] **Step 3: Implement `analyze.py`**

Create `/home/notlord/ros2_ws/Siclo1_V1/analyze.py`:

```python
"""
================================================================================
PROJECT SICLO1 — POST-RUN TELEMETRY ANALYSIS
================================================================================

Reads telemetry.csv from a session folder and produces 4 matplotlib figures.
No PyBullet dependency. Runs fully headless (Agg backend) unless --show is used.

Output files written to session folder:
  com_trajectory.png  — COM x/y/z vs time
  contact_forces.png  — left + right foot force (N) vs time
  timing.png          — compute time (µs) per cycle + violation markers
  stability.png       — stability margin + stability status vs time

Usage:
    python3 analyze.py sessions/2026-04-13_14-30-00/
    python3 analyze.py sessions/2026-04-13_14-30-00/ --show

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import argparse
import os
import numpy as np

# CSV column indices — must match CSV_HEADER in telemetry.py
_COL_TIME    = 0
_COL_CYCLE   = 1
_COL_ERR     = 2
_COL_COM_X   = 3
_COL_COM_Y   = 4
_COL_COM_Z   = 5
_COL_L_CONT  = 6
_COL_R_CONT  = 7
_COL_STAB    = 8
_COL_L_FORCE = 9
_COL_R_FORCE = 10
_COL_MARGIN  = 11
_COL_COMP_US = 12


def analyze(session_path: str, show: bool = False) -> None:
    """Generate 4 PNG analysis figures from telemetry.csv.

    session_path : folder containing telemetry.csv
    show         : if True, call plt.show() after saving (requires X11 display)
    """
    csv_path = os.path.join(session_path, 'telemetry.csv')
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(
            f"telemetry.csv not found in {session_path}"
        )

    import matplotlib
    if not show:
        matplotlib.use('Agg')   # headless — must be set before pyplot import
    import matplotlib.pyplot as plt

    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]  # single-row CSV → (1, 16) array

    t       = data[:, _COL_TIME]
    com_x   = data[:, _COL_COM_X]
    com_y   = data[:, _COL_COM_Y]
    com_z   = data[:, _COL_COM_Z]
    l_force = data[:, _COL_L_FORCE]
    r_force = data[:, _COL_R_FORCE]
    stab    = data[:, _COL_STAB]
    margin  = data[:, _COL_MARGIN]
    comp_us = data[:, _COL_COMP_US]
    err     = data[:, _COL_ERR]

    # ── Figure 1: COM trajectory ──────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Centre of Mass Trajectory')
    for ax, col, ylabel, color in zip(
        axes,
        [com_x, com_y, com_z],
        ['X (m)', 'Y (m)', 'Z (m)'],
        ['tab:blue', 'tab:orange', 'tab:green'],
    ):
        ax.plot(t, col, color=color, linewidth=0.8)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    _save_fig(fig, session_path, 'com_trajectory.png')
    plt.close(fig)

    # ── Figure 2: Contact forces ──────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, l_force, label='Left (N)',  color='tab:blue',   linewidth=0.8)
    ax.plot(t, r_force, label='Right (N)', color='tab:orange', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Contact Force (N)')
    ax.set_title('Foot Contact Forces')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'contact_forces.png')
    plt.close(fig)

    # ── Figure 3: Timing ──────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, comp_us, color='tab:purple', linewidth=0.5, label='Compute (µs)')
    viol_mask = err > 0
    if viol_mask.any():
        ax.scatter(t[viol_mask], comp_us[viol_mask],
                   color='red', s=12, zorder=5, label='Violation')
    ax.axhline(10_000, color='red', linestyle='--', linewidth=0.8,
               label='10 ms limit')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Compute Time (µs)')
    ax.set_title('Cycle Timing')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'timing.png')
    plt.close(fig)

    # ── Figure 4: Stability ───────────────────────────────────────────────────
    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(t, margin, color='tab:green', linewidth=0.8, label='Margin (m)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Stability Margin (m)', color='tab:green')
    ax1.tick_params(axis='y', labelcolor='tab:green')
    ax2 = ax1.twinx()
    ax2.plot(t, stab, color='tab:red', linewidth=0.5, alpha=0.6, label='Status')
    ax2.set_ylabel('Stability Status', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    ax1.set_title('Stability Margin & Status')
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'stability.png')
    plt.close(fig)

    if show:
        plt.show()


def _save_fig(fig, session_path: str, filename: str) -> None:
    path = os.path.join(session_path, filename)
    fig.savefig(path, dpi=120)
    print(f"[analyze] Saved {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Siclo1 post-run telemetry analysis'
    )
    parser.add_argument('session', help='Path to session folder (contains telemetry.csv)')
    parser.add_argument('--show', action='store_true',
                        help='Open figures interactively after saving (requires X11)')
    args = parser.parse_args()
    analyze(args.session, show=args.show)


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest test_analyze.py -v
```

Expected: 4 PASSED

- [ ] **Step 5: Commit**

```bash
git add analyze.py test_analyze.py
git commit -m "feat: add analyze.py — post-run telemetry plots (COM, forces, timing, stability)"
```

---

### Task 7: Full regression and acceptance verification

**Files:** No new files. Runs existing test suite + acceptance checks from the spec.

- [ ] **Step 1: Run the full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest test_heartbeat_strict.py test_gui_sync_fix.py test_pose_logger.py test_replay.py test_analyze.py test_grf.py test_stance_anchor.py test_step_phase_guards.py test_gait_planner_fsm.py test_contact_tick_decay.py -v
```

Expected: all PASSED

- [ ] **Step 2: Acceptance check — headless run produces poses.npy**

```bash
python3 main.py --duration 200 2>&1 | grep -E "Pose log|cycles"
```

Expected output includes: `[Siclo1] Pose log saved: sessions/.../poses.npy  (200 frames)`

- [ ] **Step 3: Acceptance check — analyze.py on that session**

```bash
SESSION=$(ls -td sessions/*/ | head -1)
python3 analyze.py "$SESSION"
ls "$SESSION"*.png
```

Expected: 4 PNG files listed

- [ ] **Step 4: Acceptance check — headless replay**

```bash
SESSION=$(ls -td sessions/*/ | head -1)
python3 replay.py "$SESSION" --headless --speed 100
```

Expected: completes without error, prints `[replay] X frames loaded`

- [ ] **Step 5: Final commit**

```bash
git add .
git commit -m "test: full regression pass — GUI visual pipeline complete"
```
