# Offline Video Review Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move GUI rendering off the 100 Hz hot path into a `GUISyncThread`, automatically recording a frame-accurate MP4 of every `--gui --walk` mission stored in the session folder.

**Architecture:** A new `PoseSnapshot` dataclass and `GUISyncThread` class are added to `HeartBeat.py`. The 100 Hz loop writes the latest robot pose into a single shared slot (non-blocking); the thread reads it at `--viz-hz` fps (default 30), mirrors it to the GUI client, calls `p.stepSimulation(gui_client)` to capture each MP4 frame, and polls mission state to start/stop `p.startStateLogging`. `extra_0` in each telemetry row stores the current MP4 frame index for cycle→frame lookup.

**Tech Stack:** Python 3.10+, PyBullet, threading, dataclasses, subprocess, pytest

---

## File Map

| File | Change |
|------|--------|
| `HeartBeat.py` | Add `PoseSnapshot` dataclass, add `GUISyncThread` class, modify `Siclo1Controller.__init__`, `step()`, `finalize_telemetry()`, `shutdown()`, remove `_sync_gui()` |
| `telemetry.py` | Add `TelemetryThread.set_video_meta()`, update `SessionLogger.write_summary()` with VIDEO section, add `_write_open_script()` |
| `main.py` | Change `--viz-hz` default 10→30, add `import subprocess`, add `_open_session_artifacts()`, call it after `finalize_telemetry()` |
| `test_gui_sync_thread.py` | New — unit tests for `PoseSnapshot` and `GUISyncThread` |
| `test_telemetry.py` | Extend — tests for `set_video_meta()`, VIDEO summary section, `open_session.sh` |
| `test_main_walk_arg.py` | Extend — tests for new default and `_open_session_artifacts()` |

---

## Task 1: `PoseSnapshot` dataclass + `GUISyncThread` skeleton

**Files:**
- Modify: `HeartBeat.py:29-35` (imports)
- Modify: `HeartBeat.py:395` (insert new classes before `Siclo1Controller`)
- Create: `test_gui_sync_thread.py`

- [ ] **Step 1: Write the failing tests**

Create `test_gui_sync_thread.py`:

```python
"""Tests for PoseSnapshot and GUISyncThread in HeartBeat.py."""
import threading
import time
import pytest
from unittest.mock import patch, MagicMock

from shared_state import MissionState


def _make_snapshot(mission_state=MissionState.IDLE, emergency=False):
    from HeartBeat import PoseSnapshot
    return PoseSnapshot(
        base_pos=(0.0, 0.0, 0.88),
        base_orn=(0.0, 0.0, 0.0, 1.0),
        joint_states={'Left_Hip_Forwards': (0.1, 0.0)},
        link_positions={'Left_Upper_Leg_1': [0.0, 0.0, 0.5]},
        mission_state=mission_state,
        emergency_stop=emergency,
    )


def _make_thread(walk_active=False, viz_fps=30, session_path='/tmp/test_session'):
    from HeartBeat import GUISyncThread
    with patch('HeartBeat.p'):
        t = GUISyncThread(
            gui_client=5,
            gui_robot_id=1,
            joint_list=[('Left_Hip_Forwards', 0)],
            session_path=session_path,
            viz_fps=viz_fps,
            walk_active=walk_active,
            visualizer=None,
            left_hip_link='Left_Upper_Leg_1',
            right_hip_link='Right_Upper_Leg_1',
        )
    return t


# --------------------------------------------------------------------------- #
class TestPoseSnapshot:
    def test_fields_accessible(self):
        snap = _make_snapshot()
        assert snap.base_pos == (0.0, 0.0, 0.88)
        assert snap.base_orn == (0.0, 0.0, 0.0, 1.0)
        assert snap.joint_states == {'Left_Hip_Forwards': (0.1, 0.0)}
        assert snap.mission_state == MissionState.IDLE
        assert snap.emergency_stop is False

    def test_ramp_state(self):
        snap = _make_snapshot(MissionState.RAMP)
        assert snap.mission_state == MissionState.RAMP

    def test_emergency_flag(self):
        snap = _make_snapshot(emergency=True)
        assert snap.emergency_stop is True


# --------------------------------------------------------------------------- #
class TestGUISyncThreadSlot:
    def test_initial_frame_count_is_zero(self):
        t = _make_thread()
        assert t.video_frame_count == 0

    def test_initial_slot_is_none(self):
        t = _make_thread()
        assert t._slot is None

    def test_push_pose_updates_slot(self):
        t = _make_thread()
        snap = _make_snapshot(MissionState.WALK)
        t.push_pose(snap)
        assert t._slot is snap

    def test_push_pose_skips_when_lock_held(self):
        t = _make_thread()
        original = _make_snapshot(MissionState.IDLE)
        t._slot = original
        t._lock.acquire()
        try:
            new_snap = _make_snapshot(MissionState.WALK)
            t.push_pose(new_snap)   # lock busy — must skip
        finally:
            t._lock.release()
        assert t._slot is original  # unchanged

    def test_push_pose_overwrites_when_lock_free(self):
        t = _make_thread()
        snap1 = _make_snapshot(MissionState.IDLE)
        snap2 = _make_snapshot(MissionState.WALK)
        t.push_pose(snap1)
        t.push_pose(snap2)
        assert t._slot is snap2

    def test_stop_returns_metadata_no_walk(self):
        t = _make_thread(walk_active=False)
        meta = t.stop()
        assert meta['video_path'] is None
        assert meta['video_fps'] == 30
        assert meta['video_frames'] == 0

    def test_stop_returns_video_path_when_walk(self, tmp_path):
        t = _make_thread(walk_active=True, session_path=str(tmp_path))
        meta = t.stop()
        assert meta['video_path'] is not None
        assert meta['video_path'].endswith('walk.mp4')
        assert meta['video_fps'] == 30
        assert meta['video_frames'] == 0
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py -v 2>&1 | head -30
```

Expected: `ImportError: cannot import name 'PoseSnapshot' from 'HeartBeat'`

- [ ] **Step 3: Add imports to `HeartBeat.py`**

At `HeartBeat.py:29`, the existing import block starts with `import os`. Add `from dataclasses import dataclass` after the stdlib imports and add `MissionState` to the `shared_state` import:

Find this line in `HeartBeat.py`:
```python
import os
import sys
import time
import threading
import numpy as np
import pybullet as p
import pybullet_data
from typing import Optional, Tuple, Dict
```

Replace with:
```python
import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

import numpy as np
import pybullet as p
import pybullet_data
```

Then find the shared_state import block:
```python
from shared_state import (
    shared_state,
    Siclo1State,
    TelemetryRingBuffer,
    SystemStatus,
    ContactState,
    URDF_JOINT_NAMES,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
    ERR_TIMING_VIOLATION,
    ERR_MID_CYCLE_OVERRUN,
)
```

Replace with:
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
)
```

- [ ] **Step 4: Add `PoseSnapshot` and `GUISyncThread` skeleton to `HeartBeat.py`**

Insert the following block immediately before the line `class Siclo1Controller:` (currently around line 402):

```python
# ============================================================================
# POSE SNAPSHOT  — immutable robot state for GUI thread handoff
# ============================================================================

@dataclass
class PoseSnapshot:
    """Frozen robot state written by the 100 Hz loop; read by GUISyncThread.

    One slot shared between producer and consumer.
    Producer writes at 100 Hz (non-blocking); consumer reads at viz_fps Hz.
    """
    base_pos:       tuple        # (x, y, z) world position
    base_orn:       tuple        # quaternion (x, y, z, w)
    joint_states:   dict         # {joint_name: (position_rad, velocity_rad_s)}
    link_positions: dict         # {link_name: [x, y, z]} for DebugVisualizer
    mission_state:  MissionState
    emergency_stop: bool


# ============================================================================
# GUI SYNC THREAD  — owns gui_client after init; decouples V-Sync from 100 Hz
# ============================================================================

class GUISyncThread(threading.Thread):
    """Daemon thread: mirrors robot pose to GUI client at viz_fps Hz.

    Renders MP4 frames via p.stepSimulation(gui_client) — the only trigger
    for PyBullet's internal video encoder.  Starts/stops MP4 logging on
    mission state transitions.  Never called from the 100 Hz hot path.
    """

    daemon = True

    def __init__(
        self,
        gui_client:     int,
        gui_robot_id:   int,
        joint_list:     list,
        session_path:   str,
        viz_fps:        int,
        walk_active:    bool,
        visualizer:     Any,
        left_hip_link:  str,
        right_hip_link: str,
    ) -> None:
        super().__init__(name='GUISyncThread')
        self._gui_client    = gui_client
        self._gui_robot_id  = gui_robot_id
        self._joint_list    = joint_list
        self._viz_fps       = viz_fps
        self._walk_active   = walk_active
        self._visualizer    = visualizer
        self._left_hip_link  = left_hip_link
        self._right_hip_link = right_hip_link

        self._video_path: str = os.path.join(session_path, "walk.mp4")

        # Slot — one shared PoseSnapshot, protected by a lock
        self._lock:  threading.Lock             = threading.Lock()
        self._slot:  Optional[PoseSnapshot]     = None

        # MP4 state
        self._log_id:  Optional[int]  = None
        self._prev_mission_state: MissionState  = MissionState.IDLE

        # Frame counter — GIL-safe int read from step()
        self._video_frame_count: int = 0

        self._stop_event = threading.Event()

        # Prevent GUI client from self-advancing time
        p.setRealTimeSimulation(0, physicsClientId=self._gui_client)

    # ------------------------------------------------------------------ #
    @property
    def video_frame_count(self) -> int:
        """Current captured frame count. GIL-safe int read from 100 Hz loop."""
        return self._video_frame_count

    # ------------------------------------------------------------------ #
    def push_pose(self, snapshot: PoseSnapshot) -> None:
        """Write latest pose snapshot. Non-blocking — skips if thread is rendering."""
        if self._lock.acquire(blocking=False):
            self._slot = snapshot
            self._lock.release()

    # ------------------------------------------------------------------ #
    def stop(self) -> dict:
        """Signal thread to stop. Close any open MP4. Return video metadata."""
        self._stop_event.set()
        if self._log_id is not None:
            try:
                p.stopStateLogging(self._log_id, physicsClientId=self._gui_client)
            except Exception:
                pass
            self._log_id = None
        return {
            'video_path':   self._video_path if self._walk_active else None,
            'video_frames': self._video_frame_count,
            'video_fps':    self._viz_fps,
        }

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        """Thread loop — placeholder; filled in Task 3."""
        pass

    # ------------------------------------------------------------------ #
    def _handle_mp4_lifecycle(self, snapshot: PoseSnapshot) -> None:
        """Placeholder — filled in Task 2."""
        pass
```

- [ ] **Step 5: Run tests**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py -v
```

Expected: all 11 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add HeartBeat.py test_gui_sync_thread.py
git commit -m "feat: add PoseSnapshot dataclass and GUISyncThread skeleton"
```

---

## Task 2: `GUISyncThread._handle_mp4_lifecycle()`

**Files:**
- Modify: `HeartBeat.py` — implement `_handle_mp4_lifecycle`
- Modify: `test_gui_sync_thread.py` — add MP4 lifecycle tests

- [ ] **Step 1: Write the failing tests**

Append to `test_gui_sync_thread.py`:

```python
# --------------------------------------------------------------------------- #
class TestGUISyncThreadMP4Lifecycle:

    def test_start_logging_called_on_ramp_entry(self):
        """startStateLogging fires exactly once on IDLE → RAMP."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            mock_p.startStateLogging.return_value = 42
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.IDLE
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_called_once_with(
                1, t._video_path, physicsClientId=5
            )
            assert t._log_id == 42

    def test_start_logging_not_called_when_already_ramp(self):
        """No duplicate startStateLogging if already in RAMP."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.RAMP   # already in RAMP
            t._log_id = 10                               # already recording
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_not_called()

    def test_stop_logging_called_on_idle_reentry(self):
        """stopStateLogging fires on STOP → IDLE."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 42
            t._prev_mission_state = MissionState.STOP
            snap = _make_snapshot(MissionState.IDLE)
            t._handle_mp4_lifecycle(snap)

            mock_p.stopStateLogging.assert_called_once_with(42, physicsClientId=5)
            assert t._log_id is None

    def test_stop_logging_called_on_emergency(self):
        """stopStateLogging fires immediately when emergency_stop=True."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 77
            t._prev_mission_state = MissionState.WALK
            snap = _make_snapshot(MissionState.WALK, emergency=True)
            t._handle_mp4_lifecycle(snap)

            mock_p.stopStateLogging.assert_called_once_with(77, physicsClientId=5)
            assert t._log_id is None

    def test_no_recording_when_walk_inactive(self):
        """No startStateLogging when walk_active=False."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=False, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.IDLE
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_not_called()

    def test_stop_safety_net_closes_open_log(self):
        """stop() calls stopStateLogging if _log_id is still open."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 99
            t.stop()
            mock_p.stopStateLogging.assert_called_once_with(99, physicsClientId=5)
            assert t._log_id is None
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py::TestGUISyncThreadMP4Lifecycle -v
```

Expected: all 6 FAIL with `AssertionError` (mock not called — method is a placeholder).

- [ ] **Step 3: Implement `_handle_mp4_lifecycle` in `HeartBeat.py`**

Find the placeholder:
```python
    def _handle_mp4_lifecycle(self, snapshot: PoseSnapshot) -> None:
        """Placeholder — filled in Task 2."""
        pass
```

Replace with:
```python
    def _handle_mp4_lifecycle(self, snapshot: PoseSnapshot) -> None:
        """Start/stop MP4 logging on mission state transitions.

        Edges watched:
          ANY → RAMP  : start recording (if walk_active and not already recording)
          ANY → IDLE  : stop recording  (mission completed cleanly)
          emergency   : stop recording  (safe freeze triggered)
        Called by run() before updating _prev_mission_state.
        """
        if not self._walk_active:
            return

        prev = self._prev_mission_state
        curr = snapshot.mission_state

        # Start on RAMP entry
        if prev != MissionState.RAMP and curr == MissionState.RAMP:
            if self._log_id is None:
                self._log_id = p.startStateLogging(
                    p.STATE_LOGGING_VIDEO_MP4,
                    self._video_path,
                    physicsClientId=self._gui_client,
                )

        # Stop on clean IDLE re-entry or emergency
        if self._log_id is not None:
            stop_now = (
                (prev != MissionState.IDLE and curr == MissionState.IDLE)
                or snapshot.emergency_stop
            )
            if stop_now:
                p.stopStateLogging(self._log_id, physicsClientId=self._gui_client)
                self._log_id = None
```

- [ ] **Step 4: Run all tests**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py -v
```

Expected: all 17 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py test_gui_sync_thread.py
git commit -m "feat: implement GUISyncThread MP4 lifecycle (RAMP→start, IDLE→stop)"
```

---

## Task 3: `GUISyncThread.run()` — mirror loop + frame counter

**Files:**
- Modify: `HeartBeat.py` — implement `run()` and `_mirror_pose()`
- Modify: `test_gui_sync_thread.py` — add thread integration tests

- [ ] **Step 1: Write the failing tests**

Append to `test_gui_sync_thread.py`:

```python
# --------------------------------------------------------------------------- #
class TestGUISyncThreadRunLoop:

    def test_run_increments_frame_count(self):
        """Thread increments video_frame_count on each render cycle."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            mock_p.stepSimulation = MagicMock()
            mock_p.resetBasePositionAndOrientation = MagicMock()
            mock_p.resetJointState = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=0, gui_robot_id=0,
                joint_list=[('Left_Hip_Forwards', 2)],
                session_path='/tmp/ts', viz_fps=60,   # 60fps → ~6 renders in 100ms
                walk_active=False, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.push_pose(_make_snapshot())
            t.start()
            time.sleep(0.15)   # allow ~9 frames at 60fps
            t.stop()
            t.join(timeout=1.0)

            assert t.video_frame_count >= 4

    def test_run_calls_step_simulation(self):
        """Thread calls p.stepSimulation(gui_client) each render cycle."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            mock_p.stepSimulation = MagicMock()
            mock_p.resetBasePositionAndOrientation = MagicMock()
            mock_p.resetJointState = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=7, gui_robot_id=1,
                joint_list=[],
                session_path='/tmp/ts', viz_fps=60,
                walk_active=False, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.push_pose(_make_snapshot())
            t.start()
            time.sleep(0.1)
            t.stop()
            t.join(timeout=1.0)

            calls = mock_p.stepSimulation.call_args_list
            assert any(c == ((), {'physicsClientId': 7}) for c in calls)

    def test_run_stops_cleanly_when_no_snapshot(self):
        """Thread exits cleanly even if no snapshot was ever pushed."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=0, gui_robot_id=0,
                joint_list=[], session_path='/tmp/ts', viz_fps=60,
                walk_active=False, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.start()
            time.sleep(0.05)
            t.stop()
            t.join(timeout=1.0)
            assert not t.is_alive()
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py::TestGUISyncThreadRunLoop -v
```

Expected: `test_run_increments_frame_count` FAIL (count stays 0), others may timeout.

- [ ] **Step 3: Implement `run()` and `_mirror_pose()` in `HeartBeat.py`**

Find the placeholder:
```python
    def run(self) -> None:
        """Thread loop — placeholder; filled in Task 3."""
        pass
```

Replace with:
```python
    def run(self) -> None:
        """~viz_fps Hz render loop. Mirrors pose, captures MP4 frame, polls mission."""
        period = 1.0 / self._viz_fps
        while not self._stop_event.is_set():
            loop_start = time.perf_counter()

            with self._lock:
                snapshot = self._slot

            if snapshot is not None:
                self._mirror_pose(snapshot)
                p.stepSimulation(physicsClientId=self._gui_client)
                self._video_frame_count += 1
                self._handle_mp4_lifecycle(snapshot)
                self._prev_mission_state = snapshot.mission_state

            elapsed = time.perf_counter() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _mirror_pose(self, snapshot: PoseSnapshot) -> None:
        """Apply snapshot to GUI robot; update DebugVisualizer if present."""
        try:
            p.resetBasePositionAndOrientation(
                self._gui_robot_id,
                snapshot.base_pos,
                snapshot.base_orn,
                physicsClientId=self._gui_client,
            )
            for jname, jid in self._joint_list:
                pos, vel = snapshot.joint_states.get(jname, (0.0, 0.0))
                p.resetJointState(
                    self._gui_robot_id, jid, pos, vel,
                    physicsClientId=self._gui_client,
                )
            if self._visualizer is not None:
                lp = snapshot.link_positions
                left_hip  = tuple(lp.get(self._left_hip_link,  [0.0, 0.0, 0.0]))
                right_hip = tuple(lp.get(self._right_hip_link, [0.0, 0.0, 0.0]))
                self._visualizer.update(shared_state, left_hip, right_hip)
        except Exception:
            pass  # GUI mirror is non-critical
```

- [ ] **Step 4: Run all tests**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gui_sync_thread.py -v
```

Expected: all 20 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py test_gui_sync_thread.py
git commit -m "feat: implement GUISyncThread run loop with pose mirroring and frame counter"
```

---

## Task 4: Wire `GUISyncThread` into `Siclo1Controller`

**Files:**
- Modify: `HeartBeat.py` — `Siclo1Controller.__init__`, `step()`, remove `_sync_gui()`

No new test file — integration is verified by running the existing full test suite.

- [ ] **Step 1: Initialize `GUISyncThread` in `Siclo1Controller.__init__`**

In `HeartBeat.py`, find the block that creates `self._visualizer` (around line 527):

```python
        # Debug visualiser — GUI mode only
        if self.gui_client is not None:
            from viz.debug_markers import DebugVisualizer
            self._visualizer = DebugVisualizer(self.gui_client)
```

Replace with:

```python
        # Debug visualiser — GUI mode only
        if self.gui_client is not None:
            from viz.debug_markers import DebugVisualizer
            self._visualizer = DebugVisualizer(self.gui_client)

        # GUISyncThread — replaces _sync_gui(); owns gui_client after this point
        self._gui_sync_thread: Optional[GUISyncThread] = None
        if self.gui_client is not None and self._gui_robot_id >= 0:
            viz_fps = max(1, 100 // viz_decimation)   # e.g. decimation=3 → 33fps
            self._gui_sync_thread = GUISyncThread(
                gui_client    = self.gui_client,
                gui_robot_id  = self._gui_robot_id,
                joint_list    = self.pybullet._joint_list,
                session_path  = self._telemetry_thread.session_path,
                viz_fps       = viz_fps,
                walk_active   = walk_distance is not None,
                visualizer    = self._visualizer,
                left_hip_link = self._left_hip_link,
                right_hip_link= self._right_hip_link,
            )
            self._gui_sync_thread.start()
```

- [ ] **Step 2: Replace hot-path GUI sync in `step()`**

In `HeartBeat.py`, find the existing GUI sync block at the end of `step()` (around line 679):

```python
        # 18. Optional GUI sync (decimated — every viz_decimation cycles)
        if (self.gui_client is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._sync_gui()
```

Replace with:

```python
        # 18. Push pose snapshot to GUI sync thread (non-blocking)
        if self._gui_sync_thread is not None:
            self._gui_sync_thread.push_pose(PoseSnapshot(
                base_pos       = tuple(shared_state.base_position),
                base_orn       = tuple(shared_state.base_orientation),
                joint_states   = {n: (shared_state.joint_positions.get(n, 0.0),
                                      shared_state.joint_velocities.get(n, 0.0))
                                  for n, _ in self.pybullet._joint_list},
                link_positions = dict(shared_state.link_positions),
                mission_state  = shared_state.mission_state,
                emergency_stop = shared_state.emergency_stop_triggered,
            ))
```

- [ ] **Step 3: Write MP4 frame index to telemetry row**

In `step()`, find this line (around line 676):

```python
        shared_state.telemetry.write(row)
```

Insert before it:

```python
        row[13] = (self._gui_sync_thread.video_frame_count
                   if self._gui_sync_thread is not None else 0)  # MP4 frame index
        shared_state.telemetry.write(row)
```

- [ ] **Step 4: Remove `_sync_gui()` method**

Find and delete the entire `_sync_gui` method (approximately lines 686–713):

```python
    # ------------------------------------------------------------------ #
    def _sync_gui(self) -> None:
        """Mirror joint states to the GUI client at decimated rate."""
        if self._gui_robot_id < 0:
            return  # GUI robot not loaded — skip silently
        try:
            rid_phys = self.pybullet.robot_id
            pc_phys  = self.physics_client
            pc_gui   = self.gui_client

            # Mirror base pose
            pos, orn = p.getBasePositionAndOrientation(
                rid_phys, physicsClientId=pc_phys)
            p.resetBasePositionAndOrientation(
                self._gui_robot_id, pos, orn, physicsClientId=pc_gui)

            # Mirror joint positions
            for jname, jid in self.pybullet._joint_list:
                js = p.getJointState(rid_phys, jid, physicsClientId=pc_phys)
                p.resetJointState(self._gui_robot_id, jid, js[0], js[1],
                                  physicsClientId=pc_gui)
            # Update debug visualisation (annulus arcs + hip→foot vectors)
            if self._visualizer is not None:
                lp = self.shared_state.link_positions
                left_hip  = tuple(lp.get(self._left_hip_link,  [0.0, 0.0, 0.0]))
                right_hip = tuple(lp.get(self._right_hip_link, [0.0, 0.0, 0.0]))
                self._visualizer.update(self.shared_state, left_hip, right_hip)
        except Exception:
            pass  # GUI sync is non-critical
```

- [ ] **Step 5: Run the full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest -v --ignore=test_gui_sync_thread.py 2>&1 | tail -20
```

Expected: all previously passing tests still PASS. No regressions.

- [ ] **Step 6: Commit**

```bash
git add HeartBeat.py
git commit -m "feat: wire GUISyncThread into Siclo1Controller; remove _sync_gui() from hot path"
```

---

## Task 5: `finalize_telemetry()` sequencing + `TelemetryThread.set_video_meta()`

**Files:**
- Modify: `HeartBeat.py` — `finalize_telemetry()`, `shutdown()`
- Modify: `telemetry.py` — `TelemetryThread.__init__`, add `set_video_meta()`
- Modify: `test_telemetry.py` — add `set_video_meta` tests

- [ ] **Step 1: Write the failing tests**

Append to `test_telemetry.py`:

```python
from telemetry import TelemetryThread
from unittest.mock import MagicMock


def _make_state():
    """Minimal shared_state mock for TelemetryThread."""
    from shared_state import shared_state
    return shared_state


def test_set_video_meta_stored(tmp_path):
    """set_video_meta() stores keys that are later included in summary."""
    state = _make_state()
    t = TelemetryThread(state)
    t.set_video_meta({
        'video_path':   str(tmp_path / 'walk.mp4'),
        'video_frames': 150,
        'video_fps':    30,
    })
    assert t._video_meta['video_path'] == str(tmp_path / 'walk.mp4')
    assert t._video_meta['video_frames'] == 150
    assert t._video_meta['video_fps'] == 30


def test_set_video_meta_empty_by_default():
    """_video_meta is empty dict before set_video_meta() is called."""
    state = _make_state()
    t = TelemetryThread(state)
    assert t._video_meta == {}
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_telemetry.py::test_set_video_meta_stored test_telemetry.py::test_set_video_meta_empty_by_default -v
```

Expected: `AttributeError: 'TelemetryThread' object has no attribute '_video_meta'`

- [ ] **Step 3: Add `_video_meta` and `set_video_meta()` to `TelemetryThread`**

In `telemetry.py`, find `TelemetryThread.__init__` and add after `self._violations: int = 0`:

```python
        # Video metadata — populated by Siclo1Controller before stop()
        self._video_meta: dict = {}
```

Below `__init__`, after the `session_path` property, add:

```python
    def set_video_meta(self, meta: dict) -> None:
        """Store video recording metadata for inclusion in summary.txt.

        Must be called BEFORE stop() so the data is available when
        TelemetryThread.run() calls write_summary() in its finally block.
        """
        self._video_meta = meta
```

In `TelemetryThread.run()`, find the finally block:

```python
        finally:
            # Tail drain + summary ALWAYS execute, even on exception
            self._drain()
            stats = self.get_summary_stats()
            stats['coded_errors'] = self._state._error_write_idx
            self._session.write_summary(stats)
```

Replace with:

```python
        finally:
            # Tail drain + summary ALWAYS execute, even on exception
            self._drain()
            stats = self.get_summary_stats()
            stats['coded_errors'] = self._state._error_write_idx
            stats.update(self._video_meta)  # merge video keys if present
            self._session.write_summary(stats)
```

- [ ] **Step 4: Update `finalize_telemetry()` in `HeartBeat.py`**

Find the existing `finalize_telemetry()`:

```python
    def finalize_telemetry(self) -> None:
        """Stop telemetry thread and flush/close the CSV.

        Call BEFORE any hold loop and BEFORE shutdown().
        shutdown() checks is_alive() so calling both is safe.
        """
        self._telemetry_thread.stop()
        self._telemetry_thread.join(timeout=2.0)
```

Replace with:

```python
    def finalize_telemetry(self) -> None:
        """Close MP4, inject video metadata, then flush telemetry CSV.

        Ordering is critical:
          1. Stop GUISyncThread first — closes MP4 cleanly, returns metadata.
          2. Inject metadata into TelemetryThread before summary is written.
          3. Stop TelemetryThread — final drain + write_summary() fires here.
        Call BEFORE any hold loop and BEFORE shutdown().
        """
        # 1. Close MP4 and collect metadata
        if self._gui_sync_thread is not None:
            video_meta = self._gui_sync_thread.stop()
            self._gui_sync_thread.join(timeout=2.0)
            # 2. Inject into telemetry before summary is written
            if video_meta.get('video_path') is not None:
                self._telemetry_thread.set_video_meta(video_meta)

        # 3. Final drain + write_summary
        self._telemetry_thread.stop()
        self._telemetry_thread.join(timeout=2.0)
```

- [ ] **Step 5: Clean up `shutdown()` in `HeartBeat.py`**

Find `shutdown()`:

```python
    def shutdown(self) -> None:
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.stop()
            self._telemetry_thread.join(timeout=2.0)
        try:
            p.disconnect(physicsClientId=self.physics_client)
        except Exception:
            pass
        if self.gui_client is not None:
            try:
                p.disconnect(physicsClientId=self.gui_client)
            except Exception:
                pass
        self._telemetry_thread.flush_to_stdout()
```

Replace with:

```python
    def shutdown(self) -> None:
        """Disconnect PyBullet clients. finalize_telemetry() must be called first."""
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.stop()
            self._telemetry_thread.join(timeout=2.0)
        try:
            p.disconnect(physicsClientId=self.physics_client)
        except Exception:
            pass
        if self.gui_client is not None:
            try:
                p.disconnect(physicsClientId=self.gui_client)
            except Exception:
                pass
        self._telemetry_thread.flush_to_stdout()
```

(No `_gui_sync_thread` call here — it was already stopped in `finalize_telemetry()`.)

- [ ] **Step 6: Run full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest -v 2>&1 | tail -25
```

Expected: all tests PASS including the two new telemetry tests.

- [ ] **Step 7: Commit**

```bash
git add HeartBeat.py telemetry.py test_telemetry.py
git commit -m "feat: finalize_telemetry sequencing — GUISyncThread stop before telemetry flush"
```

---

## Task 6: `SessionLogger` VIDEO section + `open_session.sh`

**Files:**
- Modify: `telemetry.py` — `SessionLogger.write_summary()`, add `_write_open_script()`
- Modify: `test_telemetry.py` — add VIDEO section and shell script tests

- [ ] **Step 1: Write the failing tests**

Append to `test_telemetry.py`:

```python
def _stats_base():
    return {
        'mean_dt': 0.002, 'std_dt': 0.0001, 'min_dt': 0.001, 'max_dt': 0.003,
        'jitter_ms': 0.1, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 1050, 'analyzed_cycles': 1000,
        'warmup_cycles': 50, 'coded_errors': 0,
    }


def test_write_summary_includes_video_section(tmp_path):
    """write_summary() writes [VIDEO] section when video_path key is present."""
    logger = SessionLogger(str(tmp_path))
    video_path = os.path.join(logger.session_path, 'walk.mp4')
    stats = _stats_base()
    stats.update({
        'video_path':   video_path,
        'video_frames': 300,
        'video_fps':    30,
    })
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, 'summary.txt')).read()
    assert '[VIDEO]' in content
    assert 'walk.mp4' in content
    assert 'Frames  : 300' in content
    assert 'FPS     : 30' in content
    assert 'Duration: 10.0' in content   # 300/30 = 10.0 s


def test_write_summary_omits_video_section_when_no_path(tmp_path):
    """write_summary() omits [VIDEO] section when video_path is absent."""
    logger = SessionLogger(str(tmp_path))
    logger.write_summary(_stats_base())
    content = open(os.path.join(logger.session_path, 'summary.txt')).read()
    assert '[VIDEO]' not in content


def test_write_summary_omits_video_section_when_path_none(tmp_path):
    """write_summary() omits [VIDEO] section when video_path is None."""
    logger = SessionLogger(str(tmp_path))
    stats = _stats_base()
    stats['video_path'] = None
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, 'summary.txt')).read()
    assert '[VIDEO]' not in content


def test_open_session_sh_written_when_video(tmp_path):
    """write_summary() writes open_session.sh when video_path is present."""
    logger = SessionLogger(str(tmp_path))
    video_path = os.path.join(logger.session_path, 'walk.mp4')
    stats = _stats_base()
    stats.update({'video_path': video_path, 'video_frames': 100, 'video_fps': 30})
    logger.write_summary(stats)
    script = os.path.join(logger.session_path, 'open_session.sh')
    assert os.path.isfile(script)
    content = open(script).read()
    assert 'walk.mp4' in content
    assert 'summary.txt' in content
    assert content.startswith('#!/bin/bash')


def test_open_session_sh_not_written_without_video(tmp_path):
    """write_summary() does NOT write open_session.sh when no video."""
    logger = SessionLogger(str(tmp_path))
    logger.write_summary(_stats_base())
    script = os.path.join(logger.session_path, 'open_session.sh')
    assert not os.path.isfile(script)
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_telemetry.py -k "video or open_session" -v
```

Expected: all 5 FAIL — `[VIDEO]` section does not exist yet.

- [ ] **Step 3: Update `SessionLogger.write_summary()` and add `_write_open_script()`**

In `telemetry.py`, find the `write_summary` method. Locate the lines that build the `lines` list. After the existing `[ERRORS]` block and before the `summary_file.write(...)` call, add the VIDEO section:

Find:
```python
            lines += [
                "",
                "[ERRORS]",
                f"  Coded errors in ring: {coded}",
            ]
            summary_file.write('\n'.join(lines) + '\n')
```

Replace with:
```python
            lines += [
                "",
                "[ERRORS]",
                f"  Coded errors in ring: {coded}",
            ]

            # Optional VIDEO section — only when walk.mp4 was recorded
            video_path = stats.get('video_path')
            if video_path is not None:
                frames   = stats.get('video_frames', 0)
                fps      = stats.get('video_fps', 30)
                duration = frames / fps if fps > 0 else 0.0
                # Show path relative to project root for readability
                project_root = os.path.dirname(os.path.dirname(self._session_path))
                try:
                    rel_path = os.path.relpath(video_path, project_root)
                except ValueError:
                    rel_path = video_path
                lines += [
                    "",
                    "[VIDEO]",
                    f"  File    : {rel_path}",
                    f"  Frames  : {frames}",
                    f"  FPS     : {fps}",
                    f"  Duration: {duration:.1f} s",
                ]

            summary_file.write('\n'.join(lines) + '\n')

            # Shell script fallback for WSL2
            if video_path is not None:
                self._write_open_script()
```

Then add `_write_open_script()` as a new method on `SessionLogger` (insert after `write_summary`):

```python
    def _write_open_script(self) -> None:
        """Write open_session.sh — manual fallback when xdg-open is unavailable."""
        script_path = os.path.join(self._session_path, "open_session.sh")
        try:
            with open(script_path, 'w') as f:
                f.write('#!/bin/bash\n')
                f.write('# Manual fallback for WSL2 environments without xdg-open\n')
                f.write('xdg-open "$(dirname "$0")/walk.mp4" &\n')
                f.write('xdg-open "$(dirname "$0")/summary.txt" &\n')
            os.chmod(script_path, 0o755)
        except OSError:
            pass  # non-critical — simulation is not interrupted
```

- [ ] **Step 4: Run full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest -v 2>&1 | tail -25
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add telemetry.py test_telemetry.py
git commit -m "feat: add VIDEO section to summary.txt and write open_session.sh"
```

---

## Task 7: `main.py` — `--viz-hz` default 30 + `_open_session_artifacts()`

**Files:**
- Modify: `main.py`
- Modify: `test_main_walk_arg.py`

- [ ] **Step 1: Write the failing tests**

Append to `test_main_walk_arg.py`:

```python
import os
import subprocess
from unittest.mock import patch, MagicMock


def test_viz_hz_default_is_30():
    """--viz-hz defaults to 30 fps (was 10)."""
    import main
    args = main._make_parser().parse_args([])
    assert args.viz_hz == 30


def test_open_session_artifacts_calls_xdg_open_for_both_files(tmp_path):
    """_open_session_artifacts() calls xdg-open for walk.mp4 and summary.txt."""
    import main
    mp4_path     = tmp_path / 'walk.mp4'
    summary_path = tmp_path / 'summary.txt'
    mp4_path.touch()
    summary_path.touch()

    with patch('main.subprocess') as mock_sub:
        main._open_session_artifacts(str(tmp_path))

    assert mock_sub.Popen.call_count == 2
    opened = [c[0][0] for c in mock_sub.Popen.call_args_list]
    assert ['xdg-open', str(mp4_path)]     in opened
    assert ['xdg-open', str(summary_path)] in opened


def test_open_session_artifacts_skips_missing_mp4(tmp_path):
    """_open_session_artifacts() skips walk.mp4 if it doesn't exist yet."""
    import main
    summary_path = tmp_path / 'summary.txt'
    summary_path.touch()
    # walk.mp4 NOT created

    with patch('main.subprocess') as mock_sub:
        main._open_session_artifacts(str(tmp_path))

    assert mock_sub.Popen.call_count == 1
    opened = mock_sub.Popen.call_args_list[0][0][0]
    assert opened == ['xdg-open', str(summary_path)]


def test_open_session_artifacts_noop_when_empty(tmp_path):
    """_open_session_artifacts() does nothing when no files exist."""
    import main
    with patch('main.subprocess') as mock_sub:
        main._open_session_artifacts(str(tmp_path))
    mock_sub.Popen.assert_not_called()
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_main_walk_arg.py -k "viz_hz_default or open_session_artifacts" -v
```

Expected: `test_viz_hz_default_is_30` FAIL (returns 10); `open_session_artifacts` tests FAIL (name not found).

- [ ] **Step 3: Update `main.py`**

Add `import subprocess` after the existing imports:

Find:
```python
import argparse
import time
import pybullet as p
from HeartBeat import Siclo1Controller
```

Replace with:
```python
import argparse
import os
import subprocess
import time
import pybullet as p
from HeartBeat import Siclo1Controller
```

Change `--viz-hz` default from 10 to 30:

Find:
```python
    parser.add_argument("--viz-hz", type=int, default=10, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 10, range: 1-100)")
```

Replace with:
```python
    parser.add_argument("--viz-hz", type=int, default=30, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 30, range: 1-100)")
```

Add `_open_session_artifacts()` function before `main()`:

Find:
```python
def main(argv=None) -> None:
```

Insert before it:

```python
def _open_session_artifacts(session_path: str) -> None:
    """Non-blocking open of walk.mp4 and summary.txt after a recorded walk mission.

    Uses xdg-open (Linux). Silently skips files that do not exist.
    Fallback: run open_session.sh written to the session folder by SessionLogger.
    """
    for fname in ("walk.mp4", "summary.txt"):
        path = os.path.join(session_path, fname)
        if os.path.isfile(path):
            subprocess.Popen(["xdg-open", path])


```

Call `_open_session_artifacts` in the `finally` block of `main()`. Find:

```python
    finally:
        controller.finalize_telemetry()

        if args.hold:
```

Replace with:

```python
    finally:
        controller.finalize_telemetry()

        if args.gui and args.walk is not None:
            _open_session_artifacts(controller._telemetry_thread.session_path)

        if args.hold:
```

- [ ] **Step 4: Run full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest -v 2>&1 | tail -30
```

Expected: all tests PASS. Note the total count — it should be higher than before this feature.

- [ ] **Step 5: Commit**

```bash
git add main.py test_main_walk_arg.py
git commit -m "feat: change --viz-hz default to 30; add _open_session_artifacts() post-mission hook"
```

---

## Self-Review Checklist

**Spec coverage:**
- [x] MP4 lifecycle (RAMP→start, IDLE/emergency→stop) — Task 2
- [x] Telemetry sync (`extra_0` = frame index) — Task 4, step 3
- [x] Headless stability (no sleep in hot loop; GUI client gets `setRealTimeSimulation(0)`) — Task 1, step 4
- [x] Post-mission hook (`xdg-open` + `open_session.sh`) — Tasks 6 + 7
- [x] `GUISyncThread` fully decoupled from 100 Hz loop — Task 4
- [x] `finalize_telemetry()` sequencing (GUI stop → metadata inject → telemetry stop) — Task 5
- [x] `walk.mp4` in session folder — Task 1 (`_video_path = os.path.join(session_path, "walk.mp4")`)
- [x] `[VIDEO]` section in summary.txt — Task 6
- [x] `--viz-hz` default 30 — Task 7

**Type consistency across tasks:**
- `PoseSnapshot` defined Task 1, used Task 4 → match ✓
- `GUISyncThread.stop()` returns `{'video_path', 'video_frames', 'video_fps'}` — Task 1 definition, Task 5 consumer → match ✓
- `TelemetryThread.set_video_meta(meta: dict)` — Task 5 definition, Task 5 caller → match ✓
- `_handle_mp4_lifecycle(snapshot: PoseSnapshot)` — Task 2 definition, Task 3 caller (`run()`) → match ✓

---

**KEY POINT:** V-Sync is eliminated by never calling `gui_client` from the 100 Hz loop — `GUISyncThread` owns all GUI calls after init.

**KEY LINE:** `row[13] = self._gui_sync_thread.video_frame_count if self._gui_sync_thread is not None else 0  # MP4 frame index`
