# PyBullet Render Isolation Design

**Date:** 2026-04-17  
**Project:** Siclo1 — bipedal robot simulation  
**Status:** Approved

---

## Problem

Timing violations occur when `--gui` is enabled on integrated graphics. The existing
`GUISyncThread` decouples rendering from the 100 Hz physics loop at the thread level,
but both threads share the Python GIL. When `p.stepSimulation(gui_client)` stalls on the
iGPU (WSL2 + Mesa/DXGI), the GUI thread holds the GIL for an unpredictable duration,
directly stealing time from the physics thread. Thread isolation is not sufficient;
process isolation is required.

---

## Goals

1. **Per-stage timing instrumentation** — identify exactly which stage inside `step()`
   consumes budget, retained permanently as a diagnostic tool.
2. **Subprocess GUI** — move all `p.GUI` work into a separate OS process so the GIL
   is never shared, making render stalls physically impossible to cause physics violations.
3. **Interactive camera** — the GUI window must support free pan/orbit without any
   coordination with the physics process.
4. **Non-critical GUI** — if the subprocess fails to start or dies mid-run, the physics
   loop continues headlessly; no exception reaches `step()`.

---

## Architecture Overview

### Phase 1 — Per-stage instrumentation

A pre-allocated `float64[12]` array (`_stage_times`) is added to `Siclo1State`.
At each named stage boundary inside `step()`, the elapsed time from `cycle_start`
is recorded in-place. At end of cycle the array is diff'd; if the worst stage exceeds
a threshold (e.g. 3 ms), its name and duration are logged to `TelemetryThread`'s text
buffer (not the fixed-column CSV — `analyze.py` schema is unchanged). Zero allocation;
no behavioral change to the physics loop.

### Phase 2 — Subprocess GUI (`VizBridge`)

Two new files replace `GUISyncThread`, `PoseSnapshot`, `_push_gui_snapshot()`, and
`_sync_gui()` in `HeartBeat.py`:

```
sim/viz_bridge.py    — VizBridge class: owns shared_memory block, launches subprocess
viz/gui_worker.py    — subprocess entry point: owns p.GUI, reads buffer, mirrors pose
```

`HeartBeat.py` replaces the GUI sync thread with a single field `self._viz_bridge`
and one call per `viz_decimation` cycles:

```python
self._viz_bridge.push_pose(pos, orn, joint_positions)
```

---

## Data Layout

### Shared memory buffer

One flat `float64` array. No locks — latest-value semantics (physics always overwrites,
GUI always reads most recent). Total ≈ 240 bytes for N=23 joints.

```
Index   Field          Size   Description
──────────────────────────────────────────────────────
0       seq_num        1      monotonic counter; GUI skips frame if unchanged
1–3     base_pos       3      (x, y, z) world position, metres
4–7     base_orn       4      quaternion (x, y, z, w)
8–8+N   joint_angles   N      float64 per joint in URDF_JOINT_NAMES order
──────────────────────────────────────────────────────
Total: (1 + 3 + 4 + N) × 8 bytes
```

`seq_num` is incremented **last** — after all pose fields are written — so the GUI
never reads a partially-updated frame.

### Per-stage instrumentation

```python
STAGE_NAMES: tuple[str, ...] = (
    'sensors', 'link_positions', 'perception', 'stability',
    'active_balance', 'grf', 'gait_planner', 'mission',
    'wbc', 'recovery', 'apply_control', 'step_sim',
)
# Added to Siclo1State.__init__:
self._stage_times: np.ndarray = np.zeros(len(STAGE_NAMES), dtype=np.float64)
```

At each stage boundary in `step()`:
```python
self._stage_times[i] = time.perf_counter() - self.heartbeat.cycle_start
```

---

## Interfaces

### `VizBridge` (`sim/viz_bridge.py`)

```python
class VizBridge:
    def __init__(self, joint_names: list[str], urdf_path: str,
                 viz_fps: int = 30, spawn_z: float = 0.8806): ...

    def start(self) -> None:
        # Spawns subprocess, waits up to 5 s for ready flag.
        # On timeout: logs warning, sets _active=False. Physics continues headlessly.

    def push_pose(self,
                  base_pos,           # (x, y, z)
                  base_orn,           # (x, y, z, w)
                  joint_positions: dict[str, float]) -> None:
        # Writes into pre-allocated numpy view. No allocation. No locking.
        # If subprocess died: sets _active=False silently, returns immediately.

    def stop(self) -> None:
        # terminate() → join(2s) → kill() if still alive
        # shm.close() + shm.unlink() always called (try/except double-unlink)

    @property
    def is_alive(self) -> bool: ...
```

### `gui_worker.main()` (`viz/gui_worker.py`)

```python
def main(shm_name: str, n_joints: int, joint_names: list[str],
         urdf_path: str, viz_fps: int, spawn_z: float,
         ready_flag: multiprocessing.Value) -> None:
    # 1. Connect p.GUI, suppress side panel, load plane + URDF
    # 2. Build joint index map from p.getJointInfo
    # 3. Set ready_flag.value = 1
    # 4. Render loop: read seq_num, skip if unchanged, else mirror pose + stepSimulation
    # Camera: never touched by worker — PyBullet handles mouse input natively
```

---

## Process Lifecycle

### Startup

```
VizBridge.start()
  → allocate shared_memory block, zero numpy view
  → multiprocessing.Process(target=gui_worker.main, daemon=True).start()
  → spin-wait on ready_flag (timeout=5s)
      timeout → log warning, _active=False (headless fallback)
      success → _active=True

gui_worker.main()
  → p.connect(p.GUI)
  → load plane.urdf + Siclo1.urdf
  → build joint_ids map
  → ready_flag.value = 1
  → enter render loop
```

### Steady state

```
100 Hz physics loop (main process):
  every viz_decimation cycles:
    viz_bridge.push_pose(pos, orn, joints)  ← ~15 lines, writes numpy view in-place

GUI subprocess (separate process, separate GIL):
  at viz_fps Hz:
    read seq_num
    if changed: resetBasePositionAndOrientation + resetJointState × N + stepSimulation
    else: sleep remainder
    (user pan/orbit handled natively by PyBullet window event loop)
```

### Shutdown (inside `Siclo1Controller.shutdown()`)

```
viz_bridge.stop()
  → process.terminate()
  → process.join(timeout=2.0)
  → if alive: process.kill()
  → shm.close()
  → shm.unlink()   ← always, try/except
```

---

## Files Changed

| File | Change |
|---|---|
| `shared_state.py` | Add `STAGE_NAMES`, `_stage_times` to `Siclo1State` |
| `HeartBeat.py` | Add stage timestamps to `step()`; remove `GUISyncThread`, `PoseSnapshot`, `_push_gui_snapshot`, `_sync_gui`; add `_viz_bridge` field |
| `sim/viz_bridge.py` | **New** — `VizBridge` class |
| `viz/gui_worker.py` | **New** — subprocess entry point |
| `Test Enviroment/test_viz_bridge.py` | **New** |
| `Test Enviroment/test_gui_worker.py` | **New** |
| `Test Enviroment/test_stage_instrumentation.py` | **New** |
| `Test Enviroment/test_gui_sync_thread.py` | **Delete** — `GUISyncThread` removed |
| `Test Enviroment/test_gui_sync_fix.py` | **Delete** — covered by new tests |

---

## Tests

### `test_viz_bridge.py`

| Test | Verifies |
|---|---|
| `test_shared_memory_layout` | Buffer size = `(1 + 3 + 4 + N) × 8` bytes |
| `test_push_pose_writes_seq` | `seq_num` increments on each call |
| `test_push_pose_writes_position` | `base_pos`, `base_orn` at correct indices |
| `test_push_pose_writes_joints` | Joint values in `URDF_JOINT_NAMES` order |
| `test_push_pose_nonblocking` | Returns in < 1 ms |
| `test_start_timeout_sets_inactive` | Subprocess never ready → `_active=False` |
| `test_stop_unlinks_shm` | `shm.unlink()` called even if process dead |
| `test_is_alive_false_after_stop` | `is_alive` is False after `stop()` |

### `test_gui_worker.py`

| Test | Verifies |
|---|---|
| `test_worker_applies_pose` | Correct args to `p.resetBasePositionAndOrientation` |
| `test_worker_skips_frame_same_seq` | No PyBullet calls when `seq_num` unchanged |
| `test_worker_sets_ready_flag` | `ready_flag.value == 1` after init |

### `test_stage_instrumentation.py`

| Test | Verifies |
|---|---|
| `test_stage_times_shape` | Shape `(12,)` float64 |
| `test_stage_times_monotonic` | Each entry ≥ previous in a mocked `step()` |
| `test_stage_names_count` | Exactly 12 entries matching stage order |

---

## Constraints Preserved

- All PyBullet calls in `gui_worker.py` are **not** in `sim/interface.py` — this is
  intentional. The worker is a standalone subprocess that will not be swapped to Gazebo;
  it is display-only infrastructure, not control logic.
- `shared_state.py`: no existing fields renamed. `_stage_times` and `STAGE_NAMES` are
  additions only.
- `Siclo1.urdf` not touched.
- 100 Hz hard limit enforced by existing `HeartbeatController` — no changes.
