# Offline Video Review — Design Spec

**Date:** 2026-04-08
**Project:** Siclo1 (8 kg biped, 100 Hz deterministic loop)
**Status:** Approved

---

## Problem

Real-time GUI rendering (V-Sync) causes 21 ms spikes inside the 100 Hz loop when `_sync_gui()` is called on the hot path. Running headless (DIRECT mode) eliminates the spikes (2.0 ms mean compute) but produces no visual record of the walk.

**Goal:** Preserve the 2.0 ms compute mean while producing a frame-accurate MP4 of every `--walk` mission, stored alongside `telemetry.csv` in the session folder.

---

## Scope

- Affects: `HeartBeat.py`, `telemetry.py`, `main.py`
- Does not touch: `mission.py`, `shared_state.py`, `sim/interface.py`, URDF, all test files
- CLI trigger: `python3 main.py --gui --walk 2.0` (MP4 recorded automatically when both flags are active)

---

## Architecture

```
100 Hz physics loop (DIRECT client)
  step()
    ├── physics, perception, stability, grf, gait, mission, wbc
    ├── sim.interface.step_simulation(physics_client)   ← no render stall
    ├── push_pose(PoseSnapshot)                         ← lock-free, skips if busy
    └── row[13] = gui_sync_thread.video_frame_count     ← GIL-safe int read

GUISyncThread (daemon, ~30 Hz)
  loop every 1/viz_fps seconds:
    ├── read PoseSnapshot slot
    ├── resetBasePositionAndOrientation(gui_client)
    ├── resetJointState × N (gui_client)
    ├── p.stepSimulation(gui_client)   ← captures MP4 frame
    ├── increment _video_frame_count
    └── poll mission_state → start/stop MP4 logging
```

The physics loop **never calls `gui_client` after init**. V-Sync is fully decoupled.

---

## Components

### 1. `PoseSnapshot` (dataclass, `HeartBeat.py`)

```python
@dataclass
class PoseSnapshot:
    base_pos: tuple          # (x, y, z) world position
    base_orn: tuple          # quaternion (x, y, z, w)
    joint_states: dict       # {joint_name: (position, velocity)}
    mission_state: MissionState
    emergency_stop: bool
```

Written by `step()` via `push_pose()`. One slot — always the latest state, never queued.

### 2. `GUISyncThread` (new class, `HeartBeat.py`)

**Init parameters:**
- `gui_client: int` — PyBullet GUI physics client ID
- `gui_robot_id: int` — robot body ID on the GUI client
- `joint_list: list` — pre-frozen `[(joint_name, joint_id), ...]`
- `session_path: str` — from `TelemetryThread.session_path`
- `viz_fps: int` — from `--viz-hz` (default 30)
- `walk_active: bool` — True when `--walk` was passed; gates MP4 recording

**Slot write (`push_pose`):**
```python
def push_pose(self, snapshot: PoseSnapshot) -> None:
    if self._lock.acquire(blocking=False):
        self._slot = snapshot
        self._lock.release()
    # if busy: skip — thread will read latest on next iteration
```

**Thread loop (`run`):**
1. Sleep `1 / viz_fps` seconds
2. Acquire lock (blocking), copy slot, release
3. Mirror base pose + joint states to `gui_client`
4. `p.stepSimulation(physicsClientId=gui_client)` — triggers MP4 frame capture
5. Increment `_video_frame_count`
6. Check mission state edge: `_prev_mission_state → snapshot.mission_state`
   - `→ RAMP`: call `p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_path, physicsClientId=gui_client)`; store `_log_id`
   - `→ IDLE` (post-STOP) or `snapshot.emergency_stop`: call `p.stopStateLogging(_log_id, physicsClientId=gui_client)`; set `_log_id = None`

**`stop()` safety net:**
```python
def stop(self) -> dict:
    self._stop_event.set()
    if self._log_id is not None:
        p.stopStateLogging(self._log_id, physicsClientId=self._gui_client)
        self._log_id = None
    return {
        'video_path':   self._video_path,
        'video_frames': self._video_frame_count,
        'video_fps':    self._viz_fps,
    }
```

Returns video metadata dict consumed by `finalize_telemetry()`.

**Init guard:**
```python
p.setRealTimeSimulation(0, physicsClientId=gui_client)
```
Prevents the GUI client from self-advancing time.

### 3. `Siclo1Controller` changes (`HeartBeat.py`)

- Remove `_sync_gui()` method
- Remove `% self.viz_decimation` guard in `step()`
- Add `self._gui_sync_thread: Optional[GUISyncThread]` (None when no GUI)
- In `step()`, after `sim.interface.step_simulation`:
  ```python
  if self._gui_sync_thread is not None:
      self._gui_sync_thread.push_pose(PoseSnapshot(...))
  ```
- In `step()`, before `shared_state.telemetry.write(row)`:
  ```python
  row[13] = self._gui_sync_thread.video_frame_count if self._gui_sync_thread else 0
  ```
- `finalize_telemetry()` sequencing update (order matters):
  1. `video_meta = self._gui_sync_thread.stop()` — closes MP4 cleanly, returns metadata dict
  2. `self._telemetry_thread.set_video_meta(video_meta)` — injects keys before summary is written
  3. `self._telemetry_thread.stop(); join()` — final drain + `write_summary()` fires with video keys present
- In `shutdown()`: only `p.disconnect(gui_client)` remains (GUI sync thread already stopped in step 1)

### 4. Telemetry sync (`telemetry.py`)

`extra_0` (CSV column `row[13]`) repurposed as **MP4 frame index**:

| cycle | extra_0 | meaning |
|-------|---------|---------|
| 1–50  | 0       | warmup / IDLE, not recording |
| 51    | 1       | first captured frame (RAMP entered) |
| 54    | 2       | next frame (~3 cycles later at 30 fps) |
| ...   | ...     | |

Mapping a CSV anomaly to a video frame: read `extra_0` on the spike row → seek to that frame in the MP4.

`write_summary()` gains three optional keys in the `stats` dict:

```
[VIDEO]
  File    : sessions/2026-04-08_21-38-18/walk.mp4
  Frames  : 1247
  FPS     : 30
  Duration: 41.6 s
```

Written only when `video_path` key is present in `stats`.

### 5. Video path

```python
video_path = os.path.join(session_path, "walk.mp4")
```

Stored alongside `telemetry.csv` and `summary.txt`:

```
sessions/
  2026-04-08_21-38-18/
    telemetry.csv
    summary.txt
    walk.mp4          ← new
    open_session.sh   ← new
```

### 6. Post-mission hook (`main.py`)

```python
def _open_session_artifacts(session_path: str) -> None:
    """Non-blocking open of MP4 and summary after a recorded walk mission."""
    for fname in ("walk.mp4", "summary.txt"):
        path = os.path.join(session_path, fname)
        if os.path.isfile(path):
            subprocess.Popen(["xdg-open", path])
```

Called in `main()` after `finalize_telemetry()` when `args.gui and args.walk is not None`.

`SessionLogger` also writes `open_session.sh` to the session folder when `video_path` is present:

```bash
#!/bin/bash
# Manual fallback for WSL2 environments without xdg-open
xdg-open "$(dirname "$0")/walk.mp4" &
xdg-open "$(dirname "$0")/summary.txt" &
```

---

## CLI changes

`main.py` — `--viz-hz` default changes from `10` → `30`.

No new flags. Recording is automatic when `--gui` and `--walk` are both active.

---

## Timing guarantee

| Path | Change | Impact |
|------|--------|--------|
| 100 Hz loop | Removes `_sync_gui()` call | Eliminates 21 ms V-Sync spikes |
| `push_pose()` | Non-blocking lock + dataclass copy | < 1 µs when uncontested |
| `row[13]` write | GIL-safe int read | 0 µs overhead |
| `GUISyncThread` | Sleeps `1/30 s` between renders | Zero impact on physics loop |

Target: maintain 2.0 ms mean compute in DIRECT+GUI mode.

---

## Constraints satisfied

- No UI-blocking calls in the 100 Hz loop
- MP4 stored in `session_path` (owned by `SessionLogger`)
- `startStateLogging` / `stopStateLogging` called only from `GUISyncThread`
- Recording starts at `RAMP`, closes cleanly at `IDLE` or `EmergencyStop`
- `extra_0` provides frame-accurate CSV ↔ MP4 mapping
- `open_session.sh` fallback for WSL2 environments
