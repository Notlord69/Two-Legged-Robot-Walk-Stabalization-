# GUI Visual Pipeline — Design Spec
**Date:** 2026-04-13
**Status:** Approved

---

## Problem

In GUI mode (`python3 main.py --gui`), the simulation terminates early due to
timing violations. The robot stops (window closes) even during standing, without
reaching the commanded cycle count. Headless mode is stable.

**Root causes identified:**

1. `GUISyncThread.run()` calls `p.stepSimulation(gui_client)` at 30 fps.
   PyBullet uses an internal global mutex across all clients in the same process.
   This call competes directly with the main thread's physics step, causing
   sporadic >10 ms pauses in the 100 Hz loop.

2. GUI warmup is only 5 cycles (vs 50 headless). Five physics steps are
   insufficient for contact confirmation, so the robot enters the real-time loop
   without grounded feet and falls immediately, triggering recovery/freeze logic.

3. The 10 ms hard limit is enforced identically in GUI and headless modes.
   In GUI mode (WSL2 + X11), OS and rendering overhead makes the budget
   unreachable — any violation freezes the robot permanently.

---

## Goals

- GUI mode runs to completion without timing-triggered termination.
- User can observe live 3D robot motion while the simulation runs.
- Post-run: full 3D replay with optional MP4 export.
- Post-run: time-series plots from the telemetry CSV.
- Video generation never runs during the 100 Hz loop.

---

## Architecture

```
python3 main.py --gui
       │
       ├── physics_client (p.DIRECT) ──── 100 Hz control loop
       │      │                                │
       │      │                        PoseLogBuffer (pre-alloc numpy, hot path)
       │      │
       ├── gui_client (p.GUI) ←── GUISyncThread (~10 fps, mirror-only)
       │
       └── TelemetryThread ──────────► sessions/<date>/telemetry.csv
                                       sessions/<date>/poses.npy  (written at shutdown)

python3 replay.py sessions/<date>/
       ├── Loads poses.npy
       ├── Opens p.GUI, loads URDF
       ├── Steps frames at user-controlled speed
       └── VideoRecorder (TinyRenderer) ──► sessions/<date>/walk.mp4

python3 analyze.py sessions/<date>/
       └── Reads telemetry.csv ──► 4 PNG figures
```

**Key invariant:** The main run never calls `p.getCameraImage`, never writes video,
and never steps the GUI client physics. All visually-heavy work happens post-run.

---

## Component Designs

### 1. HeartBeat.py — GUI timing fix

**1a. Soft timing in GUI mode**

`HeartbeatController` gains a `strict: bool` parameter (default `True`).

```python
self.heartbeat = HeartbeatController(target_dt=TARGET_DT, strict=not use_gui)
```

When `strict=False`:
- Violations are counted and logged as before.
- The spin-wait is skipped on overrun cycles (no wasted CPU on a cycle that's
  already late).
- Execution continues — the loop never returns `False` due to timing alone.
- The simulation runs slower than real-time; the user explicitly accepted this.

**1b. Warmup parity**

```python
warmup_cycles = 50  # same in GUI and headless modes
```

50 cycles gives the physics engine time to confirm foot contacts before the
real-time loop starts. The 5-cycle GUI value was an optimisation to show the
window sooner, but caused contact-confirmation failure.

**1c. WSL GUI settle constant**

```python
GUI_CONNECT_SETTLE_S: float = 1.0  # s — X-server buffer wait after p.connect(p.GUI)
```

Replaces the bare `time.sleep(1.0)`. Value unchanged; constant makes it findable.

---

### 2. HeartBeat.py — GUISyncThread fix

**2a. Remove `p.stepSimulation` from display thread**

`GUISyncThread.run()` currently steps the GUI physics client each frame.
This call is removed entirely. The GUI client only needs pose resets
(`resetBasePositionAndOrientation` + `resetJointState`) — not physics stepping.

**2b. Reduce sync rate**

```python
GUI_SYNC_FPS: int = 10  # Hz — GUI mirror rate; lower = less mutex pressure
```

Replaces hardcoded `viz_fps=30` in `GUISyncThread` construction. 10 fps is
sufficient to observe robot motion. Matches the `viz_decimation` default of 10
cycles, making the two consistent.

---

### 3. PoseLogger (new file: `pose_logger.py`)

Captures full robot state per cycle. Zero file I/O during the run.

**Data layout — 19 floats per row (float64):**

| Index | Field | Source |
|-------|-------|--------|
| 0 | sim_time | `shared_state.sim_time` |
| 1–3 | base_pos (x, y, z) | `p.getBasePositionAndOrientation` |
| 4–7 | base_orn (qx, qy, qz, qw) | same |
| 8–10 | left joint angles (hip, knee, ankle) | `shared_state.joint_positions` |
| 11–13 | right joint angles (hip, knee, ankle) | `shared_state.joint_positions` |
| 14–15 | foot forces (left, right) N | `shared_state` |
| 16 | stability_status | `shared_state` |
| 17 | gait_phase | `shared_state` |
| 18 | spare (zero-padded) | — |

**Storage:** 19 × 8 = 152 bytes/cycle. 1000 cycles → 148 KB. 10,000 cycles → 1.5 MB.

**API:**

```python
class PoseLogger:
    def __init__(self, max_cycles: int = 20_000) -> None: ...
    def record(self, sim_time: float, base_pos, base_orn,
               joint_positions: dict, left_force: float,
               right_force: float, stability_status: int,
               gait_phase: int) -> None: ...
    def save(self, session_path: str) -> str: ...  # returns path to poses.npy
```

`record()`: single row write into a pre-allocated `numpy` array. Zero allocation,
no file I/O. Called from `HeartBeat.step()` at step 17 (alongside telemetry row).

`base_pos` and `base_orn` for the pose logger are read once per cycle via
`p.getBasePositionAndOrientation(physics_client)` and reused for both the pose
log and `_push_gui_snapshot()`. No duplicate PyBullet calls.

`save()`: calls `np.save()` once. Called from `Siclo1Controller.shutdown()` after
the run completes.

Integration in `HeartBeat.py`:
- `self._pose_logger = PoseLogger()` in `__init__`
- `self._pose_logger.record(...)` in `step()` at step 17
- `self._pose_logger.save(self._telemetry_thread.session_path)` in `shutdown()`

---

### 4. `replay.py` (new file)

Standalone post-run 3D replay. No imports from `HeartBeat.py`.

**Usage:**
```
python3 replay.py sessions/2026-04-13_14-30-00/
python3 replay.py sessions/2026-04-13_14-30-00/ --speed 0.5
python3 replay.py sessions/2026-04-13_14-30-00/ --record
python3 replay.py sessions/2026-04-13_14-30-00/ --headless --record
```

**Playback loop:**
1. Load `poses.npy` from session folder.
2. Connect `p.GUI` (or `p.DIRECT` for `--headless`).
3. Load `plane.urdf` + `Siclo1.urdf`.
4. For each row in poses:
   - `p.resetBasePositionAndOrientation`
   - `p.resetJointState` for each of the 6 active joints
   - `p.stepSimulation` (one step to refresh visuals)
   - If `--record`: capture frame via `sim.interface.capture_frame()`
   - `time.sleep(TARGET_DT / speed_factor)` — controls playback rate
5. If `--record`: flush writer, save `walk.mp4` to same session folder.

**Dependencies:** `sim/interface.py`, `shared_state.py` (joint name constants),
`recorder.py`. No control modules (stability, gait, WBC, etc.) are imported.

**`--speed` default:** `1.0` (real-time). Values < 1.0 slow down (e.g. 0.5 = half
speed). Values > 1.0 speed up.

---

### 5. `analyze.py` (new file)

Standalone post-run plot generator. Reads only `telemetry.csv`.

**Usage:**
```
python3 analyze.py sessions/2026-04-13_14-30-00/
python3 analyze.py sessions/2026-04-13_14-30-00/ --show
```

**Output — 4 PNG files written to the session folder:**

| File | Content |
|------|---------|
| `com_trajectory.png` | COM x/y/z vs sim_time |
| `contact_forces.png` | Left + right foot force (N) vs sim_time |
| `timing.png` | Compute time (µs) per cycle, violation markers |
| `stability.png` | Stability margin + stability status overlay vs sim_time |

`--show`: calls `plt.show()` after saving for interactive viewing (requires X11).
Without `--show`: fully headless, saves PNGs only.

**Dependencies:** `numpy`, `matplotlib`. No PyBullet import.

---

## File Changes Summary

| File | Change |
|------|--------|
| `HeartBeat.py` | `HeartbeatController` strict flag; warmup parity; GUI_CONNECT_SETTLE_S constant; GUISyncThread: remove stepSimulation, GUI_SYNC_FPS=10 |
| `pose_logger.py` | New file |
| `replay.py` | New file |
| `analyze.py` | New file |

`VideoRecorder` (`recorder.py`): no changes. Called only from `replay.py --record`.
`TelemetryThread` (`telemetry.py`): no changes.
`sim/interface.py`: no changes.
`shared_state.py`: no changes.

---

## Acceptance Criteria

1. `python3 main.py --gui` runs to completion (all requested cycles) without early termination.
2. `python3 main.py --gui` shows live robot motion in the PyBullet window.
3. `python3 replay.py sessions/<date>/` plays back the session in 3D.
4. `python3 replay.py sessions/<date>/ --record` produces a valid `walk.mp4`.
5. `python3 analyze.py sessions/<date>/` produces 4 PNG files in the session folder.
6. Headless mode (`python3 main.py`) is unaffected — strict timing, no pose logger overhead beyond the pre-allocated array write.
