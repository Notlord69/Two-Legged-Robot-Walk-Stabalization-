# Dual-Mode Runner Design

## Goal

Add a `--gui` / `--viz-hz` CLI entry point (`main.py`) that decouples the 100 Hz physics heartbeat from an optional PyBullet GUI viewer, and adds live IK debug visualisation (hip→foot vectors + sagittal-plane annulus arcs).

## Architecture

```
main.py  ──(args)──►  Siclo1Controller
                            │
                 ┌──────────┴───────────┐
           physics (p.DIRECT)      GUI viewer (p.GUI, only if --gui)
           100 Hz always            render every viz_decimation cycles
                 │                       │
           shared_state  ◄──────────  _sync_gui()
                                         │
                                   DebugVisualizer.update()
                                   (reads link_positions, foot targets)
                                         │
                                   sim/interface.add_debug_line()  ── p.*
```

Physics is **always** `p.DIRECT`. The GUI viewer is a second PyBullet client that mirrors joint state at a decimated rate. The two clients never share a physics step.

## File Map

| File | Action | Responsibility |
|---|---|---|
| `main.py` | Create | argparse entry point — `--gui`, `--viz-hz`, launches `Siclo1Controller` |
| `viz/__init__.py` | Create | Package marker (empty) |
| `viz/debug_markers.py` | Create | `DebugVisualizer` — annulus arcs + life-limited hip→foot vectors |
| `Progress/Custom_Command.md` | Create | CLI reference documentation |
| `HeartBeat.py` | Modify | Fix 2 bugs; remove `main()`; add `viz_decimation` param; add `_warmup()`; wire `DebugVisualizer` into `_sync_gui()` |
| `sim/interface.py` | Modify | Add `add_debug_line()` and `remove_debug_line()` wrappers |
| `shared_state.py` | Modify | Add `left_foot_target` and `right_foot_target` fields |

## HeartBeat.py — Bug Fixes

**Bug 1 (line 386):** Physics client connects to `p.GUI` instead of `p.DIRECT`. Causes a GUI window to open in headless mode and blocks on render.

```python
# BEFORE
self.physics_client = p.connect(p.GUI)
# AFTER
self.physics_client = p.connect(p.DIRECT)
```

**Bug 2 (line 516):** `self.cycle_count` does not exist; counter lives on `shared_state`.

```python
# BEFORE
if self.cycle_count % 10 == 0:
# AFTER
if shared_state.cycle_count % VIZ_DECIMATION == 0:
```

**Bug 3 (lines 515-521):** Strobe method calls `p.configureDebugVisualizer` on the physics client. Once physics is `p.DIRECT` there is no window — these calls are dead code and must be removed. GUI rendering is handled entirely by `_sync_gui()` on the GUI client.

## HeartBeat.py — Warmup

`Siclo1Controller.__init__` gains `viz_decimation: int = 10` (replaces the module-level `VIZ_DECIMATION` constant). A new `_warmup(cycles: int)` method runs the full control pipeline (read_sensors → perception → stability → active_balance → recovery → apply_control → stepSimulation) without the `HeartbeatController` timing constraint. Purpose: let the robot settle under gravity before the real-time loop starts.

**Init sequence:**

```
Direct mode (no --gui):
  1. connect physics → p.DIRECT
  2. load URDF, setup world
  3. _warmup(50 cycles)
  4. run()

GUI mode (--gui):
  1. connect physics → p.DIRECT
  2. connect GUI viewer → p.GUI
  3. time.sleep(2.0)          ← X-server buffer for WSL
  4. load URDF in both clients
  5. _warmup(5 cycles)
  6. create DebugVisualizer
  7. run()
```

The left and right hip link names (child links of `"Left_Hip_Forwards"` and `"Right_Hip_Fowards"`) are resolved once inside `_build_joint_map()` and stored as `self._left_hip_link_name` / `self._right_hip_link_name`. `_sync_gui()` reads these from `shared_state.link_positions` and passes the world positions to `DebugVisualizer.update()`.

## DebugVisualizer

**File:** `viz/debug_markers.py`

Called only from `_sync_gui()` — never from the physics loop.

### Annulus arcs

Four sagittal-plane (X-Z) semicircles: R_min and R_max for each hip. Each arc = 18 segments (10° spacing, lower half only: θ ∈ [−π/2, +π/2] around the downward vertical).

Point geometry:
```
x = hip_x + r * sin(θ)
y = hip_y              ← constant: sagittal plane
z = hip_z − r * cos(θ)
```

Color: `[0.6, 0.85, 1.0]` (light blue), line width 1. All 72 arc segment IDs are redrawn via `replaceItemUniqueId` every render tick so the arcs follow the robot.

### Life-limited vectors

| Line | From | To | Color |
|---|---|---|---|
| Left actual | left hip world pos | `state.left_foot_position` | Green `[0,1,0]` |
| Right actual | right hip world pos | `state.right_foot_position` | Green `[0,1,0]` |
| Left target | left hip world pos | `state.left_foot_target` | Red `[1,0,0]` |
| Right target | right hip world pos | `state.right_foot_target` | Red `[1,0,0]` |

Red lines are skipped (ID stays `-1`) when target is `(0.0, 0.0, 0.0)` — no gait active.

### Class interface

```python
class DebugVisualizer:
    def __init__(self, physics_client: int): ...
    def update(self, state, left_hip: tuple, right_hip: tuple) -> None: ...
```

Imports `R_MIN`, `R_MAX` from `kinematics` and `add_debug_line` from `sim.interface`.

## sim/interface.py — New Wrappers

```python
def add_debug_line(from_xyz, to_xyz, color_rgb,
                   width: float = 1.0,
                   replace_id: int = -1,
                   physics_client: int = 0) -> int:
    """Add or replace a PyBullet debug line. Returns item ID.
    Pass replace_id >= 0 to update an existing line in-place (life-limited pattern).
    """

def remove_debug_line(item_id: int, physics_client: int = 0) -> None:
    """Remove a debug line by item ID. No-op if item_id < 0."""
```

Both use deferred `import pybullet as p` (consistent with existing wrappers in the file).

## shared_state.py — New Fields

Added to `Siclo1State.__init__` in the KINEMATICS STATE block:

```python
# Per-leg foot targets (m, world frame). Written by gait planner; read by DebugVisualizer.
# Default (0,0,0) = no active target; visualiser skips red line.
self.left_foot_target:  tuple[float, float, float] = (0.0, 0.0, 0.0)
self.right_foot_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
```

## main.py

```python
import argparse
from HeartBeat import Siclo1Controller

def main():
    parser = argparse.ArgumentParser(
        description="Siclo1 bipedal robot simulation — 100 Hz physics heartbeat"
    )
    parser.add_argument("--gui", action="store_true",
                        help="Enable PyBullet GUI viewer and debug visualisation")
    parser.add_argument("--viz-hz", type=int, default=10, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 10, range: 1-100)")
    args = parser.parse_args()

    viz_hz = max(1, min(100, args.viz_hz))
    viz_decimation = max(1, 100 // viz_hz)

    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=viz_decimation)
    try:
        controller.run(duration=30.0)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted.")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
```

## Progress/Custom_Command.md — Content

```markdown
# Siclo1 Custom Commands

## Entry Point
python3 main.py [FLAGS]

## Flags

### --gui
- Type: boolean (store_true)
- Default: off (headless p.DIRECT)
- Effect: launches PyBullet GUI viewer, enables debug visualisation
  (annulus arcs, hip→foot vectors), reduces warmup to 5 cycles,
  adds 2 s X-server buffer on startup

### --viz-hz HZ
- Type: integer
- Default: 10
- Range: 1–100 (clamped silently outside range)
- Requires: --gui (ignored in headless mode)
- Effect: controls GUI render rate. Decimation = 100 ÷ viz_hz.

## Examples
python3 main.py                    # headless, max throughput, 50-cycle warmup
python3 main.py --gui              # GUI at 10 Hz render, 5-cycle warmup
python3 main.py --gui --viz-hz 33  # GUI at 33 Hz render (~every 3rd cycle)
python3 main.py --gui --viz-hz 1   # GUI at 1 Hz (slow-motion debug)
```

## Testing Strategy

- `test_dual_mode_runner.py` — unit tests for `main.py` arg parsing (valid flags, clamping, defaults)
- `test_debug_markers.py` — unit tests for `DebugVisualizer` geometry:
  - Arc point coordinates at θ = 0, ±π/2 match R_min/R_max
  - Red line skipped when target is zero vector
  - `add_debug_line` called with `replaceItemUniqueId` on second `update()` call (mock PyBullet)
- No test for the GUI render path itself (PyBullet GUI not available in CI)

## Constraints

- `viz/debug_markers.py` must not import `pybullet` directly — all `p.*` calls go through `sim/interface.py` (CLAUDE.md)
- `DebugVisualizer.update()` is called only from `_sync_gui()`, never from the physics loop
- `_warmup()` is unclocked — runs as fast as PyBullet allows; the 10 ms timing guard only applies inside `run()`
