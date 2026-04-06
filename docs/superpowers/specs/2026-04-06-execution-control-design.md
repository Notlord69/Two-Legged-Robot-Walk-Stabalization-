# Execution Control Upgrade — Design Spec
**Date:** 2026-04-06
**Status:** Approved

---

## Goal

Replace the hardcoded 30-second run limit in `main.py` with a `--duration` cycle count flag, and add a `--hold` flag that keeps the PyBullet GUI window alive after the performance summary prints, allowing manual inspection of the robot's final pose.

Target invocation:
```bash
python main.py --gui --duration 2000 --hold
```

---

## Problem Statement

1. `main.py` passes `duration=30.0` (seconds) to `controller.run()` — this is hardcoded and cannot be changed at the CLI without editing source.
2. The simulation window vanishes instantly after the summary prints, making it impossible to inspect the robot's final joint configuration.
3. Telemetry CSV is flushed inside `shutdown()` which also calls `p.disconnect()`. A hold loop must run *after* CSV flush but *before* `p.disconnect()` — currently there is no way to split these.

---

## Approach: B — `finalize_telemetry()` + `max_cycles: int`

### Why B over A and C

- **A (all in main.py):** main.py would access the private `_telemetry_thread` attribute directly. Fragile.
- **B (new public method + API change):** Clean public interface. `max_cycles: int` is semantically exact — the user thinks in cycles, not seconds.
- **C (hold inside controller.run()):** Mixes simulation logic with CLI/UI concern. Harder to unit-test.

---

## Changes

### 1. `main.py`

**New flags in `_make_parser()`:**

| Flag | Type | Default | Notes |
|---|---|---|---|
| `--duration` | `int` | `1000` | Active HeartBeat cycles to execute |
| `--hold` | `store_true` | off | Keep GUI window alive after summary |

**Updated `main()` call sequence:**
```
parse_args()
→ Siclo1Controller(use_gui, viz_decimation)
→ controller.run(max_cycles=args.duration)
→ controller.finalize_telemetry()        # CSV flushed/closed here
→ if args.hold and gui_client:           # hold loop
      while p.isConnected(gui_client):
          p.stepSimulation(physics_client)
          time.sleep(0.01)
  elif args.hold and not gui_client:
      print("[Siclo1] --hold ignored: no GUI client active")
→ controller.shutdown()                  # p.disconnect() here
```

`KeyboardInterrupt` during the hold loop prints `"[Siclo1] Hold ended."` and falls through to `shutdown()`.

`import pybullet as p` and `import time` are already present in `main.py`'s transitive imports but must be made explicit at the top of `main.py` for the hold loop.

---

### 2. `HeartBeat.py` — `Siclo1Controller`

#### `run()` signature change

```python
# Before
def run(self, duration: float = 10.0, print_interval: float = 1.0):
    total_cycles = int(duration / TARGET_DT)

# After
def run(self, max_cycles: int = 1000, print_interval: float = 1.0):
    total_cycles = max_cycles
```

- `print_interval` parameter retained for forward compatibility (currently unused).
- Banner updated from `"{duration}s @ {TARGET_FREQ} Hz"` to `"{max_cycles} cycles @ {TARGET_FREQ} Hz"`.

#### `finalize_telemetry()` — new public method

```python
def finalize_telemetry(self) -> None:
    """Stop the telemetry thread and flush/close the CSV.

    Call this BEFORE any hold loop and BEFORE shutdown().
    Safe to call more than once (is_alive() guard in shutdown()).
    """
    self._telemetry_thread.stop()
    self._telemetry_thread.join(timeout=2.0)
```

#### `shutdown()` guard

```python
def shutdown(self) -> None:
    if self._telemetry_thread.is_alive():   # guard: already finalized?
        self._telemetry_thread.stop()
        self._telemetry_thread.join(timeout=2.0)
    p.disconnect(physicsClientId=self.physics_client)
    if self.gui_client is not None:
        try:
            p.disconnect(physicsClientId=self.gui_client)
        except Exception:
            pass
    self._telemetry_thread.flush_to_stdout()
```

---

### 3. `Progress/Custom_Command.md` — two new flags + Run Profiles section

#### `--duration N`

| Property | Value |
|---|---|
| Type | integer |
| Default | `1000` |
| Unit | HeartBeat cycles (100 Hz loop iterations) |

Effect: `run()` executes exactly `N` active cycles (warmup cycles are separate and fixed).

#### `--hold`

| Property | Value |
|---|---|
| Type | boolean (store_true) |
| Default | off |
| Requires | `--gui` (prints warning and skips if no GUI) |

Effect: After the Performance Summary prints and telemetry CSV is flushed, enters a low-priority `p.stepSimulation()` loop at ~100 Hz. The GUI window stays open and fully interactive (camera rotate/zoom). Exit with `Ctrl-C`.

#### Run Profiles (new section)

```bash
# Smoke Test — 500 cycles, headless, maximum throughput
python main.py --duration 500

# Marathon Run — 5000 cycles, GUI at 10 Hz, no hold
python main.py --gui --duration 5000

# Hold Inspect — 2000 cycles, GUI, inspect final pose
python main.py --gui --duration 2000 --hold

# Slow-motion inspect — 500 cycles, 1 Hz render, hold
python main.py --gui --viz-hz 1 --duration 500 --hold
```

---

## Execution Order (full happy path with `--gui --duration 2000 --hold`)

```
1. parse args  →  use_gui=True, max_cycles=2000, hold=True
2. Siclo1Controller.__init__()
   - p.DIRECT physics client
   - p.GUI visual client
   - warmup (5 cycles)
3. controller.run(max_cycles=2000)
   - 2000 × step()
   - _print_final_summary() → terminal output
4. controller.finalize_telemetry()
   - TelemetryThread.stop() → sets _stop_event
   - TelemetryThread.run() finally → _drain() + write_summary() + CSV closed
   - join(timeout=2.0)
5. hold loop
   - while p.isConnected(gui_client): stepSimulation + sleep(0.01)
   - Ctrl-C → "[Siclo1] Hold ended."
6. controller.shutdown()
   - is_alive() → False → skip stop/join
   - p.disconnect(physics_client)
   - p.disconnect(gui_client)
   - flush_to_stdout()
```

---

## Edge Cases

| Scenario | Behavior |
|---|---|
| `--hold` without `--gui` | Warning printed, hold loop skipped |
| `KeyboardInterrupt` during `run()` | Caught in main.py, falls through to `finalize_telemetry()` + `shutdown()` |
| `KeyboardInterrupt` during hold loop | Inner `except KeyboardInterrupt` prints message, falls through to `shutdown()` |
| `finalize_telemetry()` not called before `shutdown()` | `is_alive()` guard in `shutdown()` handles it — telemetry still flushed |
| `--duration 0` | `run()` skips loop, prints summary with 0 cycles |

---

## Files Modified

| File | Change |
|---|---|
| `main.py` | Add `--duration`, `--hold`; import `pybullet`, `time`; restructure `main()` |
| `HeartBeat.py` | `run(max_cycles: int)`, `finalize_telemetry()`, `shutdown()` guard |
| `Progress/Custom_Command.md` | Two new flag entries + Run Profiles section |

No changes to `telemetry.py`, `shared_state.py`, or any module below `main.py`.
