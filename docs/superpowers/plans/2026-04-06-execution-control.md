# Execution Control Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hardcoded 30-second run limit with `--duration` (cycle count) and add `--hold` (keep GUI window alive for pose inspection).

**Architecture:** Three-file change — `HeartBeat.py` gets `run(max_cycles: int)`, a new public `finalize_telemetry()`, and a double-stop guard in `shutdown()`; `main.py` gains two new flags and a restructured call sequence (run → finalize → hold → shutdown); `Progress/Custom_Command.md` gains two flag entries and a Run Profiles section.

**Tech Stack:** Python 3.10, PyBullet, argparse, pytest, unittest.mock

---

## File Map

| File | Change |
|---|---|
| `HeartBeat.py` | `run(max_cycles: int)`, `finalize_telemetry()`, `shutdown()` guard |
| `main.py` | `--duration`, `--hold` flags; `import pybullet as p`; `import time`; restructured `main()` |
| `test_dual_mode_runner.py` | Update broken assertion; add 6 new tests |
| `Progress/Custom_Command.md` | Two new flag sections + Run Profiles |

---

## Task 1: Change `run()` to accept cycle count

**Files:**
- Modify: `HeartBeat.py:616-630`
- Test: `test_dual_mode_runner.py:53-60`

- [ ] **Step 1: Update the existing integration test to expect the new signature**

Open `test_dual_mode_runner.py`. Replace lines 53-60 with:

```python
def test_main_passes_decimation_to_controller():
    """main() converts --viz-hz to decimation and passes it to Siclo1Controller."""
    mock_ctrl = MagicMock()
    with patch('main.Siclo1Controller', return_value=mock_ctrl) as mock_cls:
        main(["--viz-hz", "20"])
    mock_cls.assert_called_once_with(use_gui=False, viz_decimation=5)
    mock_ctrl.run.assert_called_once_with(max_cycles=1000)
    mock_ctrl.finalize_telemetry.assert_called_once()
    mock_ctrl.shutdown.assert_called_once()
```

- [ ] **Step 2: Run the test to confirm it fails**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_dual_mode_runner.py::test_main_passes_decimation_to_controller -v
```

Expected: `FAILED — AssertionError` (run called with `duration=30.0`, not `max_cycles=1000`)

- [ ] **Step 3: Update `run()` in HeartBeat.py**

Replace lines 616-630 in `HeartBeat.py`:

```python
    def run(self, max_cycles: int = 1000, print_interval: float = 1.0):
        print(f"\n{'='*70}")
        print(f"STARTING SIMULATION — {max_cycles} cycles @ {TARGET_FREQ} Hz  (OPTIMISED)")
        print(f"{'='*70}\n")

        for i in range(max_cycles):
            success = self.step()
            if not success:
                self._telemetry_thread.log(
                    f"[STOPPED] t={shared_state.sim_time:.2f}s")
                break

        self._print_final_summary()
```

- [ ] **Step 4: Run test to confirm it passes**

```bash
pytest test_dual_mode_runner.py::test_main_passes_decimation_to_controller -v
```

Expected: still `FAILED` — `finalize_telemetry` not yet defined. That is correct; Task 2 adds it.

---

## Task 2: Add `finalize_telemetry()` and `shutdown()` guard

**Files:**
- Modify: `HeartBeat.py` (after `run()`, before `_print_final_summary()`)
- Test: `test_dual_mode_runner.py`

- [ ] **Step 1: Write two failing tests**

Append to `test_dual_mode_runner.py`:

```python
# ── finalize_telemetry / shutdown guard ───────────────────────────────────── #

def test_finalize_telemetry_stops_and_joins_thread():
    """finalize_telemetry() must call stop() then join() on the telemetry thread."""
    mock_ctrl = MagicMock()
    mock_thread = MagicMock()
    mock_thread.is_alive.return_value = True
    mock_ctrl._telemetry_thread = mock_thread

    # Call the real method on our mock object by binding it
    from HeartBeat import Siclo1Controller
    Siclo1Controller.finalize_telemetry(mock_ctrl)

    mock_thread.stop.assert_called_once()
    mock_thread.join.assert_called_once_with(timeout=2.0)


def test_shutdown_skips_join_when_already_finalized():
    """shutdown() must not call stop/join if the thread is no longer alive."""
    mock_ctrl = MagicMock()
    mock_thread = MagicMock()
    mock_thread.is_alive.return_value = False   # already joined
    mock_ctrl._telemetry_thread = mock_thread
    mock_ctrl.gui_client = None

    from HeartBeat import Siclo1Controller
    Siclo1Controller.shutdown(mock_ctrl)

    mock_thread.stop.assert_not_called()
    mock_thread.join.assert_not_called()
```

- [ ] **Step 2: Run the tests to confirm they fail**

```bash
pytest test_dual_mode_runner.py::test_finalize_telemetry_stops_and_joins_thread \
       test_dual_mode_runner.py::test_shutdown_skips_join_when_already_finalized -v
```

Expected: `FAILED — AttributeError: type object 'Siclo1Controller' has no attribute 'finalize_telemetry'`

- [ ] **Step 3: Add `finalize_telemetry()` to `Siclo1Controller` in `HeartBeat.py`**

Insert after line 630 (after `self._print_final_summary()` line), before `_print_final_summary`:

```python
    # ------------------------------------------------------------------ #
    def finalize_telemetry(self) -> None:
        """Stop telemetry thread and flush/close the CSV.

        Call BEFORE any hold loop and BEFORE shutdown().
        shutdown() checks is_alive() so calling both is safe.
        """
        self._telemetry_thread.stop()
        self._telemetry_thread.join(timeout=2.0)
```

- [ ] **Step 4: Update `shutdown()` in `HeartBeat.py`**

Replace lines 677-686 with:

```python
    def shutdown(self) -> None:
        if self._telemetry_thread.is_alive():
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

- [ ] **Step 5: Run all three tests**

```bash
pytest test_dual_mode_runner.py::test_finalize_telemetry_stops_and_joins_thread \
       test_dual_mode_runner.py::test_shutdown_skips_join_when_already_finalized \
       test_dual_mode_runner.py::test_main_passes_decimation_to_controller -v
```

Expected: all three `PASSED`

- [ ] **Step 6: Commit**

```bash
git add HeartBeat.py test_dual_mode_runner.py
git commit -m "feat: run(max_cycles: int), finalize_telemetry(), shutdown() guard"
```

---

## Task 3: Add `--duration` and `--hold` to `main.py`

**Files:**
- Modify: `main.py`
- Test: `test_dual_mode_runner.py`

- [ ] **Step 1: Write failing parser tests**

Append to `test_dual_mode_runner.py`:

```python
# ── --duration flag ────────────────────────────────────────────────────────── #

def test_default_duration():
    args = _make_parser().parse_args([])
    assert args.duration == 1000


def test_duration_parsed():
    args = _make_parser().parse_args(["--duration", "2000"])
    assert args.duration == 2000


# ── --hold flag ────────────────────────────────────────────────────────────── #

def test_default_hold():
    args = _make_parser().parse_args([])
    assert args.hold is False


def test_hold_flag():
    args = _make_parser().parse_args(["--hold"])
    assert args.hold is True
```

- [ ] **Step 2: Run parser tests to confirm they fail**

```bash
pytest test_dual_mode_runner.py::test_default_duration \
       test_dual_mode_runner.py::test_duration_parsed \
       test_dual_mode_runner.py::test_default_hold \
       test_dual_mode_runner.py::test_hold_flag -v
```

Expected: `FAILED — AttributeError: Namespace object has no attribute 'duration'`

- [ ] **Step 3: Write failing hold-loop behaviour tests**

Append to `test_dual_mode_runner.py`:

```python
# ── hold loop behaviour ────────────────────────────────────────────────────── #

def test_hold_with_gui_steps_physics(capsys):
    """--hold with active GUI: stepSimulation called once before isConnected→False."""
    mock_ctrl = MagicMock()
    mock_ctrl.gui_client = 99          # non-None → GUI active
    mock_ctrl.physics_client = 0

    with patch('main.Siclo1Controller', return_value=mock_ctrl):
        with patch('main.p') as mock_p:
            with patch('main.time') as mock_time:
                mock_p.isConnected.side_effect = [True, False]
                main(["--gui", "--duration", "0", "--hold"])

    mock_p.stepSimulation.assert_called_once_with(physicsClientId=0)
    mock_ctrl.finalize_telemetry.assert_called_once()
    mock_ctrl.shutdown.assert_called_once()


def test_hold_without_gui_prints_warning(capsys):
    """--hold without --gui: warning printed, stepSimulation never called."""
    mock_ctrl = MagicMock()
    mock_ctrl.gui_client = None        # no GUI

    with patch('main.Siclo1Controller', return_value=mock_ctrl):
        with patch('main.p') as mock_p:
            main(["--duration", "0", "--hold"])

    captured = capsys.readouterr()
    assert "--hold ignored" in captured.out
    mock_p.isConnected.assert_not_called()
    mock_ctrl.finalize_telemetry.assert_called_once()
    mock_ctrl.shutdown.assert_called_once()
```

- [ ] **Step 4: Run hold tests to confirm they fail**

```bash
pytest test_dual_mode_runner.py::test_hold_with_gui_steps_physics \
       test_dual_mode_runner.py::test_hold_without_gui_prints_warning -v
```

Expected: `FAILED — AttributeError: Namespace object has no attribute 'hold'`

- [ ] **Step 5: Rewrite `main.py` with new flags and call sequence**

Replace the entire contents of `main.py` with:

```python
"""
Siclo1 bipedal robot simulation — CLI entry point.

Usage:
    python3 main.py                                # headless, 1000 cycles
    python3 main.py --gui                          # GUI at 10 Hz, 1000 cycles
    python3 main.py --gui --viz-hz 33              # GUI at 33 Hz
    python3 main.py --gui --duration 2000 --hold   # 2000 cycles, inspect final pose
"""
import argparse
import time
import pybullet as p
from HeartBeat import Siclo1Controller


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Siclo1 bipedal robot simulation — 100 Hz physics heartbeat"
    )
    parser.add_argument("--gui", action="store_true",
                        help="Enable PyBullet GUI viewer and debug visualisation")
    parser.add_argument("--viz-hz", type=int, default=10, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 10, range: 1-100)")
    parser.add_argument("--duration", type=int, default=1000, metavar="CYCLES",
                        help="Number of active HeartBeat cycles to run (default: 1000)")
    parser.add_argument("--hold", action="store_true",
                        help="Keep GUI window open after summary for pose inspection")
    return parser


def _viz_decimation(viz_hz: int) -> int:
    """Convert render rate (Hz) to loop decimation factor.

    viz_hz: desired GUI frames per second (clamped to 1-100 Hz)
    Returns: loop cycles between GUI renders (physics runs at 100 Hz)
    """
    hz = max(1, min(100, viz_hz))   # clamp: 1 Hz min, 100 Hz max
    return max(1, 100 // hz)        # cycles/render; minimum 1


def main(argv=None) -> None:
    args = _make_parser().parse_args(argv)
    decimation = _viz_decimation(args.viz_hz)
    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation)
    try:
        controller.run(max_cycles=args.duration)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted.")
    finally:
        controller.finalize_telemetry()

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

        controller.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Run all new tests**

```bash
pytest test_dual_mode_runner.py -v
```

Expected: all 15 tests `PASSED`

- [ ] **Step 7: Commit**

```bash
git add main.py test_dual_mode_runner.py
git commit -m "feat: add --duration and --hold CLI flags to main.py"
```

---

## Task 4: Update `Progress/Custom_Command.md`

**Files:**
- Modify: `Progress/Custom_Command.md`

- [ ] **Step 1: Add `--duration` section after `--viz-hz`**

In `Progress/Custom_Command.md`, insert after the `--viz-hz` section (after line 48, before `## Examples`):

```markdown
---

### `--duration CYCLES`

| Property | Value |
|---|---|
| Type | integer |
| Default | `1000` |
| Unit | HeartBeat cycles (100 Hz loop iterations) |

**Effect:** `run()` executes exactly `CYCLES` active control cycles. Warmup cycles are separate and not counted.

| `--duration` | Simulation time equivalent | Use case |
|---|---|---|
| `500`  | 5 s  | Smoke test |
| `1000` | 10 s | Default |
| `2000` | 20 s | Extended stability check |
| `5000` | 50 s | Marathon run |

---

### `--hold`

| Property | Value |
|---|---|
| Type | boolean (store_true) |
| Default | off |
| Requires | `--gui` (prints warning and skips if no GUI client) |

**Effect:** After the Performance Summary is printed and the telemetry CSV is flushed, enters a low-priority physics loop at ~100 Hz. The GUI window stays open and fully interactive (camera rotate/zoom). Robot remains physically simulated — joints settle under gravity. Exit with `Ctrl-C`.

```

- [ ] **Step 2: Replace the `## Examples` section**

Replace the existing `## Examples` block with:

```markdown
## Examples

```bash
# Headless smoke test — 500 cycles, maximum physics throughput
python3 main.py --duration 500

# GUI at default 10 Hz, default 1000 cycles
python3 main.py --gui

# GUI at ~33 Hz render rate (smoother, higher GPU load)
python3 main.py --gui --viz-hz 33

# GUI at 1 Hz — slow-motion, useful for inspecting debug markers
python3 main.py --gui --viz-hz 1

# Marathon run — 5000 cycles, no hold
python3 main.py --gui --duration 5000

# Hold inspect — 2000 cycles, keep window open to check final pose
python3 main.py --gui --duration 2000 --hold

# Slow-motion hold — 500 cycles at 1 Hz render, then inspect
python3 main.py --gui --viz-hz 1 --duration 500 --hold
```
```

- [ ] **Step 3: Add Run Profiles reference section at the end of the file**

Append to `Progress/Custom_Command.md`:

```markdown
---

## Run Profiles

| Profile | Command | Purpose |
|---|---|---|
| Smoke Test | `python3 main.py --duration 500` | Fast headless sanity check after a code change |
| Standard | `python3 main.py --gui --duration 1000` | Default interactive run |
| Marathon | `python3 main.py --gui --duration 5000` | Extended stability / jitter measurement |
| Hold Inspect | `python3 main.py --gui --duration 2000 --hold` | Inspect 687 mm shank final position |
| Debug Slow-mo | `python3 main.py --gui --viz-hz 1 --duration 200 --hold` | Frame-by-frame visual debug |
```

- [ ] **Step 4: Commit**

```bash
git add Progress/Custom_Command.md
git commit -m "docs: add --duration, --hold, and Run Profiles to Custom_Command.md"
```

---

## Final Verification

- [ ] **Run full test suite**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_dual_mode_runner.py -v
```

Expected output (15 tests):
```
test_dual_mode_runner.py::test_decimation_default_10hz PASSED
test_dual_mode_runner.py::test_decimation_33hz PASSED
test_dual_mode_runner.py::test_decimation_clamp_above_100 PASSED
test_dual_mode_runner.py::test_decimation_clamp_below_1 PASSED
test_dual_mode_runner.py::test_decimation_at_1hz PASSED
test_dual_mode_runner.py::test_default_no_gui PASSED
test_dual_mode_runner.py::test_gui_flag PASSED
test_dual_mode_runner.py::test_default_viz_hz PASSED
test_dual_mode_runner.py::test_viz_hz_parsed PASSED
test_dual_mode_runner.py::test_main_passes_decimation_to_controller PASSED
test_dual_mode_runner.py::test_finalize_telemetry_stops_and_joins_thread PASSED
test_dual_mode_runner.py::test_shutdown_skips_join_when_already_finalized PASSED
test_dual_mode_runner.py::test_default_duration PASSED
test_dual_mode_runner.py::test_duration_parsed PASSED
test_dual_mode_runner.py::test_default_hold PASSED
test_dual_mode_runner.py::test_hold_flag PASSED
test_dual_mode_runner.py::test_hold_with_gui_steps_physics PASSED
test_dual_mode_runner.py::test_hold_without_gui_prints_warning PASSED
```
