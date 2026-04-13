# Siclo1 — Custom Commands

## Entry Point

```bash
python3 main.py [FLAGS]
```

---

## Flags

### `--gui`

| Property | Value |
|---|---|
| Type | boolean (store_true) |
| Default | off — runs headless (`p.DIRECT`) |

**Effect when set:**
- Launches a second PyBullet client (`p.GUI`) as a visual window
- Enables debug visualisation: sagittal-plane annulus arcs (R_min / R_max) and hip→foot vectors
- Reduces physics warmup from 50 cycles to 5 cycles
- Adds a 2-second startup pause for the X-server window to appear (WSL)

---

### `--viz-hz HZ`

| Property | Value |
|---|---|
| Type | integer |
| Default | `10` |
| Range | 1–100 (silently clamped outside this range) |
| Requires | `--gui` (ignored in headless mode) |

**Effect:** Controls how often the GUI window refreshes.

The physics loop always runs at 100 Hz. The GUI renders every `100 ÷ viz_hz` physics cycles.

| `--viz-hz` | Render every N cycles | Effective GUI Hz |
|---|---|---|
| `1`  | 100 cycles | 1 Hz (slow-motion debug) |
| `10` | 10 cycles  | 10 Hz (default) |
| `33` | 3 cycles   | ~33 Hz (smooth) |
| `100`| 1 cycle    | 100 Hz (every cycle) |

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

---

### `--on`

| Property | Value |
|---|---|
| Type | boolean (store_true) |
| Default | off — terminal output is silent by default |

**Effect when set:** Enables live telemetry output in the terminal — the per-cycle log dump and the final `SIMULATION COMPLETE` summary block are printed to stdout.

**Without `--on` (default):** The terminal is silent. All data is still written to:
- `sessions/<timestamp>/telemetry.csv` — every cycle, always
- `sessions/<timestamp>/summary.txt` — timing stats + the exact command used, always

The `summary.txt` always records the command that was used to start the simulation (e.g. `python3 main.py --gui --walk 2.0`) so you can identify session conditions without needing terminal output.

---

## Examples

```bash
# Headless smoke test — 500 cycles, silent terminal (CSV/summary still written)
python3 main.py --duration 500

# GUI at default 10 Hz, default 1000 cycles, silent terminal
python3 main.py --gui

# GUI at default 10 Hz, walking distance of 2 meters, silent terminal
python3 main.py --gui --walk 2.0

# Same walk, but with full telemetry printed to terminal
python3 main.py --gui --walk 2.0 --on

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

# Headless run with terminal output enabled
python3 main.py --duration 1000 --on
```

---

## Debug Visualisation (GUI mode only)

When `--gui` is active, three overlays appear in the PyBullet window:

| Overlay | Colour | Meaning |
|---|---|---|
| Annulus arcs | Light blue | R_min (0.631 m) and R_max (0.743 m) reachable workspace per hip |
| Hip → Foot (actual) | Green | Current foot position relative to hip-pitch joint |
| Hip → Foot (target) | Red | IK foot target (hidden until gait planner sets a target) |

The arcs follow the robot as it moves. Lines are replaced in-place each render tick (no accumulation).

---

## Run Profiles

| Profile | Command | Purpose |
|---|---|---|
| Smoke Test | `python3 main.py --duration 500` | Fast headless sanity check; check CSV after |
| Standard | `python3 main.py --gui --duration 1000` | Default interactive run; silent terminal |
| Verbose | `python3 main.py --gui --duration 1000 --on` | Same, with live telemetry in terminal |
| Marathon | `python3 main.py --gui --duration 5000` | Extended stability / jitter measurement |
| Hold Inspect | `python3 main.py --gui --duration 2000 --hold` | Inspect 687 mm shank final position |
| Debug Slow-mo | `python3 main.py --gui --viz-hz 1 --duration 200 --hold` | Frame-by-frame visual debug |
| Walk + Log | `python3 main.py --gui --walk 2.0 --on` | Walk 2 m with full terminal output |
