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

## Examples

```bash
# Headless — maximum physics throughput, 50-cycle warmup
python3 main.py

# GUI at default 10 Hz render rate
python3 main.py --gui

# GUI at ~33 Hz render rate (smoother, higher GPU load)
python3 main.py --gui --viz-hz 33

# GUI at 1 Hz — slow-motion, useful for inspecting debug markers
python3 main.py --gui --viz-hz 1
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
