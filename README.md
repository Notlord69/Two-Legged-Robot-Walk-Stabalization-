# Siclo1 — Bipedal Robot Simulation

**8 kg** bipedal robot, custom Fusion 360 URDF, PyBullet simulation, Python 3.10+.
Simulation-only — no hardware targets.

---

## Quick Start

```bash
pip install -r requirements.txt

# Headless, 1000 cycles
python3 main.py

# GUI at 10 Hz
python3 main.py --gui

# Walk 2.0 m then stop
python3 main.py --walk 2.0

# GUI + walk + show telemetry output
python3 main.py --gui --walk 2.0 --on
```

### CLI Reference

| Flag | Default | Description |
|------|---------|-------------|
| `--gui` | off | Enable PyBullet GUI + debug visualisation |
| `--viz-hz HZ` | 10 | GUI render rate (1–100 Hz) |
| `--duration N` | 1000 | Number of 100 Hz cycles to run |
| `--hold` | off | Keep GUI open after run for pose inspection |
| `--walk D` | off | Walk `D` metres then stop |
| `--on` | off | Print telemetry to terminal (CSV always written) |

### Post-run Analysis

```bash
# Generate 4 PNG figures from telemetry.csv
python3 analyze.py sessions/2026-05-04_12-00-00/

# Open figures interactively (requires X11)
python3 analyze.py sessions/2026-05-04_12-00-00/ --show
```

---

## File Structure

```
Siclo1_V1/
├── main.py                 CLI entry point — argparse, launches Siclo1Controller
├── HeartBeat.py            100 Hz heartbeat — 12-stage pipeline, WBC, sensor reads
├── shared_state.py         Single source of truth — all inter-module data
│
├── Siclo1_Primitive.urdf   Active URDF — cylinder primitives with corrected inertials
├── Siclo1.urdf             Original mesh URDF (retained as reference, not loaded)
│
├── kinematics.py           Analytic 2-link IK/FK — L_THIGH=0.390m, L_SHANK=0.360m
├── gait_planner.py         5-phase gait FSM — DOUBLE_SUPPORT/COM_SHIFT/LIFT/SWING/PLACE
├── mission.py              5-state mission FSM — IDLE/RAMP/WALK/DECEL/STOP
├── balance_controller.py   LIPM capture-point balance — lateral roll + sagittal pitch PID
├── grf.py                  Ground-reaction-force controller — spring-damper + Jacobian
├── stability.py            COM / support-polygon stability monitor
├── perception.py           Per-foot contact FSM — 4 states, slip detection
├── recovery.py             Recovery controller — 5-priority watchdog
│
├── telemetry.py            72-column ring buffer, CSV logger, 10 Hz drain thread
├── regime_monitor.py       11 operating regimes, 4 conditions, per-signal confidence
├── recorder.py             MP4 video recorder — TinyRenderer, 15 Hz, daemon thread
├── analyze.py              Post-run analysis — 4 matplotlib figures from telemetry.csv
├── replay.py               Replay recorded sessions
├── pose_logger.py          Per-cycle joint-angle logger
│
├── sim/
│   ├── interface.py        PyBullet abstraction layer — all p.* calls live here
│   └── viz_bridge.py       Subprocess-based GUI bridge — shared-memory pose buffer
│
├── viz/
│   ├── gui_worker.py       GUI subprocess entry point
│   └── debug_markers.py    PyBullet debug-line helpers
│
├── Test_Enviroment/        Test suite (pytest)
└── docs/                   Design specifications
```

---

## Architecture

### 100 Hz Control Pipeline (HeartBeat.py)

```
┌────────────────────────────────────────────────────────────────────┐
│  Stage      Module              Action                             │
├────────────────────────────────────────────────────────────────────┤
│  1  sensors        HeartBeat      PyBullet reads via sim/interface │
│  2  link_positions HeartBeat      getLinkState for all links       │
│  3  perception     perception.py  Per-foot contact FSM             │
│  4  stability      stability.py   COM / support-polygon check      │
│  5  balance        balance_ctrl   LIPM capture-point PID           │
│  6  grf            grf.py         Spring-damper + Jacobian torques │
│  7  gait_planner   gait_planner   5-phase step FSM                 │
│  8  mission        mission.py     5-state mission FSM, ramp_gain   │
│  9  wbc            HeartBeat      Joint-space PD — KP=30 KD=10     │
│  10 recovery       recovery.py    5-priority watchdog              │
│  11 apply_control  HeartBeat      Torques → sim/interface          │
│  12 step_sim       HeartBeat      sim/interface.step_simulation()  │
└────────────────────────────────────────────────────────────────────┘
Hard limit: 10 ms per cycle. Violation → Safe Freeze.
```

### Module Priority Order (safety → balance → locomotion)

```
Safety / Recovery
    └─► Balance (LIPM / Capture Point)
            └─► Load Stability
                    └─► Gait / Swing
                            └─► WBC (Whole Body Controller)
```

### Data Bus

All inter-module data lives in `shared_state.Siclo1State` (singleton).
No direct module-to-module calls — all communication is via reads/writes on `shared_state`.

---

## Key Constants

| Constant | Value | Source |
|----------|-------|--------|
| Loop rate | 100 Hz | HeartBeat.py |
| Cycle budget | 10 ms | HeartBeat.py |
| L_THIGH | 0.390 m | kinematics.py, URDF joint origin |
| L_SHANK | 0.360 m | kinematics.py, URDF joint origin |
| R_MAX | 0.745 m | kinematics.py (thigh+shank−5 mm buffer) |
| WBC_KP | 30.0 N·m/rad | HeartBeat.py |
| WBC_KD | 10.0 N·m·s/rad | HeartBeat.py |
| URDF spawn Z | −0.14 m | HeartBeat.py |
| Robot mass | 8.0 kg | DEFAULT_LINK_DATA |
| GRF K_SPRING | 1589 N/m | grf.py (m·g / 5 cm max compression) |

---

## URDF: Siclo1_Primitive.urdf

The active simulation file. Leg links use cylinder collision/visual primitives.
Inertial sections were recomputed from first principles after the original
Fusion 360 mesh values caused phantom COM positions outside link bounds — the
root cause of the robot launching off the ground on spawn.

Key corrections vs. the original `Siclo1.urdf`:

| Link | Old COM z | New COM z | Issue fixed |
|------|-----------|-----------|-------------|
| Left_Lower_Leg_1 | −0.415 m | −0.180 m | COM 55 mm outside 0.360 m shank |
| Right_Lower_Leg_1 | −0.495 m | −0.180 m | COM 135 mm outside shank |
| Left_Upper_Leg_1 | off-axis | 0 0 −0.195 | Axis-aligned COM |
| Right_Upper_Leg_1 | off-axis | 0 0 −0.195 | Axis-aligned COM |

Inertia tensors were recalculated using cylinder formulas:
`Ixx = Iyy = m(3r² + h²)/12`, `Izz = mr²/2`, all cross-products = 0.

---

## Test Suite

```bash
# Run all tests
cd Test_Enviroment
pytest -v

# Key test files
test_idle_stance.py      # Idle stance IK targets and WBC convergence
test_wbc_tracking.py     # WBC PD tracking accuracy
```

---

## Session Output

Each run writes a timestamped session folder under `sessions/`:

```
sessions/2026-05-04_12-00-00/
├── telemetry.csv     72-column per-cycle telemetry ring buffer
├── regime.csv        Per-cycle regime classification
├── summary.txt       Run summary (duration, violations, regime breakdown)
└── walk.mp4          Video recording (if --record flag active)
```

---

## Simulator Abstraction

All PyBullet calls go through `sim/interface.py`. Direct `p.*` calls in
control logic are prohibited. This enables a future Gazebo swap by changing
only that file.

---

## Future: ROS2 + Gazebo

The PyBullet layer is the only Gazebo-incompatible code. Swap plan:
1. Replace `sim/interface.py` with a ROS2/Gazebo equivalent.
2. Replace `sim/viz_bridge.py` with an RViz publisher.
3. `HeartBeat.py` and all control modules are simulator-agnostic.
