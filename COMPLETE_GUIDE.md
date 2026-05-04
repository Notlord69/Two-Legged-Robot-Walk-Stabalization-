# Siclo1 — Complete Developer Guide

## System at a Glance

- **Robot:** 8 kg bipedal, 2-DOF legs (hip-pitch + knee), custom Fusion 360 URDF
- **Simulator:** PyBullet (DIRECT or GUI mode)
- **Loop rate:** 100 Hz, 10 ms hard budget per cycle
- **URDF:** `Siclo1_Primitive.urdf` — cylinder primitives, physics-correct inertials
- **Controller entry point:** `main.py` → `HeartBeat.py` → 12-stage pipeline

---

## Running the Simulation

```bash
# Headless, default 1000 cycles
python3 main.py

# GUI window, 30 Hz render rate, 5000 cycles, keep open after
python3 main.py --gui --viz-hz 30 --duration 5000 --hold

# Walk 2 m headless with terminal telemetry
python3 main.py --walk 2.0 --on

# Walk 2 m with GUI
python3 main.py --gui --walk 2.0
```

Post-run analysis:
```bash
python3 analyze.py sessions/<folder>/         # save 4 PNGs
python3 analyze.py sessions/<folder>/ --show  # also open interactively
```

---

## 12-Stage Control Pipeline

Every 10 ms `HeartBeat._run_one_cycle()` executes these stages in order.
Each stage reads from and writes to `shared_state` exclusively.

```
Stage  Name             Who runs it          What it does
─────  ───────────────  ───────────────────  ────────────────────────────────
  1    sensors          HeartBeat.py         p.getJointState, getLinkState,
                                             base pose, foot contact normals
  2    link_positions   HeartBeat.py         getLinkState for every link →
                                             shared_state.link_positions dict
  3    perception       perception.py        Per-foot 4-state contact FSM;
                                             slip detection
  4    stability        stability.py         2D FK fallback, Shapely support
                                             polygon, LIPM capture point,
                                             STABLE/MARGINAL/UNSTABLE
  5    balance          balance_controller   Lateral roll PID + sagittal pitch
                                             PID; emergency torque injection
                                             when CP error > 8 cm
  6    grf              grf.py               Spring-damper Fz + 2-link sagittal
                                             Jacobian torque correction on
                                             stance feet; inactive in IDLE
  7    gait_planner     gait_planner.py      5-phase step FSM per leg;
                                             idle-stance ramp (50 cycles)
  8    mission          mission.py           5-state mission FSM; ramp_gain
                                             [0→1] over 50 cycles on RAMP,
                                             [1→0] over 20 on DECEL
  9    wbc              HeartBeat.py         Joint-space PD for 8 joints:
                                             KP=30 N·m/rad, KD=10 N·m·s/rad
 10    recovery         recovery.py          5-priority watchdog; triggers
                                             ABORT_HOLD / REPOSITION /
                                             EMERGENCY_STOP
 11    apply_control    HeartBeat.py         Sums WBC + balance + GRF torques
                                             → sim/interface.set_joint_*
 12    step_sim         HeartBeat.py         sim/interface.step_simulation()
```

---

## Module Reference

### shared_state.py — Single Source of Truth

All inter-module data lives here as attributes of the `shared_state` singleton.
No module writes directly to another module's variables.

Key enums: `ContactState`, `MissionState`, `StepPhase`, `StabilityStatus`,
`RecoveryAction`, `PrimaryRegime`, `Condition`

Key constants (URDF-locked — do not change without updating URDF):
```python
DEFAULT_LINK_DATA      # masses, lengths, COMs from URDF
URDF_JOINT_NAMES       # ordered joint name list
URDF_JOINT_LIMITS      # lower/upper/effort per joint
```

Thread-safe setters: `set_contact_state()`, `set_stability_status()`,
`set_recovery_action()`, `set_slip_detection()`

Telemetry: 72-column `TelemetryRingBuffer` — zero-allocation numpy-backed ring.

---

### sim/interface.py — PyBullet Abstraction Layer

**Rule:** All `p.*` calls must go through this file. Never call `p.getJointState()`
or `p.stepSimulation()` inline in control logic.

URDF parsers (`get_joint_limits`, `get_segment_lengths`) are pure stdlib —
importable without PyBullet.

```python
from sim.interface import (
    get_joint_limits,          # parse URDF → {name: {lower, upper}}
    get_segment_lengths,       # parse URDF → {left/right: {thigh, shank}} m
    get_joint_state,           # (body_id, joint_index) → (pos_rad, vel_rad_s)
    set_joint_position_target, # PD position control wrapper
    step_simulation,           # p.stepSimulation()
    capture_frame,             # TinyRenderer → (H,W,3) uint8 RGB
    add_debug_line,
    remove_debug_line,
)
```

Active URDF path: `Siclo1_Primitive.urdf` (hardcoded in `_URDF_PATH`).

---

### kinematics.py — Analytic IK / FK

```python
L_THIGH = 0.390   # m
L_SHANK = 0.360   # m
R_MIN   = 0.035   # m  (singularity buffer)
R_MAX   = 0.745   # m  (thigh + shank − 5 mm)

# Solve IK: 3D foot target → (q_hip_inwards, q_hip_forwards, q_knee)
solve_ik(foot_target_xyz, side='left'|'right') -> tuple[float,float,float]

# Forward kinematics: joint angles → foot position
forward_kinematics(q_hip_inwards, q_hip_forwards, q_knee, side) -> np.ndarray
```

Axis-sign convention: Left hip/knee axis = −X (angles negated in Jacobian);
Right hip/knee axis = +X (angles used as-is).

---

### gait_planner.py — Gait FSM

5-phase step cycle:
```
DOUBLE_SUPPORT → COM_SHIFT → LIFT → SWING → PLACE
```

Idle stance (ramp-in over 50 cycles):
```python
IDLE_FALLBACK_LEFT  = (0.0,  0.3253, -0.6711, 0.0)  # hip_in, hip_fwd, knee, ankle
IDLE_FALLBACK_RIGHT = (0.0, -0.3253,  0.6711, 0.0)
```
These angles place feet at 95 % of R_MAX (≈ 0.708 m leg extension).

`swing_side` alternates per step. Step targets computed via IK from
`target_foot_position` in `shared_state`.

---

### mission.py — Mission FSM

```
IDLE ──(--walk D)──► RAMP ──(ramp done)──► WALK ──(dist reached)──► DECEL ──► STOP
```

`ramp_gain` scales WBC and GRF torques: 0.0 at IDLE/beginning of RAMP, 1.0 during WALK.
`RAMP_RATE = 1/50` (50-cycle, 0.5 s ramp-up). `STOP_RATE = 1/20`.

---

### balance_controller.py — LIPM Balance

- **Lateral:** Roll PID on CP lateral error → hip inwards torque correction.
  `LATERAL_ROLL_GAIN = 0.8`
- **Sagittal:** Pitch PID on CP sagittal error → hip-pitch offset.
  `SAGITTAL_PITCH_GAIN = 1.2`
- **Emergency:** Direct torque injection if `|CP_error| > EMERGENCY_THRESHOLD = 0.08 m`.

Capture point: `CP = com_xy + v_com_xy / ω`, where `ω = sqrt(g / z_com)`.

---

### grf.py — Ground Reaction Force Controller

Virtual spring-damper + Jacobian torque correction for stance legs.

```python
F_z = GRAVITY_COMP + K_SPRING * (Z_REST - leg_ext) - B_DAMPER * z_dot_foot
# K_SPRING = 1589 N/m, B_DAMPER = 94 N·s/m, Z_REST = 0.75 m
```

Sagittal Jacobian (hip-pitch + knee only, no ankle):
```
τ_hip  = -(L_thigh·sin(θ_hip) + L_shank·sin(θ_hip+θ_knee)) · F_z
τ_knee = -L_shank·sin(θ_hip+θ_knee) · F_z
```

GRF is suppressed during IDLE and on the swing leg during LIFT/SWING/PLACE.
DECEL state boosts K_SPRING by 20 % to absorb stopping impulse.

---

### stability.py — COM Stability Monitor

Reads `link_positions` and `joint_positions` → computes 2D FK fallback →
builds Shapely support polygon from confirmed contact points →
classifies `STABLE / MARGINAL / UNSTABLE`.

Also computes LIPM capture point and checks against polygon.

Outputs: `com_position`, `stability_margin`, `stability_status`,
`capture_point`, `current_safety_margin`.

---

### perception.py — Contact State Machines

Per-foot 4-state FSM:
```
NO_CONTACT → TOUCH_EXPECTED → CONTACT_TENTATIVE → CONTACT_CONFIRMED
```

Confirmation gate: `contact_ticks >= 3` AND `foot_flat == True`.
Slip detection: sudden force drop (>50 %) or lateral velocity > 0.3 m/s.

---

### recovery.py — Recovery Watchdog

Evaluates 5 conditions in priority order every cycle:

| Priority | Condition | Action |
|----------|-----------|--------|
| 1 | `is_unstable AND step_duration > timeout` | EMERGENCY_STOP |
| 2 | Previously-confirmed contact lost | ABORT_HOLD |
| 3 | Both feet unconfirmed AND not IDLE AND timed out | REPOSITION |
| 4 | Any foot slipping | REPOSITION |
| 5 | MARGINAL stability AND step timed out | ABORT_HOLD |

Step timer is reset every cycle while `mission_state == IDLE` so
watchdog checks 1, 3, 5 never fire at rest.

---

### telemetry.py — Ring Buffer + CSV Logger

`TelemetryRingBuffer`: 72 columns, numpy-backed, zero allocation per cycle.
`TelemetryThread`: daemon thread, drains ring at 10 Hz → `telemetry.csv`.

Also writes `regime.csv` (per-cycle regime classification) and
`summary.txt` (run summary with violation count and regime breakdown).

---

### regime_monitor.py — Operating Regime Classifier

11 `PrimaryRegime` values:
`STARTUP / WARMUP / IDLE_STANDING / IDLE_PERTURBED / RAMP_UP / WALKING_STABLE /
WALKING_DEGRADED / DECEL / STOPPING / RECOVERY / FALLEN`

4 `Condition` values: `NOMINAL / DEGRADED / CRITICAL / FALLEN`

`RegimeMonitor.classify()` scores each signal with per-signal confidence and
returns `(PrimaryRegime, Condition)` each cycle.

---

### sim/viz_bridge.py — GUI Subprocess Bridge

Writes robot pose to a shared-memory float64 buffer; a daemon subprocess
(`viz/gui_worker.py`) reads it at `viz_fps` Hz and mirrors to a `p.GUI` window.

Zero GIL contention — all `p.*` calls stay in the subprocess.
Physics runs headlessly if subprocess fails to start (timeout 5 s).

---

### recorder.py — Video Recorder

`VideoRecorder` daemon thread: captures frames at 15 Hz via TinyRenderer
(`p.getCameraImage`, software-only, no GPU) and writes `walk.mp4` using OpenCV.
Works in both DIRECT and GUI mode.

---

### analyze.py — Post-run Analysis

```bash
python3 analyze.py sessions/<folder>/
```

Reads `telemetry.csv` and writes 4 PNG figures:
`com_trajectory.png`, `contact_forces.png`, `timing.png`, `stability.png`.

---

## WBC Joint Mapping

```python
# Left leg (URDF axis = -X → sign_flip = -1)
Left_Hip_Inwards   index 0   sign -1
Left_Hip_Forwards  index 1   sign -1
Left_Knee          index 2   sign -1
Left_Ankle         index 3   sign -1

# Right leg (URDF axis = +X → sign_flip = +1)
Right_Hip_Inwards  index 0   sign +1
Right_Hip_Fowards  index 1   sign +1   ← URDF typo preserved
Right_Knee         index 2   sign +1
Right_Ankle        index 3   sign +1
```

Target joint angles are provided by `gait_planner` via `shared_state.wbc_targets`.
WBC applies: `τ = KP*(q_target − q) − KD*q_dot` clamped to URDF effort limits.

---

## Timing Budget

```
100 Hz = 10.0 ms per cycle

Estimated allocation:
  Sensor reads      ~0.5 ms
  Perception        ~0.2 ms
  Stability         ~0.4 ms
  Balance + GRF     ~0.3 ms
  Gait + Mission    ~0.2 ms
  WBC + apply       ~0.3 ms
  step_simulation   ~5.0 ms   (PyBullet physics)
  Overhead          ~1.0 ms
  ───────────────────────────
  Budget remaining  ~2.1 ms
```

Violations are counted in `shared_state.timing_violations` and flagged in
`telemetry.csv` (`err` column). If a violation triggers `freeze_robot`,
HeartBeat enters Safe Freeze — all joint targets hold, no new commands.

---

## Shared State: Persistent Variables Across Cycles

Any value that must survive from one 100 Hz cycle to the next belongs in
`Siclo1State`. Key fields:

```python
# Timing
sim_time                    # s, wall-clock simulation time
cycle_count                 # int, monotonic
timing_violations           # int, cumulative

# Physics
com_position                # (3,) m
com_velocity                # (3,) m/s
base_position               # (3,) m
base_orientation            # (4,) quaternion
joint_positions             # {name: rad}
joint_velocities            # {name: rad/s}
link_positions              # {name: (3,) m}

# Contact
left_foot_position          # (3,) m
left_foot_velocity          # (3,) m/s
left_foot_force             # N
left_foot_contact_state     # ContactState
left_contact_ticks          # int, 3-tick gate
left_foot_flat              # bool
right_* (same)

# Control
wbc_targets                 # {joint_name: rad}
grf_torque_correction       # {joint_name: N·m}
balance_torque_correction   # {joint_name: N·m}
ramp_gain                   # [0, 1]
mission_state               # MissionState
step_phase                  # StepPhase
stance_side                 # 'left' | 'right'

# Safety
freeze_robot                # bool
emergency_stop_triggered    # bool
recovery_action             # RecoveryAction
recovery_active             # bool
recovery_reason             # str
```

---

## Common Issues

**Robot launches off the ground on spawn**
URDF inertial COM is outside the cylinder geometry bounds. `Siclo1_Primitive.urdf`
fixes this. Ensure `HeartBeat.py` loads `Siclo1_Primitive.urdf`, not `Siclo1.urdf`.

**Timing violations every cycle**
Profile with `python3 -m cProfile main.py`. Most common cause: `p.stepSimulation`
slower than expected. Reduce `--viz-hz` or run headless.

**Recovery always triggers at startup**
`mission_state == IDLE` resets the step timer every cycle. If recovery still
fires, check `shared_state.emergency_stop_triggered` and error log with
`shared_state.print_status(verbose=True)`.

**IK returns `None`**
Foot target is outside the reachable workspace (`R_MAX = 0.745 m`).
Check `target_foot_position` in shared_state.

**GUI subprocess never becomes ready**
`VizBridge` timeout is 5 s. Falls back to headless silently. On WSL2, check that
an X server or WSLg is running.
