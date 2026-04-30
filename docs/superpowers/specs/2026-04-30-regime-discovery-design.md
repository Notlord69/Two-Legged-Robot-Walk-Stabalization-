# Regime Discovery Specification — Siclo1 RegimeMonitor

**Date:** 2026-04-30
**Status:** Design approved, pending implementation
**Observer rate:** 10 Hz (telemetry consumer thread)

---

## 1. Architecture

### Position in the Stack

```
HeartBeat.step() [100 Hz]
  └─ telemetry.write(row)  ──►  TelemetryRingBuffer (72 cols)
                                        │
                                        ▼
                          TelemetryThread.run() [10 Hz]
                            └─ read_batch()
                            └─ RegimeMonitor.classify(row)
                                  │
                                  ├─ primary_regime   (enum, deterministic lookup)
                                  ├─ condition         (NOMINAL / DEGRADED / CRITICAL / FALLEN)
                                  └─ confidence_vec    (per-signal float ∈ [0, 1])
```

The RegimeMonitor is a **pure observer**. It reads from the 72-column telemetry ring
buffer and produces a classification tuple. It does not write to `shared_state` and has
no effect on control decisions. The existing FSM states (`StepPhase`, `MissionState`)
continue to gate all control logic.

### Output Tuple

```python
(primary_regime: PrimaryRegime, condition: Condition, confidence: dict[str, float])
```

- `primary_regime`: deterministic lookup from telemetry columns 51 (step_phase) and 54 (mission_state)
- `condition`: worst-case aggregation of per-signal confidences
- `confidence`: per-signal confidence values ∈ [0, 1], keyed by signal name

---

## 2. Derivation Basis — Physical Constants

All values derived from source files. Line references included.

```
Robot mass:           m = 8.100127 kg          # shared_state.py:694 LoadConfig.base_mass
Gravity:              g = 9.81 m/s²            # stability.py:66
Torso COM height:     h = 0.8806 m             # shared_state.py:250 base_position[2]
Foot width:           w = 0.029 m              # wiki: siclo1-robot.md (28.6 mm ankle separation)
Thigh length:         L_t = 0.060661 m         # shared_state.py:813 DEFAULT_LINK_DATA['l_thigh']
Shank length:         L_s = 0.686961 m         # shared_state.py:818 DEFAULT_LINK_DATA['l_shank']
Total leg length:     L = L_t + L_s = 0.7476 m # derived
URDF effort limit:    τ_max = 100.0 N·m        # shared_state.py:766-776 (all joints)
WBC gains:            KP = 100.0 N·m/rad       # HeartBeat.py:81
                      KD = 28.0 N·m·s/rad      # HeartBeat.py:82

Total weight force:   W = m × g = 79.5 N       # 8.100127 × 9.81
Per-foot (DS):        W/2 = 39.7 N             # derived
Tipping angle:        arctan((w/2) / h)         # arctan(0.0145 / 0.8806) = 0.94° ≈ 1.0°

LIPM natural freq:    ω₀ = √(g/h)              # √(9.81 / 0.8806) = 3.34 rad/s

GRF spring:           K = 1589.0 N/m           # grf.py:68
GRF damper:           B = 94.0 N·s/m           # grf.py:69
GRF rest length:      Z_REST = 0.75 m          # grf.py:67
DECEL spring boost:   1.2×                     # grf.py:71

Swing height:         0.06 m                   # gait_planner.py:58
Swing duration:       0.50 s                   # gait_planner.py:59
Step length:          0.12 m (0.06 m in DECEL) # gait_planner.py:55, _compute_x_target

Contact thresholds:
  force_threshold_min:       5.0 N             # shared_state.py:710
  force_threshold_confirmed: 15.0 N            # shared_state.py:711
  force_threshold_release:   3.0 N             # shared_state.py:715
  height_threshold:          0.20 m            # shared_state.py:709
  settling_time:             0.025 s           # shared_state.py:712

Balance controller:
  LATERAL_ROLL_GAIN:   0.8 rad/m               # balance_controller.py:58
  HIP_ROLL_MAX:        0.25 rad (~14°)         # balance_controller.py:62
  PITCH_OFFSET_MAX:    0.15 rad (~8.6°)        # balance_controller.py:70
  EMERGENCY_THRESHOLD: 0.08 m                  # balance_controller.py:74
  EMERGENCY_HYSTERESIS: 0.02 m                 # balance_controller.py:78

Phase timeouts:
  DS_MIN_TIME:      0.10 s                     # gait_planner.py:62
  DS_TIMEOUT:       2.0 s                      # gait_planner.py:63
  COM_SHIFT_TIMEOUT: 1.0 s                     # gait_planner.py:64
  COM_SHIFT_THRESHOLD: 0.03 m (lateral)        # gait_planner.py:65
  COM_SHIFT_SAGITTAL: 0.05 m                   # gait_planner.py:79
  LIFT_TIMEOUT:     0.15 s                     # gait_planner.py:66
  SWING_TIMEOUT:    0.75 s (1.5 × 0.50)       # gait_planner.py:67
  PLACE_TIMEOUT:    0.5 s                      # gait_planner.py:68
  PLACE_ENTRY_PHI:  0.85                       # gait_planner.py:77

Recovery thresholds:
  timeout_threshold:   3.0 s                   # shared_state.py:720
  marginal_timeout:    2.0 s                   # shared_state.py:721
  max_recovery_attempts: 3                     # shared_state.py:722
```

### Critical Insight: Tipping Angle

The static tipping angle is **~1°** (arctan(0.0145 / 0.8806)). This means:

- Pitch/roll tolerances for Siclo1 are dramatically tighter than a typical humanoid
- The robot survives tilts beyond 1° only through dynamic balance (CP correction)
- Any acceptable band wider than ±1° relies entirely on active control authority
- In FROZEN (no control), tilts >1° lead to unrecoverable fall

---

## 3. Confidence Function

For each signal, confidence is computed from the deviation from the regime's optimal value:

```python
def compute_confidence(measured: float, optimal: float,
                       acceptable_band: float,
                       threshold_05: float,
                       threshold_00: float) -> float:
    """
    Returns confidence ∈ [0.0, 1.0] with a smooth ramp and an explicit 0.5 knee.

    [1.0]────────┐
                 │  linear ramp 1.0 → 0.5
    [0.5]────────┼────────┐
                 │        │  linear ramp 0.5 → 0.0
    [0.0]────────┼────────┼────────
                 acceptable  0.5     0.0
                   band    threshold threshold
    """
    deviation = abs(measured - optimal)
    if deviation <= acceptable_band:
        return 1.0
    elif deviation <= threshold_05:
        return 0.5 + 0.5 * (threshold_05 - deviation) / (threshold_05 - acceptable_band)
    elif deviation <= threshold_00:
        return 0.5 * (threshold_00 - deviation) / (threshold_00 - threshold_05)
    else:
        return 0.0
```

### Condition Overlay Aggregation

```python
min_confidence = min(confidence_vec.values())

if freeze_robot or emergency_stop:
    condition = FALLEN
elif min_confidence == 0.0:
    condition = CRITICAL
elif min_confidence <= 0.5:
    condition = DEGRADED
else:
    condition = NOMINAL
```

---

## 4. Primary Regime Definitions

### Regime Lookup Table

Determined from telemetry columns:
- `step_phase`: col 51 (StepPhase enum value)
- `mission_state`: col 54 (MissionState enum value)
- `ramp_gain`: col 55

```
FROZEN:           freeze_robot detected (cycle_count stops advancing)
IDLE_STANDING:    mission_state == IDLE
RAMP_UP:          mission_state == RAMP
WALK_DS:          mission_state == WALK  AND step_phase == DOUBLE_SUPPORT
WALK_COM_SHIFT:   mission_state == WALK  AND step_phase == COM_SHIFT
WALK_LIFT:        mission_state == WALK  AND step_phase == LIFT
WALK_SWING:       mission_state == WALK  AND step_phase == SWING
WALK_PLACE:       mission_state == WALK  AND step_phase == PLACE
DECEL_SWING:      mission_state == DECEL AND step_phase ∈ {SWING, PLACE}
DECEL_DS:         mission_state == DECEL AND step_phase ∈ {DOUBLE_SUPPORT, COM_SHIFT, LIFT}
RAMP_DOWN:        mission_state == STOP
```

Priority: FROZEN > all others (checked first).

---

## 5. Per-Regime Telemetry Profiles

### REGIME: IDLE_STANDING

**Source:** mission.py:77-78, gait_planner.py:270-304, stability.py:310-394

**FSM State:** MissionState.IDLE, StepPhase.DOUBLE_SUPPORT

**Physical Description:** Robot standing still on both feet, no gait commands, gravity compensation only.

**Entry Condition:** MissionState → IDLE (ramp_gain reaches 0.0 in STOP, or simulation startup).

**Exit Condition:** walk_distance set AND both feet CONTACT_CONFIRMED → RAMP (mission.py:98-104).

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z (col 5) | 0.88 m | ±0.02 m | ±0.05 m | <0.60 m or >1.05 m |
| base_pitch (from orn) | 0° | ±2° | ±5° | >10° |
| base_roll (from orn) | 0° | ±2° | ±5° | >10° |
| n_contacts (cols 9,10) | 2 | 2 | 1 | 0 |
| contact_force_z (cols 12,13) | 39.7 N | ±12 N (30%) | ±20 N | <5 N or >80 N |
| joint_torque_norm | [TO CALIBRATE] | [TO CALIBRATE] | [TO CALIBRATE] | [TO CALIBRATE] |
| angular_velocity (cols 16-18) | 0 rad/s | ±0.1 rad/s | ±0.5 rad/s | >1.0 rad/s |
| swing_phase (col 52) | 0.0 | 0.0 | >0.0 | >0.1 |
| ramp_gain (col 55) | 0.0 | 0.0 | >0.0 | >0.5 |
| wbc_tracking_error (cols 33-38) | <0.05 rad | <0.1 rad | >0.2 rad | >0.5 rad |

**Derivations:**
- base_z: URDF COM height 0.8806 m. ±0.02 m from quiet standing oscillation (PyBullet solver jitter). 0.60 m = knee-buckle height (~0.8 × L_s). 1.05 m = spawn + bounce ceiling.
- base_pitch/roll: tipping angle 1°; ±2° is 2× static tip margin. 5° = balance controller EMERGENCY threshold equivalent (0.08 m CP error / 0.88 m ≈ 5.2°). 10° = unrecoverable.
- contact_force_z: W/2 = 39.7 N. 30% = oscillation margin. 5 N = force_threshold_min. 80 N ≈ full bodyweight on one foot.
- angular_velocity: standing = near zero. 0.5 = visible wobble. 1.0 = uncontrolled.

**joint_torque_norm calibration:** Run sim in IDLE for 500 cycles, log `np.linalg.norm(list(shared_state.joint_torques.values()))` each cycle, compute mean ± 2σ. Expected: gravity compensation ~10-20 N·m total norm.

#### Violation Signatures

- `base_z` below band: legs buckling — check WBC KP, GRF spring, URDF mass mismatch
- `base_pitch` beyond band: COM ahead/behind support polygon — sagittal balance not correcting
- `base_roll` beyond band: lateral drift — 29 mm feet, even 3° roll moves CP outside polygon
- `contact_force_z` asymmetric (>60 N / <20 N): weight not centered; lateral integrator wound up
- `angular_velocity` nonzero persistent: oscillation — check WBC KD or balance rate limits

#### Transition Risk

- **Entry:** After RAMP_DOWN, residual angular velocity can persist. Balance integrators reset by `reset_balance()`.
- **Exit:** IDLE→RAMP requires both feet CONFIRMED. Decayed tick counts delay ramp start.

#### Siclo1-Specific Fragilities

- 29 mm foot: static tipping angle 1°. PyBullet solver jitter can briefly push CP outside polygon. Integral term (KI=0.05) accumulates during micro-corrections.
- Single contact point common in PyBullet. `_compute_foot_flat` falls back to pitch check (<7°).

---

### REGIME: RAMP_UP

**Source:** mission.py:106-111, gait_planner.py:294

**FSM State:** MissionState.RAMP, StepPhase.DOUBLE_SUPPORT

**Physical Description:** Robot standing on both feet, torque authority increasing 0→100% over 0.5 s (50 cycles). GRF and gait planner gated until ramp_gain=1.0.

**Entry Condition:** IDLE→RAMP when walk_distance set AND both CONTACT_CONFIRMED.

**Exit Condition:** ramp_gain=1.0 → WALK (mission.py:110-111).

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.88 m | ±0.03 m | ±0.06 m | <0.60 m or >1.05 m |
| base_pitch | 0° | ±3° | ±6° | >10° |
| base_roll | 0° | ±3° | ±6° | >10° |
| n_contacts | 2 | 2 | 1 | 0 |
| contact_force_z | 39.7 N | ±15 N (38%) | ±25 N | <5 N or >80 N |
| ramp_gain (col 55) | 0.0→1.0 | monotonic increase | decreasing or stalled >0.5 s | stuck at 0.0 |
| angular_velocity | 0 rad/s | ±0.3 rad/s | ±0.8 rad/s | >1.5 rad/s |
| wbc_tracking_error | <0.1 rad | <0.2 rad | >0.3 rad | >0.5 rad |

**Derivations:**
- Wider bands than IDLE: GRF spring engagement (K=1589 N/m) causes transient height/force variation as ramp_gain multiplies torques.
- ramp_gain: RAMP_RATE = 1/50 per cycle. 0→1 in exactly 50 cycles (0.5 s). Stall or decrease means controller stuck.

#### Violation Signatures

- `ramp_gain` stuck: mission controller not advancing — check freeze_robot, emergency_stop
- `base_z` dropping: GRF spring settling. Below 0.82 m = spring constant too low for robot mass.
- `contact_force` asymmetric: GRF torques unbalanced L/R — check URDF mirroring

#### Transition Risk

- **Entry:** Stable — both feet confirmed is the gate.
- **Exit:** ramp_gain=1.0 allows DS→COM_SHIFT immediately. Unsettled ramp transients can cause first COM_SHIFT failure.

#### Siclo1-Specific Fragilities

- GRF spring at full gain: K=1589 N/m on 5 cm deflection = 79.5 N per leg. The initial torque "kick" at ramp start can excite pitch oscillation. STOP_RATE (1/20) is 2.5× faster than RAMP_RATE (1/50), so ramp-down is more abrupt.

---

### REGIME: WALK_DS

**Source:** gait_planner.py:270-304

**FSM State:** MissionState.WALK, StepPhase.DOUBLE_SUPPORT

**Physical Description:** Brief double-support between steps. Both feet grounded, weight approximately centered. Stance foot anchor locked for next step.

**Entry Condition:** PLACE→DS via `_complete_step()` — sides flip, step_count increments.

**Exit Condition:** Both CONFIRMED + timer ≥ 0.10 s + ramp_gain=1.0 + force ratio ≤ 2.0 → COM_SHIFT.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.86 m | ±0.04 m | ±0.08 m | <0.60 m or >1.05 m |
| base_pitch | 0° | ±4° | ±8° | >12° |
| base_roll | 0° | ±4° | ±8° | >12° |
| n_contacts | 2 | 2 | 1 | 0 |
| contact_force_z | 39.7 N | ±15 N | ±25 N | <5 N or >80 N |
| angular_velocity | ±0.2 rad/s | ±0.5 rad/s | ±1.0 rad/s | >2.0 rad/s |
| swing_phase (col 52) | 0.0 | 0.0 | >0.0 | >0.1 |
| step_phase_timer | 0.0→0.10+ s | 0.0–0.5 s | >1.0 s | >2.0 s (freeze) |

**Derivations:**
- base_z lower than IDLE (0.86 vs 0.88): legs slightly bent from prior swing phase settling.
- Wider pitch/roll bands: residual from prior step dynamics.
- DS_TIMEOUT = 2.0 s triggers `freeze_robot = True`.
- Force ratio: FORCE_BALANCE_RATIO = 2.0, FORCE_BALANCE_FLOOR = 10 N (gait_planner.py:74-75).

#### Violation Signatures

- `step_phase_timer` approaching 2.0 s: DS stuck — contact not confirming or force ratio gate failing
- `contact_force` ratio >2.0: weight not re-centered. Balance integrator may need reset.

#### Transition Risk

- **Entry from PLACE:** Contact may take 30 ms (3 ticks) to confirm. DS 100 ms minimum absorbs this.
- **Exit to COM_SHIFT:** Force ratio gate is the most common failure point. Asymmetric landing → ratio >2.0 → DS stalls → eventually freezes.

#### Siclo1-Specific Fragilities

- Force balance floor = 10 N. Post-contact bounce drops force below 10 N → ratio gate undefined → gate fails → DS extends.

---

### REGIME: WALK_COM_SHIFT

**Source:** gait_planner.py:306-330, balance_controller.py:150-196

**FSM State:** MissionState.WALK, StepPhase.COM_SHIFT

**Physical Description:** COM shifting laterally over stance foot. Both feet grounded, weight transferring. Hip roll driving pelvis tilt.

**Entry Condition:** DS exit (both confirmed, timer ≥ 0.10 s, force balanced).

**Exit Condition:** CP within 0.03 m lateral AND 0.05 m sagittal of stance foot AND stability ≠ UNSTABLE.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.86 m | ±0.04 m | ±0.08 m | <0.60 m or >1.05 m |
| base_pitch | 0° | ±3° | ±6° | >10° |
| base_roll | ±3° | ±6° | ±10° | >14° |
| n_contacts | 2 | 2 | 1 | 0 |
| contact_force_z (stance) | 55→70 N | 40–80 N | <30 N or >90 N | <10 N or >100 N |
| contact_force_z (swing) | 25→10 N | 5–40 N | <3 N (premature) | >50 N (shift failing) |
| CP error lateral | decreasing → <0.03 m | <0.06 m | >0.08 m | >0.15 m |
| CP error sagittal | ±0.03 m | <0.05 m | >0.08 m (EMERGENCY) | >0.15 m |
| hip_roll (stance) | ≈0.10 rad | 0.0–0.25 rad | <0.0 (wrong dir) | saturated at limit |
| angular_velocity | ±0.3 rad/s | ±0.8 rad/s | ±1.5 rad/s | >2.5 rad/s |

**Derivations:**
- base_roll: intentional lateral tilt. HIP_ROLL_MAX = 0.25 rad ≈ 14° (balance_controller.py:62).
- Stance force: weight shifting from 50/50 → stance should reach ~70% of W (55 N).
- CP error: COM_SHIFT_THRESHOLD = 0.03 m (gait_planner.py:65). Must converge below this.
- EMERGENCY_THRESHOLD = 0.08 m (balance_controller.py:74) triggers emergency sagittal torque.

#### Violation Signatures

- `CP error lateral` not converging: balance controller saturated at HIP_ROLL_MAX, or integrator wound up wrong direction
- `contact_force_z (swing)` stuck >20 N: weight not transferring — hip roll too small or GRF fighting shift
- `base_roll` >10°: dangerous lateral tilt — hip roll rate limit (0.03 rad/cycle) may cause overshoot

#### Transition Risk

- **Entry:** Clean from DS. Asymmetric DS force creates lateral bias.
- **Exit:** COM_SHIFT_TIMEOUT = 1.0 s. Force-advance to LIFT with off-center CP is risky.

#### Siclo1-Specific Fragilities

- 29 mm foot: stance foot provides no lateral support polygon width. Entire COM_SHIFT relies on dynamic balance. Any hip roll command discontinuity creates lateral oscillation the narrow foot cannot absorb.
- Wiki: COM_SHIFT previously conflicted with active_balance lateral center. Fixed 2026-04-20 by returning stance foot position during walking phases.

---

### REGIME: WALK_LIFT

**Source:** gait_planner.py:332-359

**FSM State:** MissionState.WALK, StepPhase.LIFT

**Physical Description:** Swing foot unloading, separating from ground. Stance foot absorbing full bodyweight. Double → single support transition.

**Entry Condition:** COM_SHIFT exit (CP converged or timeout with swing unloaded).

**Exit Condition:** swing_force < 5 N AND swing_vel_z < 0.05 m/s AND stance_force ≥ 5 N.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.85 m | ±0.04 m | ±0.08 m | <0.60 m or >1.05 m |
| base_pitch | 0° | ±3° | ±7° | >12° |
| base_roll | ±4° | ±8° | ±12° | >14° |
| n_contacts | 2→1 | 1 or 2 | 0 | 0 sustained >3 cycles |
| contact_force_z (stance) | 79.5 N | 60–100 N | <40 N | <10 N |
| contact_force_z (swing) | →0 N | 0–5 N | >10 N (not unloading) | >30 N sustained |
| swing_foot_z (col 58 or 61) | 0.0→0.01 m | 0.0–0.03 m | <-0.01 m (penetration) | <-0.02 m |
| step_phase_timer | 0→0.15 s | 0–0.15 s | >0.15 s (timeout) | >0.30 s |

**Derivations:**
- Stance force: full bodyweight W = 79.5 N on one foot.
- Swing force: must drop below SWING_UNLOAD_THRESHOLD = 5.0 N (gait_planner.py:71).
- LIFT_TIMEOUT = 0.15 s = 15 cycles (gait_planner.py:66).
- Stance guard: STANCE_LOAD_THRESHOLD = 5.0 N (gait_planner.py:72-73) prevents SWING entry with both feet unloaded (GJK glitch guard).

#### Violation Signatures

- `contact_force_z (swing)` stuck >10 N: weight transfer incomplete. COM_SHIFT may have exited prematurely.
- `stance_force` <40 N: stance foot losing contact — GRF not holding. Check z_hip - z_foot vs Z_REST.
- Timer >0.15 s: LIFT timeout. Aborts to DS if swing still loaded or stance <5 N.

#### Transition Risk

- **Entry:** If COM_SHIFT force-advanced, CP may not be over stance foot. LIFT starts with off-center COM.
- **Exit:** Requires stance_force ≥ 5 N — GJK glitch guard. Without this, SWING starts with both feet unloaded.

#### Siclo1-Specific Fragilities

- LIFT_TIMEOUT = 15 cycles. Combined with 3-tick contact delay: only ~12 usable cycles.
- GJK glitch: both feet can report zero force for 1-2 frames. Decay-based tick counter (HeartBeat.py:205-215) prevents single-frame drops from killing CONFIRMED status.

---

### REGIME: WALK_SWING

**Source:** gait_planner.py:361-398

**FSM State:** MissionState.WALK, StepPhase.SWING

**Physical Description:** Swing foot tracing cycloidal arc. Robot in single-leg support. Phi advances 0→0.85.

**Entry Condition:** LIFT exit (swing unloaded, stance loaded).

**Exit Condition:** phi ≥ 0.85 → PLACE.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.84 m | ±0.05 m | ±0.10 m | <0.55 m or >1.05 m |
| base_pitch | ±3° | ±5° | ±8° | >12° |
| base_roll | ±5° | ±8° | ±12° | >14° |
| n_contacts | 1 | 1 | 2 (premature) | 0 (stance lost) |
| contact_force_z (stance) | 79.5 N | 55–105 N | <35 N | <10 N |
| contact_force_z (swing) | 0 N | 0 N | >3 N | >10 N |
| swing_foot_z | 0→0.06→~0.02 m | 0–0.065 m | <-0.005 m or >0.08 m | <-0.01 m (collision) |
| swing_phase (col 52) | 0.0→0.85 | monotonic increase | decreasing or stalled >0.3 s | phi=0 after entry |
| angular_velocity | ±0.5 rad/s | ±1.0 rad/s | ±2.0 rad/s | >3.0 rad/s |
| wbc_tracking_error | <0.2 rad | <0.4 rad | >0.6 rad | >1.0 rad |

**Derivations:**
- base_z: lower than DS (0.84 vs 0.86). Single-leg compliance, COM sags.
- Swing foot z: cycloidal z = 0.06 × (1 - cos(2π·φ)) / 2. Peak 0.06 m at φ=0.5.
- Phi rate: dt/SWING_DURATION = 0.01/0.50 = 0.02 per cycle. 0→0.85 in ~43 cycles.
- SWING_TIMEOUT = 0.75 s (gait_planner.py:67, 1.5 × 0.50). Force-advances to PLACE + ERR_PHASE_TIMEOUT.
- WBC tracking: wiki (whole-body-controller.md) notes 0.2-0.4 rad typical, 2.5 rad spikes at transitions.

#### Violation Signatures

- `swing_foot_z` < 0: foot dragging — clearance too low or IK workspace exceeded. Check rel_z guard.
- `base_roll` >12°: tipping during single support. Balance at hip roll limit. 29 mm foot = no passive recovery.
- `contact_force_z (stance)` <35 N: stance unloading — GRF spring not compensating.
- `phi` stalled: timing_violation_this_cycle blocks gait planner phase advance.
- `wbc_tracking_error` >0.6 rad: torque saturation. Check sat flags (cols 39-44).

#### Transition Risk

- **Entry from LIFT:** If LIFT exited on timeout, `_snapshot_swing_foot_x` captures wrong initial position.
- **Exit to PLACE:** phi=0.85 is geometric, not force-based. Foot may be above ground if IK clamped.

#### Siclo1-Specific Fragilities

- Short thigh (60.7 mm): hip must swing 60-90° for forward foot placement. Near URDF limit (±1.57 rad). Large angles → large WBC errors → saturation.
- IK reachable annulus: R_MIN=0.631 m, R_MAX=0.743 m. Arc must stay within this or IK raises ValueError → gait planner holds last angles (silent failure).

---

### REGIME: WALK_PLACE

**Source:** gait_planner.py:400-435

**FSM State:** MissionState.WALK, StepPhase.PLACE

**Physical Description:** Swing foot descending toward ground, phi 0.85→1.0. Awaiting contact confirmation.

**Entry Condition:** SWING exit (phi ≥ 0.85).

**Exit Condition:** Swing foot CONTACT_CONFIRMED AND vel_z < 0.05 m/s → `_complete_step()`.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.84 m | ±0.05 m | ±0.10 m | <0.55 m or >1.05 m |
| base_pitch | ±2° | ±5° | ±8° | >12° |
| base_roll | ±4° | ±8° | ±12° | >14° |
| n_contacts | 1→2 | 1 or 2 | 0 | 0 sustained |
| contact_force_z (stance) | 79.5→50 N | 40–100 N | <30 N | <10 N |
| contact_force_z (swing) | 0→30 N | 0–50 N | stuck at 0 (miss) | 0 after 0.5 s (freeze) |
| swing_foot_z | 0.006→0.0 m | 0.0–0.03 m | >0.04 m (not descending) | >0.06 m |
| swing_phase (col 52) | 0.85→1.0 | 0.85–1.0 | <0.85 | clamped at 1.0 |
| step_phase_timer | 0→0.5 s | 0–0.5 s | >0.3 s no contact | >0.5 s (timeout) |

**Derivations:**
- swing_foot_z at phi=0.85: z = 0.06 × (1 - cos(1.7π)) / 2 ≈ 0.006 m. Descending toward 0.
- PLACE_TIMEOUT = 0.5 s (gait_planner.py:68). If swing_force >5 N at timeout → completes step (sensor lag). If swing_force ≤5 N → freeze_robot = True (missed ground).
- SETTLE_VEL_THRESHOLD = 0.05 m/s (gait_planner.py:76).

#### Violation Signatures

- `swing_foot_z` not reaching 0: IK placing foot above ground plane. Check x_target vs workspace.
- `contact_force_z (swing)` stuck at 0 after 0.3 s: foot missed ground. PLACE_TIMEOUT at 0.5 s will freeze.
- Timer >0.5 s: terminal failure path.

#### Transition Risk

- **Entry:** Clean from SWING at phi=0.85.
- **Exit:** `_complete_step()` flips sides, increments step_count, locks new stance foot. Most complex single transition.

#### Siclo1-Specific Fragilities

- Cycloidal profile: z_dot(phi=1.0) = 0. Zero impact velocity eliminates spikes but means foot "floats" onto ground slowly. Contact detection may be delayed.
- 3-tick gate: 30 ms delay between contact and CONFIRMED. Post-contact bounce resets ticks. Margin is comfortable (0.5 s timeout), but bouncing robot may never confirm.

---

### REGIME: DECEL_SWING

**Source:** gait_planner.py:373-376, mission.py:119-123, grf.py:197-199

**FSM State:** MissionState.DECEL, StepPhase ∈ {SWING, PLACE}

**Physical Description:** Final step with halved step length (0.06 m) and 20% boosted GRF spring. Decelerating.

**Entry Condition:** WALK → DECEL when steps_remaining ≤ 1.

**Exit Condition:** steps_remaining = 0 → STOP.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.85 m | ±0.05 m | ±0.10 m | <0.55 m or >1.05 m |
| base_pitch | ±2° | ±4° | ±7° | >12° |
| base_roll | ±4° | ±7° | ±10° | >14° |
| contact_force_z (stance) | 79.5 N | 55–110 N | <35 N | <10 N |
| swing_foot_z | same as WALK_SWING | same | same | same |

**Derivations:**
- Pitch band tighter: shorter step (0.06 m) = less sagittal disturbance.
- Force band wider on high end: DECEL_SPRING_BOOST = 1.2 × K = 1907 N/m. Higher restoring forces on contact. Watch for >100 N spikes.

#### Violation Signatures

- Same as WALK_SWING plus: contact force spikes from boosted spring constant.

#### Transition Risk

- **Entry:** Mid-step transition. `_compute_x_target` halves STEP_LENGTH. Current step adjusts cleanly.
- **Exit to RAMP_DOWN:** Last step landing quality determines STOP stability.

---

### REGIME: DECEL_DS

**Source:** mission.py:121-127, gait_planner.py:270-304

**FSM State:** MissionState.DECEL, StepPhase ∈ {DOUBLE_SUPPORT, COM_SHIFT, LIFT}

**Physical Description:** Final double-support / weight transfer before stopping.

**Entry Condition:** Last DECEL step completes via `_complete_step()`.

**Exit Condition:** steps_remaining = 0 → STOP.

#### Expected Telemetry

Same as WALK_DS, with:
- `steps_remaining` should reach 0.
- GRF spring still boosted (K = 1907 N/m). Contact forces ~10% higher than normal DS.

---

### REGIME: RAMP_DOWN

**Source:** mission.py:129-134

**FSM State:** MissionState.STOP, StepPhase.DOUBLE_SUPPORT

**Physical Description:** Robot standing, torque authority decaying 1→0 over 0.2 s (20 cycles).

**Entry Condition:** DECEL → STOP when steps_remaining = 0.

**Exit Condition:** ramp_gain = 0.0 → IDLE.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | 0.87 m | ±0.03 m | ±0.06 m | <0.60 m or >1.05 m |
| base_pitch | 0° | ±3° | ±6° | >10° |
| base_roll | 0° | ±3° | ±6° | >10° |
| n_contacts | 2 | 2 | 1 | 0 |
| contact_force_z | 39.7 N | ±15 N | ±25 N | <5 N or >80 N |
| ramp_gain (col 55) | 1.0→0.0 | monotonic decrease | increasing or stalled | stuck at 1.0 |
| angular_velocity | ±0.2 rad/s | ±0.5 rad/s | ±1.0 rad/s | >2.0 rad/s |

**Derivations:**
- STOP_RATE = 1/20 per cycle. 1→0 in 20 cycles (0.2 s). 2.5× faster than ramp-up.
- base_z settling: removing torque authority causes legs to settle under passive stiffness.

#### Violation Signatures

- `ramp_gain` not decreasing: mission controller stuck.
- `base_z` dropping rapidly: torque removal causes leg buckle. Below 0.80 m may be unrecoverable.

#### Transition Risk

- **Entry:** Poor last-step landing → STOP starts oscillating. Removing torques while oscillating can amplify.
- **Exit to IDLE:** Integrators from walking may carry over. `reset_balance()` called at next warmup, not at IDLE entry.

---

### REGIME: FROZEN

**Source:** shared_state.py:449, HeartBeat.py:762, gait_planner.py:145

**FSM State:** Any, with freeze_robot=True or emergency_stop_triggered=True.

**Physical Description:** All control output halted. Robot falling passively.

**Entry Condition:** Any of:
- timing_violations > 10 (shared_state.py:509)
- DS_TIMEOUT 2.0 s (gait_planner.py:290-291)
- PLACE timeout with no contact (gait_planner.py:435)
- EMERGENCY_STOP recovery (recovery.py:212-213)

**Exit Condition:** None — requires manual reset.

#### Expected Telemetry

| Signal | Optimal | Acceptable Band | Conf 0.5 Threshold | Conf 0.0 Threshold |
|--------|---------|-----------------|--------------------|--------------------|
| base_z | any | decreasing | <0.50 m | <0.30 m (ground) |
| base_pitch | any | any | >20° | >45° |
| base_roll | any | any | >20° | >45° |
| n_contacts | any | any | any | any |
| angular_velocity | any | any | >3.0 rad/s | >5.0 rad/s |

Confidence thresholds are lenient — FROZEN is already the terminal condition.
The interesting diagnostic is *what caused the freeze*: inspect last error_code and
pre-freeze telemetry rows for the originating regime's violation pattern.

#### Siclo1-Specific Fragilities

- FROZEN is binary and unrecoverable. A transient timing spike (ConvexHull >5 ms) counts toward the 10-violation threshold. Multiple runs can freeze from Shapely/SciPy startup overhead.

---

## 6. Flagged Inconsistencies

### Wiki vs Code Discrepancies

| # | Wiki Claim | Code Reality | Severity |
|---|-----------|-------------|----------|
| 1 | gait-planning.md: "z_swing = 0.04 × 4φ(1−φ)" (parabolic, 40mm) | gait_planner.py:58-104: `SWING_HEIGHT = 0.06` m, cycloidal profile `(1-cos(2πφ))/2` | **Medium** — wiki outdated. Code changed from parabolic to cycloidal and height from 40mm to 60mm. Wiki still describes old profile. |
| 2 | gait-planning.md: "Step length = 0.12 m" | gait_planner.py:55: `STEP_LENGTH = 0.12` m | OK — consistent |
| 3 | active-balance-lipm.md: "FOOT_WIDTH = 0.0286 m" | siclo1-robot.md: "28.6 mm" | OK — consistent, 28.6 mm vs 29 mm is rounding |
| 4 | unified-balance-controller.md: "As of 2026-04-27, this is a plan — no code" | balance_controller.py exists with full implementation | **Low** — wiki not updated after implementation. Code is source of truth. |
| 5 | gait-planning.md: "Hip roll step changes cause oscillation; smooth ramp-in is open issue" | balance_controller.py:63: `HIP_ROLL_RATE_LIMIT = 0.03 rad/cycle` | **Low** — rate limiting was implemented. Wiki tension section outdated. |

### FSM States Without Wiki Description

- **MissionState.RAMP and STOP**: described in code (mission.py) and implicitly in gait-planning.md phase table, but have no dedicated wiki concept page. The ramp-up / ramp-down dynamics (torque authority scaling) are not documented in the wiki.

### Wiki Descriptions Without Code FSM State

- None found. All wiki-described regimes map to existing code states.

### Potential WBC Conflicts

- Balance controller PITCH_OFFSET_MAX = 0.15 rad (8.6°) is added to IK hip_pitch in `_wbc_step()`. If IK already targets a large hip_pitch (near URDF limit ±1.57 rad during SWING), the offset can push the total beyond the URDF limit. `_clip_position` would clamp, but WBC would then fight the clamp with full KP, causing saturation. This is a theoretical conflict — not yet observed in logs.

---

## 7. Open Questions

These require either a simulation run or human judgment to resolve:

### Requires Simulation Run

1. **joint_torque_norm per regime** — All regimes have `[TO CALIBRATE]` for joint torque norm. Procedure: run a full walk cycle (IDLE→RAMP→WALK 10 steps→DECEL→STOP→IDLE), log `np.linalg.norm(list(joint_torques.values()))` each cycle, segment by regime, compute mean ± 2σ per regime.

2. **base_z actual range during SWING** — Derived value (0.84 m) assumes slight COM sag under single-leg load. Actual sag depends on GRF spring constant and WBC tracking quality. Calibrate from logged `com_z` during SWING phases.

3. **WBC tracking error distribution per regime** — Current thresholds based on wiki note ("0.2-0.4 rad typical, 2.5 rad spikes at transitions"). Full distribution needed per regime for proper confidence thresholds. Calibrate from logged `max(|wbc_tracking_error|)` per regime.

4. **Contact force variance during COM_SHIFT** — The stance/swing force ramp during weight transfer is derived from physics (W shifting from 50/50 to 100/0) but the actual rate and variance depend on hip roll dynamics and PyBullet contact model. Calibrate from logged forces.

5. **FROZEN detection from telemetry** — The telemetry buffer does not contain `freeze_robot` or `emergency_stop_triggered` directly. The monitor must infer FROZEN from: cycle count (col 1) stops advancing, or error_code (col 2) contains ERR_TIMING_VIOLATION followed by cessation. Needs validation that this inference is reliable.

### Requires Human Judgment

6. **Should DECEL_DS and DECEL_SWING share profiles with their WALK counterparts?** — Current design separates them because DECEL has shorter step length and boosted GRF spring. But the telemetry profiles are nearly identical. Merging would reduce the regime count from 11 to 9.

7. **Balance controller integrator carry-over at IDLE entry** — `reset_balance()` is called after warmup but not at STOP→IDLE transition. Should the RegimeMonitor flag non-zero integrator state as a DEGRADED condition in IDLE_STANDING?

8. **Confidence function shape** — The piecewise linear ramp with a 0.5 knee is simple and interpretable, but a sigmoid or Gaussian might better model gradual degradation. Worth revisiting after initial calibration reveals actual signal distributions.

9. **Multi-signal correlation** — Current design evaluates each signal independently. Some violations are correlated (e.g., base_z drop + contact_force increase during GRF spring settling is expected, not anomalous). Should the monitor suppress known-correlated violations? This adds complexity but reduces false DEGRADED classifications during transients.

10. **Telemetry column for regime output** — The current 72-column layout has no spare column for regime classification. Options: (a) add cols 72-74 for regime/condition/min_confidence, (b) keep regime output in a separate data structure. Adding columns changes the telemetry schema (CSV header, ring buffer COLS constant).

---

## 8. Summary

### What This Document Defines

- **11 primary regimes** derived from the cross-product of MissionState × StepPhase, with FROZEN as a universal override
- **4-level condition overlay** (NOMINAL / DEGRADED / CRITICAL / FALLEN) computed from per-signal confidence vectors
- **~110 signal-threshold pairs** across all regimes (11 regimes × ~10 signals each), each with derived acceptable band, 0.5 confidence threshold, and 0.0 confidence threshold
- **5 flagged inconsistencies** between wiki documentation and current code
- **10 open questions** requiring sim runs or human judgment before implementation

### What This Document Does Not Define

- Implementation code (this is a discovery/specification pass only)
- Integration with TelemetryThread (deferred to implementation plan)
- Logging format for regime output
- Dashboard or visualization for regime/condition display
