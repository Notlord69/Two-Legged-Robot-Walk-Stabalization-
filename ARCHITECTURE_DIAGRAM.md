# Siclo1 — Architecture Diagram

```
╔══════════════════════════════════════════════════════════════════════════════╗
║              PROJECT SICLO1 — 100 Hz BIPEDAL ROBOT CONTROLLER               ║
║                   8 kg | PyBullet | Siclo1_Primitive.urdf                   ║
╚══════════════════════════════════════════════════════════════════════════════╝


════════════════════════════════════════════════════════════════════════════════
  ENTRY POINT
════════════════════════════════════════════════════════════════════════════════

  main.py  ──────────────────────────────────────────────────────────────────
  CLI entry: argparse --gui --viz-hz --duration --hold --walk --on
  Constructs Siclo1Controller (HeartBeat.py), calls .run(max_cycles=N)


════════════════════════════════════════════════════════════════════════════════
  12-STAGE CONTROL PIPELINE  (HeartBeat.py, one iteration = 10 ms)
════════════════════════════════════════════════════════════════════════════════

  ┌─────────────────────────────────────────────────────────────────────────┐
  │  STAGE 1  sensors          sim/interface.py                             │
  │                            getJointState, getLinkState, getContactPoints│
  │                            → shared_state.{joint_positions, base_pose,  │
  │                               foot_position/velocity/force/flat/ticks}  │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 2  link_positions   HeartBeat.py                                 │
  │                            getLinkState for every link                  │
  │                            → shared_state.link_positions                │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 3  perception       perception.py                                │
  │                            4-state contact FSM (per foot)               │
  │                            slip detection (force drop + lateral vel)    │
  │                            → shared_state.{*_contact_state, *_slip}     │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 4  stability        stability.py                                 │
  │                            2D FK fallback, Shapely support polygon      │
  │                            LIPM capture point                           │
  │                            → shared_state.{stability_status,            │
  │                               stability_margin, com_position,           │
  │                               capture_point}                            │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 5  balance          balance_controller.py                        │
  │                            Lateral roll PID   (LATERAL_ROLL_GAIN=0.8)  │
  │                            Sagittal pitch PID (SAGITTAL_PITCH_GAIN=1.2) │
  │                            Emergency torque injection  (threshold 8 cm) │
  │                            → shared_state.balance_torque_correction     │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 6  grf              grf.py                                       │
  │                            Spring-damper Fz (K=1589 N/m, B=94 N·s/m)  │
  │                            Sagittal 2-link Jacobian torques             │
  │                            Suppressed in IDLE; suppressed on swing leg  │
  │                            → shared_state.grf_torque_correction         │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 7  gait_planner     gait_planner.py                              │
  │                            5-phase step FSM per leg                     │
  │                            Idle-stance ramp (50 cycles)                 │
  │                            → shared_state.{wbc_targets, step_phase,     │
  │                               stance_side, target_foot_position}        │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 8  mission          mission.py                                   │
  │                            5-state mission FSM                          │
  │                            ramp_gain [0→1] over 50 cycles (RAMP)        │
  │                            ramp_gain [1→0] over 20 cycles (DECEL)       │
  │                            → shared_state.{mission_state, ramp_gain}    │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 9  wbc              HeartBeat.py                                 │
  │                            Joint-space PD: KP=30 N·m/rad KD=10 N·m·s/r │
  │                            τ = KP*(q_target−q) − KD*q_dot               │
  │                            → shared_state.wbc_torques                   │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 10 recovery         recovery.py                                  │
  │                            5-priority watchdog (see FSM below)          │
  │                            → shared_state.{recovery_action,             │
  │                               freeze_robot, emergency_stop_triggered}   │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 11 apply_control    HeartBeat.py                                 │
  │                            Σ(wbc + balance + grf) torques               │
  │                            → sim/interface.set_joint_position_target()  │
  ├─────────────────────────────────────────────────────────────────────────┤
  │  STAGE 12 step_sim         HeartBeat.py                                 │
  │                            sim/interface.step_simulation()              │
  └─────────────────────────────────────────────────────────────────────────┘

  Timing budget: 10.0 ms/cycle.  Violation → timing_violations++
  Critical violations → freeze_robot = True (Safe Freeze)


════════════════════════════════════════════════════════════════════════════════
  SHARED STATE BUS  (shared_state.py)
════════════════════════════════════════════════════════════════════════════════

  ┌──────────────────────────────────────────────────────────────────────────┐
  │                       shared_state  (Siclo1State singleton)              │
  │                                                                          │
  │  ┌────────────┬──────────────┬──────────────┬────────────┬────────────┐ │
  │  │  TIMING    │  PHYSICS     │  CONTACT     │  CONTROL   │  SAFETY    │ │
  │  │            │              │              │            │            │ │
  │  │ sim_time   │ com_position │ *_contact_   │ wbc_       │ freeze_    │ │
  │  │ cycle_cnt  │ base_pose    │   state      │  targets   │  robot     │ │
  │  │ timing_    │ joint_pos/   │ *_foot_      │ grf_torque │ emergency_ │ │
  │  │  violations│  vel         │   pos/vel/   │ balance_   │  stop      │ │
  │  │            │ link_pos     │   force/flat │  torque    │ recovery_  │ │
  │  │            │              │ *_ticks      │ ramp_gain  │  action    │ │
  │  └────────────┴──────────────┴──────────────┴────────────┴────────────┘ │
  │                                                                          │
  │  + TelemetryRingBuffer (72 cols, numpy, zero-allocation)                 │
  │  + URDF_JOINT_NAMES, URDF_JOINT_LIMITS, DEFAULT_LINK_DATA               │
  └──────────────────────────────────────────────────────────────────────────┘

         ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲  ▲
         │  all modules read/write only shared_state  │
         │  no direct module-to-module calls          │


════════════════════════════════════════════════════════════════════════════════
  MODULE DEPENDENCY GRAPH
════════════════════════════════════════════════════════════════════════════════

                    ┌────────────────────────┐
                    │     shared_state.py    │  ← no external dependencies
                    └──┬──┬──┬──┬──┬──┬──┬──┘
                       │  │  │  │  │  │  │
          ┌────────────┘  │  │  │  │  │  └──────────────┐
          │     ┌─────────┘  │  │  │  └────────┐        │
          │     │     ┌──────┘  │  └────┐      │        │
          ▼     ▼     ▼         ▼       ▼       ▼        ▼
      percep  stab  balance   grf   gait_  mission  recovery
      tion    ility  ctrl          planner
          │     │     │         │       │       │        │
          └─────┴─────┴─────────┴───────┴───────┴────────┘
                                 │
                                 ▼
                            HeartBeat.py          ← imports all of the above
                                 │
                            sim/interface.py       ← all p.* calls
                                 │
                            PyBullet physics

          sim/viz_bridge.py  ──► viz/gui_worker.py (subprocess)
          telemetry.py       ──► TelemetryThread (daemon thread)
          recorder.py        ──► VideoRecorder (daemon thread)
          kinematics.py      ──  pure math, no shared_state writes

  ✓ No circular dependencies
  ✓ Each control module imports only shared_state
  ✓ HeartBeat.py orchestrates; nothing imports HeartBeat


════════════════════════════════════════════════════════════════════════════════
  STATE MACHINE DIAGRAMS
════════════════════════════════════════════════════════════════════════════════

  CONTACT FSM  (perception.py, one instance per foot)

      ┌───────────────┐
      │  NO_CONTACT   │◄────────────────────────────────────────┐
      └───────┬───────┘                                         │
              │ z < height_threshold                            │ airborne
              ▼                                                  │
      ┌───────────────┐                                         │
      │TOUCH_EXPECTED │──── force > min_thresh ──────────────►  │
      └───────┬───────┘                                         │
              │ force > min_thresh                              │
              ▼                                                  │
      ┌───────────────┐                                         │
      │   CONTACT     │──── force drop ──────────────────────── ┘
      │   TENTATIVE   │
      └───────┬───────┘
              │ ticks >= 3 AND foot_flat == True
              ▼
      ┌───────────────┐
      │   CONTACT     │──── force < release_thresh ──────────── ┐
      │   CONFIRMED   │                                         │
      └───────────────┘                                         │
                                      ┌──────────────────────── ┘
                                      ▼
                               (back to TOUCH_EXPECTED
                                or NO_CONTACT based on z)

  ────────────────────────────────────────────────────────────────

  GAIT FSM  (gait_planner.py, per-step cycle)

      IDLE_STANCE (ramp over 50 cycles)
              │ walk command received
              ▼
      DOUBLE_SUPPORT ──► COM_SHIFT ──► LIFT ──► SWING ──► PLACE
              ▲                                                │
              └────────────────────────────────────────────────┘
                          (alternating swing side)

  ────────────────────────────────────────────────────────────────

  MISSION FSM  (mission.py)

      IDLE ──(walk D requested)──► RAMP ──(ramp_gain==1)──► WALK
                                                                │
             STOP ◄──(decel done)── DECEL ◄──(dist reached)────┘

      ramp_gain: 0→1 over 50 cycles on RAMP entry
                 1→0 over 20 cycles on DECEL entry

  ────────────────────────────────────────────────────────────────

  RECOVERY DECISION TREE  (recovery.py, evaluated every cycle)

      is_unstable AND step_timeout exceeded?
          │ YES ──► EMERGENCY_STOP
          │ NO
          ▼
      Previously-CONFIRMED contact now lost?
          │ YES ──► ABORT_HOLD
          │ NO
          ▼
      Both feet unconfirmed AND NOT IDLE AND timed out?
          │ YES ──► REPOSITION (or EMERGENCY_STOP if max_attempts)
          │ NO
          ▼
      Any foot slipping?
          │ YES ──► REPOSITION
          │ NO
          ▼
      MARGINAL stability AND marginal_timeout exceeded?
          │ YES ──► ABORT_HOLD
          │ NO
          ▼
          NONE  (no recovery needed)

  Note: step timer resets every cycle while mission_state == IDLE,
  so priorities 1, 3, and 5 never fire while the robot stands still.


════════════════════════════════════════════════════════════════════════════════
  SIDE SYSTEMS
════════════════════════════════════════════════════════════════════════════════

  TELEMETRY PIPELINE

      HeartBeat.py writes 72-column row into TelemetryRingBuffer each cycle
                  │
                  ▼  (zero allocation — numpy ring buffer)
      TelemetryThread (10 Hz drain) ──► sessions/<ts>/telemetry.csv
                                    ──► sessions/<ts>/regime.csv
                                    ──► sessions/<ts>/summary.txt
                  │
                  ▼
      analyze.py (post-run) ──► com_trajectory.png
                             ──► contact_forces.png
                             ──► timing.png
                             ──► stability.png

  REGIME MONITOR

      RegimeMonitor.classify(shared_state)
          │
          ├── scores 11 PrimaryRegimes (STARTUP … FALLEN)
          ├── per-signal confidence weighting
          └── returns (PrimaryRegime, Condition)

      Condition: NOMINAL / DEGRADED / CRITICAL / FALLEN

  VIZ BRIDGE

      HeartBeat.py
          │  push_pose(base_pos, base_orn, joint_positions)
          ▼
      SharedMemory float64 buffer  [seq | pos(3) | orn(4) | joints(N)]
          │  (lock-free write barrier: seq incremented last)
          ▼
      viz/gui_worker.py subprocess  ──► p.GUI PyBullet window @ viz_fps Hz

  VIDEO RECORDER

      recorder.VideoRecorder (daemon thread)
          │  sim/interface.capture_frame() at 15 Hz
          │  TinyRenderer — no GPU, works in DIRECT mode
          ▼
      sessions/<ts>/walk.mp4  (OpenCV mp4v)


════════════════════════════════════════════════════════════════════════════════
  TIMING BUDGET  (100 Hz = 10.0 ms per cycle)
════════════════════════════════════════════════════════════════════════════════

  ┌─────────────────────────────────────────────────────────────────────────┐
  │  0.0 ms ─────────────────────────────────── START CYCLE                │
  │  0.5 ms ───────────┤ Sensor reads (stages 1–2)                         │
  │  0.7 ms ───────────────┤ Perception (stage 3)                          │
  │  1.1 ms ──────────────────────┤ Stability (stage 4)                    │
  │  1.4 ms ─────────────────────────────┤ Balance + GRF (stages 5–6)      │
  │  1.6 ms ───────────────────────────────────┤ Gait + Mission (7–8)      │
  │  1.9 ms ──────────────────────────────────────────┤ WBC (stage 9)      │
  │  2.2 ms ─────────────────────────────────────────────┤ Recovery (10)   │
  │  2.5 ms ────────────────────────────────────────────────┤ Apply (11)   │
  │  7.5 ms ─────────────────────────────────────────────────────┤ p.step  │
  │ 10.0 ms ──────────────────────────────────────────────────────── END   │
  └─────────────────────────────────────────────────────────────────────────┘

  p.stepSimulation() dominates (~5 ms). Overhead and telemetry ~1 ms.
  Violations logged in shared_state.timing_violations and telemetry.csv.
```
