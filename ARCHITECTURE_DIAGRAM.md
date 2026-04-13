# Project Siclo1 - Visual Architecture Guide

## System Architecture Diagram

```
╔═══════════════════════════════════════════════════════════════════════════╗
║                    PROJECT SICLO1 - MODULAR ARCHITECTURE                  ║
║                         200Hz Bipedal Robot Controller                    ║
╚═══════════════════════════════════════════════════════════════════════════╝


┌───────────────────────────────────────────────────────────────────────────┐
│                          SHARED_STATE.PY                                  │
│                       (Single Source of Truth)                            │
│                                                                           │
│  ┌─────────────┬──────────────┬───────────────┬──────────────┐           │
│  │   TIMING    │   PHYSICS    │    CONTACT    │  STABILITY   │           │
│  │             │              │               │              │           │
│  │ • sim_time  │ • positions  │ • L_state     │ • status     │           │
│  │ • cycle#    │ • velocities │ • R_state     │ • COM        │           │
│  │ • dt        │ • forces     │ • slip_flags  │ • margin     │           │
│  │ • violations│ • load_mass  │               │              │           │
│  └─────────────┴──────────────┴───────────────┴──────────────┘           │
│                                                                           │
│  Thread-safe setters • Convenience methods • Diagnostics                 │
└───────────────────────────────────────────────────────────────────────────┘
         ▲                ▲                ▲                ▲
         │                │                │                │
         │ read/write     │ read/write     │ read/write     │ read/write
         │                │                │                │
    ┌────┴──────┐    ┌───┴────────┐   ┌───┴────────┐   ┌──┴────────┐
    │           │    │            │   │            │   │           │
    │  MAIN.PY  │    │PERCEPTION  │   │ STABILITY  │   │ RECOVERY  │
    │           │    │   .PY      │   │    .PY     │   │   .PY     │
    │           │    │            │   │            │   │           │
    │ ┌───────┐ │    │ ┌────────┐ │   │ ┌────────┐ │   │ ┌───────┐ │
    │ │200Hz  │ │    │ │Contact │ │   │ │  COM   │ │   │ │Monitor│ │
    │ │Heart  │ │    │ │  FSM   │ │   │ │Tracker │ │   │ │ &     │ │
    │ │beat   │ │    │ │ (L+R)  │ │   │ │+ Load  │ │   │ │Trigger│ │
    │ └───────┘ │    │ └────────┘ │   │ └────────┘ │   │ └───────┘ │
    │           │    │            │   │            │   │           │
    │ ┌───────┐ │    │ ┌────────┐ │   │ ┌────────┐ │   │ ┌───────┐ │
    │ │PyBullet│ │    │ │  Slip  │ │   │ │Support │ │   │ │Failure│ │
    │ │Sensor │ │    │ │Detect  │ │   │ │Polygon │ │   │ │Detect │ │
    │ │Read   │ │    │ │        │ │   │ └────────┘ │   │ └───────┘ │
    │ └───────┘ │    │ └────────┘ │   │            │   │           │
    └───────────┘    └────────────┘   └────────────┘   └───────────┘
         │
         │  Control Loop Order (5ms cycle):
         │
         ├─► 1. Read Sensors (→ shared_state)
         ├─► 2. perception.update()      ┐
         ├─► 3. stability.update()       │ Modules in sequence
         ├─► 4. recovery.update()        ┘
         ├─► 5. Apply Control
         ├─► 6. Step Simulation
         └─► 7. Check Timing


════════════════════════════════════════════════════════════════════════════


DATA FLOW DIAGRAM:

Time: t=0ms
┌──────────────┐
│  PyBullet    │  Read physical state
│  Simulation  │  • Foot positions, velocities, forces
└──────┬───────┘  • Base position, orientation
       │
       ▼ Write
┌────────────────────────────────────────────┐
│         shared_state                       │
│  left_foot_position = [x, y, z]            │
│  left_foot_velocity = [vx, vy, vz]         │
│  left_foot_force = F                       │
│  (same for right foot)                     │
└────────────────────────────────────────────┘


Time: t=1ms
       │ Read
       ▼
┌──────────────┐
│ PERCEPTION   │  Process sensor data
│  MODULE      │  • Run contact FSM
└──────┬───────┘  • Detect slip
       │
       ▼ Write
┌────────────────────────────────────────────┐
│         shared_state                       │
│  left_foot_contact_state = CONFIRMED       │
│  right_foot_contact_state = TENTATIVE      │
│  left_foot_slip_detected = False           │
│  right_foot_slip_detected = False          │
└────────────────────────────────────────────┘


Time: t=2ms
       │ Read (contact points)
       ▼
┌──────────────┐
│  STABILITY   │  Compute physics
│   MODULE     │  • Calculate COM (with load)
└──────┬───────┘  • Check polygon containment
       │
       ▼ Write
┌────────────────────────────────────────────┐
│         shared_state                       │
│  com_position = [x, y, z]                  │
│  stability_status = STABLE                 │
│  stability_margin = 0.025m                 │
│  current_safety_margin = 0.018m (with load)│
└────────────────────────────────────────────┘


Time: t=3ms
       │ Read (stability + contacts)
       ▼
┌──────────────┐
│  RECOVERY    │  Monitor state
│   MODULE     │  • Check for failures
└──────┬───────┘  • Decide action
       │
       ▼ Write
┌────────────────────────────────────────────┐
│         shared_state                       │
│  recovery_action = NONE                    │
│  recovery_active = False                   │
│  recovery_reason = ""                      │
└────────────────────────────────────────────┘


Time: t=4ms
       │ Read (recovery action)
       ▼
┌──────────────┐
│ MAIN CONTROL │  Execute control
│              │  • Apply joint torques
└──────┬───────┘  • Step simulation
       │
       ▼
   (cycle repeats)


════════════════════════════════════════════════════════════════════════════


DEPENDENCY GRAPH:

                    ┌──────────────┐
                    │shared_state.py│  (No dependencies)
                    └───────┬──────┘
                            │
              ┌─────────────┼─────────────┬──────────────┐
              │             │             │              │
              ▼             ▼             ▼              ▼
       ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
       │perception│  │stability │  │ recovery │  │  main.py │
       │   .py    │  │   .py    │  │   .py    │  │          │
       └──────────┘  └──────────┘  └──────────┘  └────┬─────┘
                                                        │
                                                        │ imports all
                                                        ▼
                                             ┌─────────────────────┐
                                             │ Orchestrates        │
                                             │ all modules         │
                                             └─────────────────────┘

✓ NO CIRCULAR DEPENDENCIES
✓ Each module only depends on shared_state
✓ main.py coordinates everything


════════════════════════════════════════════════════════════════════════════


STATE MACHINE DIAGRAMS:

CONTACT FSM (in perception.py):

    ┌─────────────┐
    │ NO_CONTACT  │◄──────────────────┐
    └──────┬──────┘                   │
           │                          │
           │ height < threshold       │
           ▼                          │
    ┌──────────────┐                 │
    │    TOUCH     │                 │
    │  EXPECTED    │                 │ release + airborne
    └──────┬───────┘                 │
           │                         │
           │ force detected          │
           ▼                         │
    ┌──────────────┐                │
    │  CONTACT     │                │
    │  TENTATIVE   │────────────────┤ force drop
    └──────┬───────┘                │
           │                        │
           │ settled + 25ms         │
           ▼                        │
    ┌──────────────┐                │
    │  CONTACT     │────────────────┘
    │  CONFIRMED   │
    └──────────────┘


RECOVERY DECISION TREE (in recovery.py):

    Is unstable AND timeout > 3s?
           │
           ├─YES──► EMERGENCY_STOP
           │
           └─NO
              │
              Contact lost unexpectedly?
                     │
                     ├─YES──► ABORT_HOLD
                     │
                     └─NO
                        │
                        Timeout without contact?
                               │
                               ├─YES──► REPOSITION
                               │
                               └─NO
                                  │
                                  Slip detected?
                                         │
                                         ├─YES──► REPOSITION
                                         │
                                         └─NO──► NONE (all good)


════════════════════════════════════════════════════════════════════════════


TIMING BUDGET (200Hz = 5ms per cycle):

┌─────────────────────────────────────────────────────┐
│ 0.0ms ├─┤ Start Heartbeat                           │
│ 0.5ms ├────────┤ Read Sensors (PyBullet)            │
│ 1.0ms ├──────────────┤ Perception.update()          │
│ 1.5ms ├──────────────────────┤ Stability.update()   │
│ 2.0ms ├────────────────────────────┤ Recovery.update()│
│ 2.5ms ├──────────────────────────────────┤ Control   │
│ 3.5ms ├────────────────────────────────────────┤ Sim │
│ 4.5ms ├──────────────────────────────────────────────┤│
│ 5.0ms │ End Cycle (sleep if ahead, warn if behind)  │
└─────────────────────────────────────────────────────┘

Headroom: ~0.5ms for overhead


════════════════════════════════════════════════════════════════════════════


KEY FEATURES BY MODULE:

┌──────────────────────────────────────────────────────────────────┐
│ PERCEPTION (perception.py)                                       │
├──────────────────────────────────────────────────────────────────┤
│ ✓ 4-state FSM per foot                                           │
│ ✓ Temporal filtering (25ms settling)                             │
│ ✓ Hysteresis (5N engage, 3N release)                             │
│ ✓ Slip detection (force drop + lateral motion)                   │
│ ✓ Emergency detection (unsafe velocity/penetration)              │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│ STABILITY (stability.py)                                         │
├──────────────────────────────────────────────────────────────────┤
│ ✓ COM calculation from links                                     │
│ ✓ Week 1: Dynamic load offset (0-5kg)                            │
│ ✓ Week 1: Safety margin scaling (load-dependent)                 │
│ ✓ Support polygon from confirmed contacts                        │
│ ✓ Point-in-polygon stability check                               │
│ ✓ STABLE/MARGINAL/UNSTABLE classification                        │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│ RECOVERY (recovery.py)                                           │
├──────────────────────────────────────────────────────────────────┤
│ ✓ Monitors stability.py outputs                                  │
│ ✓ Monitors perception.py outputs                                 │
│ ✓ Timeout detection                                              │
│ ✓ Contact loss detection                                         │
│ ✓ Slip response                                                  │
│ ✓ Priority-based action selection                                │
│ ✓ Event logging                                                  │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│ MAIN (main.py)                                                   │
├──────────────────────────────────────────────────────────────────┤
│ ✓ 200Hz deterministic heartbeat                                  │
│ ✓ Microsecond-precision timing                                   │
│ ✓ Timing violation detection                                     │
│ ✓ PyBullet sensor reading                                        │
│ ✓ Module orchestration                                           │
│ ✓ System freeze on critical violations                           │
└──────────────────────────────────────────────────────────────────┘


════════════════════════════════════════════════════════════════════════════


EXAMPLE: Complete Cycle with Data

t=0.000s  START CYCLE
          │
          ├─ shared_state.cycle_count = 0
          ├─ shared_state.sim_time = 0.000
          │
t=0.001s  READ SENSORS
          │
          ├─ shared_state.left_foot_position = [-0.1, 0.0, 0.0]
          ├─ shared_state.left_foot_force = 45.3 N
          ├─ shared_state.right_foot_position = [0.1, 0.0, 0.0]
          ├─ shared_state.right_foot_force = 48.7 N
          │
t=0.002s  PERCEPTION.UPDATE()
          │
          ├─ Read: left_foot_force = 45.3 N
          ├─ Process: FSM detects stable force
          ├─ Write: left_foot_contact_state = CONFIRMED
          │
          ├─ Read: right_foot_force = 48.7 N
          ├─ Process: FSM detects stable force
          └─ Write: right_foot_contact_state = CONFIRMED
          │
t=0.003s  STABILITY.UPDATE()
          │
          ├─ Read: confirmed_contact_points = [left_pos, right_pos]
          ├─ Read: current_load_mass = 2.5 kg
          ├─ Compute: COM with 2.5kg load = [0.0, 0.0, 1.02]
          ├─ Compute: Support polygon from 2 points
          ├─ Check: COM inside polygon ✓
          ├─ Write: stability_status = STABLE
          ├─ Write: stability_margin = 0.023m
          └─ Write: current_safety_margin = 0.019m (scaled)
          │
t=0.004s  RECOVERY.UPDATE()
          │
          ├─ Read: stability_status = STABLE
          ├─ Read: left_foot_contact_state = CONFIRMED
          ├─ Read: right_foot_contact_state = CONFIRMED
          ├─ Read: step_duration = 0.5s
          ├─ Evaluate: All conditions good
          ├─ Write: recovery_action = NONE
          └─ Write: recovery_active = False
          │
t=0.005s  END CYCLE
          │
          ├─ Computation time: 4.2ms
          ├─ Sleep: 0.8ms
          └─ Next cycle: t=0.005s

```

**This diagram shows how data flows through the system in real-time!**
