# Siclo1 Telemetry Logging System — Design Spec
**Date:** 2026-04-03  
**Status:** Approved  
**Files touched:** `telemetry.py` (new), `HeartBeat.py` (modified)

---

## 1. Objective

Refactor the Siclo1 control stack to add:

1. **Session-based file logging** — each simulation run generates a timestamped folder containing a live-written `telemetry.csv` and a final `summary.txt`.
2. **Warm-up filter** — the first 50 cycles are excluded from performance statistics (Mean compute, Jitter, Violations) but are still recorded to the CSV for black-box debugging.

---

## 2. Constraints

- **100 Hz hot loop is untouched.** `step()` in `HeartBeat.py` has zero changes. No I/O, no string formatting in the producer path.
- **Error codes only on the hot path.** `add_error_code(int)` remains the only legal call from the loop. String decoding happens in the consumer.
- **CSV integrity.** Preserves the existing 16-column `TelemetryRingBuffer` format. Append-only via the drain process.
- **Clean shutdown.** File handles guarded by `try/finally` in `SessionLogger.write_summary()`. Survives `KeyboardInterrupt`.
- **Graceful filesystem degradation.** If folder/file creation fails, logging degrades to stdout-only — simulation does not crash.

---

## 3. Architecture

```
HeartBeat.py                        telemetry.py
─────────────────────               ──────────────────────────────────────
Siclo1Controller                    TelemetryThread  (extracted from HB)
  └─ HeartbeatController              ├─ _session: SessionLogger
  └─ PyBulletInterface                │    ├─ creates timestamped folder
  └─ TelemetryThread  ──────────►     │    ├─ opens telemetry.csv (buffered)
                                      │    └─ writes summary.txt on close()
                                      ├─ online accumulators (post-warmup)
                                      └─ drain loop at ~10 Hz
```

### Module ownership after refactor

| File | Owns |
|---|---|
| `telemetry.py` | `SessionLogger`, `TelemetryThread`, `WARMUP_CYCLES` constant |
| `HeartBeat.py` | `HeartbeatController`, `PyBulletInterface`, `Siclo1Controller` |
| `shared_state.py` | Unchanged — `TelemetryRingBuffer`, `Siclo1State`, error codes |

---

## 4. `telemetry.py` — Detailed Design

### 4.1 Constants

```python
WARMUP_CYCLES: int = 50  # cycles — excluded from performance statistics
CSV_HEADER: str = (
    "timestamp_s,cycle,error_code,com_x,com_y,com_z,"
    "left_contact,right_contact,stability_status,"
    "left_force_n,right_force_n,stability_margin_m,"
    "compute_us,extra_0,extra_1,extra_2"
)
```

### 4.2 `SessionLogger`

**Created in:** `TelemetryThread.__init__` — folder exists before the first drain cycle.

**Session folder path:** `<script_dir>/sessions/YYYY-MM-DD_HH-MM-SS/`

**Public interface:**

| Method | Description |
|---|---|
| `__init__()` | Creates folder, opens `telemetry.csv` with `buffering=8192`, writes header |
| `append_row(row: np.ndarray)` | Formats 16 columns, writes one CSV line |
| `write_summary(stats: dict)` | Writes `summary.txt`; `finally` block flushes and closes all handles |
| `.session_path: str` | Read-only property — path to the session folder |

**File open mode:** `open(csv_path, 'w', buffering=8192)` — block-buffered to avoid WSL2 per-write latency.

**Failure mode:** If `OSError` is raised during `__init__`, `_csv` is set to `None`. All subsequent `append_row` calls guard with `if self._csv is None: return`. Simulation continues with stdout-only logging.

### 4.3 `TelemetryThread`

Extracted verbatim from `HeartBeat.py` with the following additions:

**New instance scalars (zero allocation):**

```python
_total_cycles:    int    # every row seen, including warmup
_analyzed_cycles: int    # rows where row[1] > WARMUP_CYCLES
_sum_compute:     float  # post-warmup compute_us sum
_sum_sq_compute:  float  # for std dev
_max_compute:     float
_min_compute:     float  # initialised to float('inf')
_violations:      int    # post-warmup ERR_TIMING_VIOLATION count
```

**Drain loop logic per row:**

```
1. _session.append_row(row)          # always — warmup rows go to CSV
2. _total_cycles += 1
3. if row[1] > WARMUP_CYCLES:        # col 1 = cycle number
       update accumulators
       if row[2] == ERR_TIMING_VIOLATION: _violations += 1
```

**New public methods:**

| Method | Description |
|---|---|
| `get_summary_stats() → dict` | Returns same shape as `HeartbeatController.get_statistics()` — used by `_print_final_summary` in HeartBeat. Returns a zero-filled dict if `_analyzed_cycles == 0` (run ended before warmup completed). |

The final drain and `_session.write_summary(stats)` call happen at the **tail of `run()`** after the `while not _stop_event.is_set()` loop exits — not in a separate method. `stop()` remains the signal method (unchanged from today).

**`get_summary_stats()` dict keys:**

```
mean_dt, std_dt, min_dt, max_dt, jitter_ms,
violations, violation_rate,
total_cycles, analyzed_cycles, warmup_cycles
```
All timing values are in **seconds** (consistent with `HeartbeatController`). `compute_us` from the ring buffer is divided by 1e6 before accumulation.

### 4.4 Shutdown sequence

```
Siclo1Controller.shutdown()
  1. _telemetry_thread.stop()          # sets _stop_event
  2. _telemetry_thread.join(timeout=2.0)
       └─ TelemetryThread.run() exits while loop
       └─ [tail of run()] final read_batch() drain
       └─ [tail of run()] _session.write_summary(stats)  # closes CSV + writes summary.txt
  3. p.disconnect()
  4. _telemetry_thread.flush_to_stdout()  # in-memory log lines (init messages)
```

File writes complete before `p.disconnect()`. `flush_to_stdout()` is retained for init/shutdown messages that never enter the ring buffer.

---

## 5. `HeartBeat.py` — Changes

### 5.1 Import swap

Remove the `TelemetryThread` class definition (~50 lines). Add:

```python
from telemetry import TelemetryThread
```

`TelemetryThread(self.shared_state)` constructor call in `Siclo1Controller.__init__` is **unchanged**.

### 5.2 `_print_final_summary` update

Replace:
```python
ts = self.heartbeat.get_statistics()
```
With:
```python
ts = self._telemetry_thread.get_summary_stats()
```

The dict shape is identical — print formatting lines are unchanged.

### 5.3 Shutdown sequence update

Replace `join(timeout=1.0)` with `join(timeout=2.0)` to allow the final drain and file write to complete.

### 5.4 What does NOT change

- `step()` — zero changes
- `HeartbeatController` — zero changes  
- `PyBulletInterface` — zero changes
- `Siclo1Controller.__init__` — one-line import swap only

---

## 6. Session Folder & File Format

### Folder layout

```
sessions/
└── 2026-04-03_14-32-07/
    ├── telemetry.csv     ← live-written, one row per cycle (including warmup)
    └── summary.txt       ← written once on clean shutdown
```

### `telemetry.csv` columns (16, preserves ring buffer format)

```
timestamp_s, cycle, error_code, com_x, com_y, com_z,
left_contact, right_contact, stability_status,
left_force_n, right_force_n, stability_margin_m,
compute_us, extra_0, extra_1, extra_2
```

### `summary.txt` content

```
Session: 2026-04-03_14-32-07
Total cycles   : 1000
Analyzed cycles: 950  (warmup excluded: 50)

[TIMING]
  Mean compute : X.XXX ms
  Std dev      : X.XXX ms
  Min          : X.XXX ms
  Max          : X.XXX ms
  Jitter       : X.XXX ms
  Violations   : N  (X.X%)
  Status       : PASS / FAIL

[ERRORS]
  Coded errors in ring: N
```

PASS criteria (consistent with existing `_print_final_summary`): mean < 10.5 ms AND jitter < 1.0 ms.

---

## 7. Error Handling

| Scenario | Behaviour |
|---|---|
| `OSError` on folder/file creation | `_csv = None`; logging degrades to stdout; simulation continues |
| `KeyboardInterrupt` | `finally` in `main()` calls `shutdown()`; `write_summary` `finally` closes handles |
| `TelemetryThread.join` timeout (2 s) | File may be incomplete; CSV rows already written are valid; summary.txt may be absent |
| Ring buffer overrun (producer faster than consumer) | Existing overwrite behaviour preserved; no change |
| `append_row` called with `_csv is None` | Silently returns; no exception propagated to drain loop |

No bare `except` on any file or physics logic path.

---

## 8. Data Flow Diagram

```
100 Hz producer              TelemetryThread (~10 Hz drain)       Disk
─────────────────            ──────────────────────────────       ────
step() hot loop
  write(row) → ring  →→→→   read_batch()
                             for row in batch:
                               session.append_row(row)    →→→→   telemetry.csv
                               _total_cycles += 1
                               if row[1] > WARMUP_CYCLES:
                                 update accumulators
                             _stop_event.wait(0.1)

shutdown signal →→→→→→→→→   exit loop
                             final read_batch()
                             session.write_summary(stats)  →→→→   summary.txt
                             [handles closed in finally]
```

---

## 9. Out of Scope

- `shared_state.py` — no changes
- `sim/interface.py` abstraction layer (future Gazebo swap) — not affected
- ROS2 / Gazebo migration — not affected
- GUI sync path (`_sync_gui`) — not affected
- Any module other than `HeartBeat.py` and `telemetry.py` (new)
