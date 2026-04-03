# Telemetry Logging System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract `TelemetryThread` from `HeartBeat.py` into a new `telemetry.py` module, adding `SessionLogger` for per-run file logging and a 50-cycle warm-up filter on performance statistics.

**Architecture:** `SessionLogger` owns the timestamped `sessions/YYYY-MM-DD_HH-MM-SS/` folder, a block-buffered `telemetry.csv`, and `summary.txt` written on shutdown. `TelemetryThread` owns the 10 Hz drain loop, calls `session.append_row()` for every ring buffer row, and maintains post-warmup online accumulators. The final drain and `write_summary()` happen at the tail of `run()` after `_stop_event` is set. `HeartBeat.py`'s `step()` is completely untouched — the only changes are an import swap, one stats-call update, and a `join` timeout extension.

**Tech Stack:** Python 3.10+, pytest, numpy, threading, os, datetime

---

## File Map

| Action | File | Responsibility |
|---|---|---|
| Create | `telemetry.py` | `WARMUP_CYCLES`, `CSV_HEADER`, `SessionLogger`, `TelemetryThread` |
| Create | `test_telemetry.py` | All tests for `telemetry.py` |
| Modify | `HeartBeat.py` | Import swap, `get_summary_stats()` call, `join(timeout=2.0)` |

`shared_state.py` — **no changes**. `HeartBeat.py:step()` — **no changes**.

---

### Task 1: `SessionLogger` — folder creation, CSV header, `append_row`

**Files:**
- Create: `telemetry.py`
- Create: `test_telemetry.py`

- [ ] **Step 1: Create `test_telemetry.py` with four failing tests**

```python
"""Tests for telemetry.py — SessionLogger and TelemetryThread."""
import os
import numpy as np
import pytest
from telemetry import SessionLogger, CSV_HEADER


def test_session_logger_creates_folder(tmp_path):
    logger = SessionLogger(str(tmp_path))
    assert os.path.isdir(logger.session_path)
    assert 'sessions' in logger.session_path


def test_session_logger_writes_csv_header(tmp_path):
    logger = SessionLogger(str(tmp_path))
    logger._csv.flush()
    csv_path = os.path.join(logger.session_path, "telemetry.csv")
    with open(csv_path) as f:
        first_line = f.readline().strip()
    assert first_line == CSV_HEADER


def test_session_logger_append_row_writes_data(tmp_path):
    logger = SessionLogger(str(tmp_path))
    row = np.zeros(16, dtype=np.float64)
    row[0] = 1.23     # timestamp_s
    row[1] = 7.0      # cycle
    row[12] = 8500.0  # compute_us
    logger.append_row(row)
    logger._csv.flush()
    csv_path = os.path.join(logger.session_path, "telemetry.csv")
    with open(csv_path) as f:
        lines = f.readlines()
    assert len(lines) == 2  # header + 1 data row
    assert '1.23' in lines[1]
    assert '7' in lines[1]
    assert '8500' in lines[1]


def test_session_logger_session_path_is_string(tmp_path):
    logger = SessionLogger(str(tmp_path))
    assert isinstance(logger.session_path, str)
```

- [ ] **Step 2: Run — verify all four fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_telemetry.py -v
```

Expected: `ModuleNotFoundError: No module named 'telemetry'`

- [ ] **Step 3: Create `telemetry.py` with constants and `SessionLogger`**

```python
"""
================================================================================
PROJECT SICLO1 — TELEMETRY MODULE
================================================================================

Session-based file logging for the 100 Hz control loop.

  SessionLogger   — owns timestamped folder, telemetry.csv, summary.txt
  TelemetryThread — 10 Hz drain loop; consumer of TelemetryRingBuffer

WARMUP_CYCLES rows are written to CSV but excluded from performance statistics.
Zero I/O and zero string formatting happen on the 100 Hz hot path.

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import os
import datetime
import threading
import numpy as np
from typing import Optional

from shared_state import Siclo1State, ERR_TIMING_VIOLATION


# ============================================================================
# CONSTANTS
# ============================================================================

WARMUP_CYCLES: int = 50  # cycles — excluded from performance statistics

CSV_HEADER: str = (
    "timestamp_s,cycle,error_code,com_x,com_y,com_z,"
    "left_contact,right_contact,stability_status,"
    "left_force_n,right_force_n,stability_margin_m,"
    "compute_us,extra_0,extra_1,extra_2"
)


# ============================================================================
# SESSION LOGGER  — owns all file I/O for one simulation run
# ============================================================================

class SessionLogger:
    """
    Creates a timestamped session folder and manages telemetry.csv
    and summary.txt for one simulation run.

    Constructed inside TelemetryThread.__init__ so the folder exists
    before the first drain cycle.

    Failure mode: OSError during __init__ sets _csv = None.
    All subsequent calls degrade silently — the simulation is never interrupted.
    """

    def __init__(self, base_dir: str) -> None:
        session_name = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._session_path: str = os.path.join(base_dir, "sessions", session_name)
        self._csv: Optional[object] = None

        try:
            os.makedirs(self._session_path, exist_ok=True)
            csv_path = os.path.join(self._session_path, "telemetry.csv")
            self._csv = open(csv_path, 'w', buffering=8192)
            self._csv.write(CSV_HEADER + '\n')
        except OSError:
            self._csv = None

    @property
    def session_path(self) -> str:
        return self._session_path

    def append_row(self, row: np.ndarray) -> None:
        """Write one 16-column telemetry row to CSV. No-op if file unavailable."""
        if self._csv is None:
            return
        self._csv.write(','.join(f'{v:.6g}' for v in row) + '\n')
```

- [ ] **Step 4: Run — verify four tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add telemetry.py test_telemetry.py
git commit -m "feat: add SessionLogger with folder creation, CSV header, and append_row"
```

---

### Task 2: `SessionLogger.write_summary`

**Files:**
- Modify: `telemetry.py` (add `write_summary` method to `SessionLogger`)
- Modify: `test_telemetry.py` (add 3 tests)

- [ ] **Step 1: Append three failing tests to `test_telemetry.py`**

```python
def test_write_summary_pass(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.005, 'std_dt': 0.0003, 'min_dt': 0.003, 'max_dt': 0.009,
        'jitter_ms': 0.3, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 1000, 'analyzed_cycles': 950,
        'warmup_cycles': 50, 'coded_errors': 0,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'PASS' in content
    assert 'Total cycles   : 1000' in content
    assert 'Analyzed cycles: 950' in content
    assert logger._csv is None  # closed by write_summary


def test_write_summary_fail(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.012, 'std_dt': 0.002, 'min_dt': 0.008, 'max_dt': 0.020,
        'jitter_ms': 2.0, 'violations': 10, 'violation_rate': 0.011,
        'total_cycles': 950, 'analyzed_cycles': 900,
        'warmup_cycles': 50, 'coded_errors': 2,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'FAIL' in content
    assert 'Violations   : 10' in content
    assert 'Coded errors in ring: 2' in content


def test_write_summary_zero_analyzed_cycles(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.0, 'std_dt': 0.0, 'min_dt': 0.0, 'max_dt': 0.0,
        'jitter_ms': 0.0, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 30, 'analyzed_cycles': 0,
        'warmup_cycles': 50, 'coded_errors': 0,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'No analyzed cycles' in content
    assert 'Total cycles   : 30' in content
    assert logger._csv is None
```

- [ ] **Step 2: Run — verify three new tests fail**

```bash
pytest test_telemetry.py -v
```

Expected: 4 passed, 3 failed (`AttributeError: 'SessionLogger' object has no attribute 'write_summary'`)

- [ ] **Step 3: Add `write_summary` to `SessionLogger` in `telemetry.py`**

Add this method inside `SessionLogger`, after `append_row`:

```python
    def write_summary(self, stats: dict) -> None:
        """
        Write summary.txt from the stats dict, then close all file handles.
        try/finally guarantees handles are closed even if an exception occurs.

        PASS criteria: mean_dt < 10.5 ms AND jitter_ms < 1.0 ms.
        """
        session_name = os.path.basename(self._session_path)
        n      = stats.get('analyzed_cycles', 0)
        total  = stats.get('total_cycles', 0)
        warmup = stats.get('warmup_cycles', WARMUP_CYCLES)
        coded  = stats.get('coded_errors', 0)

        summary_file = None
        try:
            summary_path = os.path.join(self._session_path, "summary.txt")
            summary_file = open(summary_path, 'w')

            lines = [
                f"Session: {session_name}",
                f"Total cycles   : {total}",
                f"Analyzed cycles: {n}  (warmup excluded: {warmup})",
                "",
                "[TIMING]",
            ]

            if n > 0:
                mean_ms  = stats['mean_dt']        * 1000.0
                std_ms   = stats['std_dt']         * 1000.0
                min_ms   = stats['min_dt']         * 1000.0
                max_ms   = stats['max_dt']         * 1000.0
                jitter   = stats['jitter_ms']
                viol     = stats['violations']
                viol_pct = stats['violation_rate'] * 100.0
                status   = "PASS" if mean_ms < 10.5 and jitter < 1.0 else "FAIL"
                lines += [
                    f"  Mean compute : {mean_ms:.3f} ms",
                    f"  Std dev      : {std_ms:.3f} ms",
                    f"  Min          : {min_ms:.3f} ms",
                    f"  Max          : {max_ms:.3f} ms",
                    f"  Jitter       : {jitter:.3f} ms",
                    f"  Violations   : {viol}  ({viol_pct:.1f}%)",
                    f"  Status       : {status}",
                ]
            else:
                lines.append("  No analyzed cycles (run ended before warmup completed)")

            lines += [
                "",
                "[ERRORS]",
                f"  Coded errors in ring: {coded}",
            ]
            summary_file.write('\n'.join(lines) + '\n')

        finally:
            if self._csv is not None:
                self._csv.flush()
                self._csv.close()
                self._csv = None
            if summary_file is not None:
                summary_file.close()
```

- [ ] **Step 4: Run — verify all 7 tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add telemetry.py test_telemetry.py
git commit -m "feat: add SessionLogger.write_summary with PASS/FAIL/zero-cycle handling"
```

---

### Task 3: `SessionLogger` — OSError degradation

**Files:**
- Modify: `test_telemetry.py` (add 2 tests)
- `telemetry.py` — OSError handler already present; tests verify it

- [ ] **Step 1: Append two tests to `test_telemetry.py`**

```python
def test_session_logger_oserror_sets_csv_none(monkeypatch, tmp_path):
    import telemetry as _tel
    monkeypatch.setattr(_tel.os, 'makedirs', lambda *a, **k: (_ for _ in ()).throw(OSError("disk full")))
    logger = SessionLogger(str(tmp_path))
    assert logger._csv is None


def test_session_logger_append_row_noop_when_csv_none(monkeypatch, tmp_path):
    import telemetry as _tel
    monkeypatch.setattr(_tel.os, 'makedirs', lambda *a, **k: (_ for _ in ()).throw(OSError()))
    logger = SessionLogger(str(tmp_path))
    row = np.zeros(16, dtype=np.float64)
    logger.append_row(row)  # must not raise
```

- [ ] **Step 2: Run — verify both pass**

```bash
pytest test_telemetry.py -v
```

Expected: 9 passed

If either fails with a generator trick issue, replace the lambda with:

```python
def _raise(*a, **k):
    raise OSError("disk full")
monkeypatch.setattr(_tel.os, 'makedirs', _raise)
```

- [ ] **Step 3: Commit**

```bash
git add test_telemetry.py
git commit -m "test: verify SessionLogger OSError degradation path"
```

---

### Task 4: `TelemetryThread` — extraction and CSV drain

**Files:**
- Modify: `telemetry.py` (add `TelemetryThread` class — full implementation)
- Modify: `test_telemetry.py` (add helpers + 3 tests)

- [ ] **Step 1: Append helpers and three failing tests to `test_telemetry.py`**

```python
# ---------------------------------------------------------------------------
# Helpers shared by all TelemetryThread tests
# ---------------------------------------------------------------------------
from unittest.mock import MagicMock, patch
from shared_state import Siclo1State, ERR_TIMING_VIOLATION
from telemetry import TelemetryThread, WARMUP_CYCLES


def _make_row(cycle: int, err: int = 0, compute_us: float = 5000.0) -> np.ndarray:
    """Build a 16-column ring buffer row for testing."""
    row = np.zeros(16, dtype=np.float64)
    row[0]  = cycle * 0.01   # timestamp_s
    row[1]  = float(cycle)   # cycle number  (col 1)
    row[2]  = float(err)     # error code    (col 2)
    row[12] = compute_us     # compute_us    (col 12)
    return row


@pytest.fixture
def sim_state():
    return Siclo1State()


@pytest.fixture
def mock_session(tmp_path):
    m = MagicMock()
    m.session_path = str(tmp_path / "sessions" / "2026-01-01_00-00-00")
    return m


# ---------------------------------------------------------------------------
# Task 4 tests
# ---------------------------------------------------------------------------

def test_telemetry_thread_has_session_path(sim_state, tmp_path):
    with patch('telemetry.SessionLogger') as MockLogger:
        instance = MockLogger.return_value
        instance.session_path = str(tmp_path / "sessions" / "test")
        t = TelemetryThread(sim_state)
        assert t.session_path == instance.session_path


def test_telemetry_thread_drain_calls_append_row_per_row(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        for cycle in [1, 2, 3]:
            sim_state.telemetry.write(_make_row(cycle))
        t._drain()
        assert mock_session.append_row.call_count == 3


def test_telemetry_thread_total_cycles_increments(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        for cycle in range(1, 6):
            sim_state.telemetry.write(_make_row(cycle))
        t._drain()
        assert t._total_cycles == 5
```

- [ ] **Step 2: Run — verify three new tests fail**

```bash
pytest test_telemetry.py -v
```

Expected: 9 passed, 3 failed (`ImportError` or `AttributeError` on `TelemetryThread`)

- [ ] **Step 3: Add `TelemetryThread` to `telemetry.py`**

Append after `SessionLogger`:

```python
# ============================================================================
# TELEMETRY THREAD  — extracted from HeartBeat.py, extended with file logging
# ============================================================================

class TelemetryThread(threading.Thread):
    """
    Low-priority consumer thread — drains TelemetryRingBuffer at ~10 Hz.

    Responsibilities:
      - Call SessionLogger.append_row() for every ring buffer row (all cycles)
      - Update post-warmup online accumulators (cycle > WARMUP_CYCLES only)
      - At tail of run(): final drain + write_summary()
      - Maintain in-memory _log_lines for flush_to_stdout() (unchanged behaviour)

    The 100 Hz hot path (HeartBeat.step) has zero knowledge of this class.
    """
    daemon = True

    def __init__(self, state: Siclo1State) -> None:
        super().__init__(name="TelemetryConsumer")
        self._state       = state
        self._stop_event  = threading.Event()
        self._log_lines: list = []

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self._session = SessionLogger(base_dir)

        # Online accumulators — post-warmup only, zero allocation (scalars)
        self._total_cycles:    int   = 0
        self._analyzed_cycles: int   = 0
        self._sum_compute:     float = 0.0        # seconds
        self._sum_sq_compute:  float = 0.0
        self._max_compute:     float = 0.0
        self._min_compute:     float = float('inf')
        self._violations:      int   = 0

    @property
    def session_path(self) -> str:
        return self._session.session_path

    def run(self) -> None:
        while not self._stop_event.is_set():
            self._drain()
            self._stop_event.wait(timeout=0.1)  # ~10 Hz
        # Tail: final drain, then write session files
        self._drain()
        stats = self.get_summary_stats()
        stats['coded_errors'] = self._state._error_write_idx
        self._session.write_summary(stats)

    def _drain(self) -> None:
        batch = self._state.telemetry.read_batch()
        if batch.shape[0] > 0:
            self._format_batch(batch)

    def _format_batch(self, batch: np.ndarray) -> None:
        """Write all rows to CSV; accumulate post-warmup stats; build log lines."""
        for row in batch:
            # Always write to CSV (warmup rows kept for black-box debugging)
            self._session.append_row(row)
            self._total_cycles += 1

            ts, cycle, err, cx, cy, cz, lc, rc, stab, lf, rf, margin, comp_us, *_ = row

            # Human-readable in-memory log (unchanged from HeartBeat.py)
            line = (f"[t={ts:7.3f}s c={int(cycle):>6d}] "
                    f"COM=[{cx:+.3f},{cy:+.3f},{cz:+.3f}] "
                    f"L={int(lc)} R={int(rc)} Stab={int(stab)} "
                    f"F=[{lf:.0f},{rf:.0f}] "
                    f"margin={margin:.4f} "
                    f"compute={comp_us:.0f}\u00b5s")
            if int(err) > 0:
                line += f" ERR={int(err)}"
            self._log_lines.append(line)

            # Post-warmup accumulation: strict greater-than (cycle 50 is warmup)
            if cycle > WARMUP_CYCLES:
                compute_s = comp_us / 1_000_000.0   # µs → seconds
                self._analyzed_cycles += 1
                self._sum_compute     += compute_s
                self._sum_sq_compute  += compute_s * compute_s
                if compute_s > self._max_compute:
                    self._max_compute = compute_s
                if compute_s < self._min_compute:
                    self._min_compute = compute_s
                if int(err) == ERR_TIMING_VIOLATION:
                    self._violations += 1

    def get_summary_stats(self) -> dict:
        """
        Timing stats in the same dict shape as HeartbeatController.get_statistics().
        Returns zero-filled dict when no post-warmup cycles were analyzed
        (e.g. run terminated inside the warm-up window).
        """
        n = self._analyzed_cycles
        if n == 0:
            return {
                'mean_dt':         0.0,
                'std_dt':          0.0,
                'min_dt':          0.0,
                'max_dt':          0.0,
                'jitter_ms':       0.0,
                'violations':      0,
                'violation_rate':  0.0,
                'total_cycles':    self._total_cycles,
                'analyzed_cycles': 0,
                'warmup_cycles':   WARMUP_CYCLES,
                'coded_errors':    0,
            }
        mean     = self._sum_compute / n
        variance = (self._sum_sq_compute / n) - (mean * mean)
        std      = variance ** 0.5 if variance > 0.0 else 0.0
        return {
            'mean_dt':         mean,
            'std_dt':          std,
            'min_dt':          self._min_compute,
            'max_dt':          self._max_compute,
            'jitter_ms':       std * 1000.0,
            'violations':      self._violations,
            'violation_rate':  self._violations / max(1, n),
            'total_cycles':    self._total_cycles,
            'analyzed_cycles': n,
            'warmup_cycles':   WARMUP_CYCLES,
            'coded_errors':    0,   # caller overwrites from _state._error_write_idx
        }

    def stop(self) -> None:
        self._stop_event.set()

    def flush_to_stdout(self) -> None:
        for line in self._log_lines:
            print(line)
        self._log_lines.clear()

    def log(self, msg: str) -> None:
        """Compatibility shim for cold-path messages (init, shutdown, tests)."""
        self._log_lines.append(msg)
```

- [ ] **Step 4: Run — verify 12 tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 12 passed

- [ ] **Step 5: Commit**

```bash
git add telemetry.py test_telemetry.py
git commit -m "feat: add TelemetryThread to telemetry.py with CSV drain and online accumulators"
```

---

### Task 5: Warm-up filter boundary conditions

**Files:**
- Modify: `test_telemetry.py` (add 3 tests)
- `telemetry.py` — implementation already complete from Task 4

- [ ] **Step 1: Append three tests to `test_telemetry.py`**

```python
def test_warmup_cycles_excluded_from_accumulators(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        # Cycles 1..50: all warmup — none should update accumulators
        for cycle in range(1, WARMUP_CYCLES + 1):
            sim_state.telemetry.write(_make_row(cycle, compute_us=9000.0))
        t._drain()
        assert t._analyzed_cycles == 0
        assert t._total_cycles == WARMUP_CYCLES


def test_post_warmup_cycles_update_accumulators(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        sim_state.telemetry.write(_make_row(50, compute_us=4000.0))  # warmup (==50, not >50)
        sim_state.telemetry.write(_make_row(51, compute_us=6000.0))  # analyzed (>50)
        t._drain()
        assert t._analyzed_cycles == 1
        assert t._total_cycles == 2
        assert abs(t._sum_compute - 6000.0 / 1_000_000.0) < 1e-12


def test_violations_counted_post_warmup_only(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        sim_state.telemetry.write(_make_row(49, err=ERR_TIMING_VIOLATION))  # warmup
        sim_state.telemetry.write(_make_row(51, err=ERR_TIMING_VIOLATION))  # analyzed
        t._drain()
        assert t._violations == 1
```

- [ ] **Step 2: Run — verify all 15 tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 15 passed

If boundary tests fail: check that `_format_batch` uses `cycle > WARMUP_CYCLES` (strict `>`), not `>=`.

- [ ] **Step 3: Commit**

```bash
git add test_telemetry.py
git commit -m "test: verify warm-up filter boundary and violation counting"
```

---

### Task 6: `get_summary_stats` correctness

**Files:**
- Modify: `test_telemetry.py` (add 3 tests)
- `telemetry.py` — implementation already complete from Task 4

- [ ] **Step 1: Append three tests to `test_telemetry.py`**

```python
def test_get_summary_stats_zero_cycles_returns_safe_dict(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        stats = t.get_summary_stats()
        assert stats['analyzed_cycles'] == 0
        assert stats['mean_dt']   == 0.0
        assert stats['jitter_ms'] == 0.0
        assert stats['violations'] == 0
        assert stats['warmup_cycles'] == WARMUP_CYCLES


def test_get_summary_stats_correct_mean_and_extremes(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        # 4 ms and 8 ms → mean 6 ms, min 4 ms, max 8 ms
        sim_state.telemetry.write(_make_row(51, compute_us=4000.0))
        sim_state.telemetry.write(_make_row(52, compute_us=8000.0))
        t._drain()
        stats = t.get_summary_stats()
        assert stats['analyzed_cycles'] == 2
        assert abs(stats['mean_dt'] - 0.006) < 1e-9
        assert abs(stats['min_dt']  - 0.004) < 1e-9
        assert abs(stats['max_dt']  - 0.008) < 1e-9


def test_get_summary_stats_violation_rate(sim_state, mock_session):
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        # 1 violation in 2 post-warmup cycles → 50%
        sim_state.telemetry.write(_make_row(51, err=ERR_TIMING_VIOLATION))
        sim_state.telemetry.write(_make_row(52, err=0))
        t._drain()
        stats = t.get_summary_stats()
        assert stats['violations'] == 1
        assert abs(stats['violation_rate'] - 0.5) < 1e-9
```

- [ ] **Step 2: Run — verify all 18 tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 18 passed

- [ ] **Step 3: Commit**

```bash
git add test_telemetry.py
git commit -m "test: verify get_summary_stats values including zero-cycle guard"
```

---

### Task 7: `run()` tail — final drain and `write_summary`

**Files:**
- Modify: `test_telemetry.py` (add 2 tests)
- `telemetry.py` — implementation already complete from Task 4

- [ ] **Step 1: Append two tests to `test_telemetry.py`**

```python
def test_run_tail_calls_write_summary_once(sim_state, mock_session):
    """After stop() + join(), write_summary must be called exactly once."""
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        t.start()
        t.stop()
        t.join(timeout=1.0)
        assert not t.is_alive(), "Thread did not exit within 1 s"
        mock_session.write_summary.assert_called_once()


def test_run_tail_drains_remaining_rows(sim_state, mock_session):
    """Rows written just before stop() must still reach append_row."""
    with patch('telemetry.SessionLogger', return_value=mock_session):
        t = TelemetryThread(sim_state)
        t.start()
        for cycle in range(1, 6):
            sim_state.telemetry.write(_make_row(cycle))
        t.stop()
        t.join(timeout=1.0)
        # All 5 rows must have been consumed (in drain loop or tail drain)
        assert mock_session.append_row.call_count == 5
```

- [ ] **Step 2: Run — verify all 20 tests pass**

```bash
pytest test_telemetry.py -v
```

Expected: 20 passed

- [ ] **Step 3: Commit**

```bash
git add test_telemetry.py
git commit -m "test: verify run() tail drains buffer and calls write_summary on shutdown"
```

---

### Task 8: `HeartBeat.py` — import swap, `get_summary_stats`, `join` timeout

**Files:**
- Modify: `HeartBeat.py`

No new tests — `step()` is byte-for-byte unchanged. Verified via smoke import.

- [ ] **Step 1: Delete the `TelemetryThread` class from `HeartBeat.py`**

Remove lines 73–118 (the entire class, from `class TelemetryThread(threading.Thread):` through the final `def log` method closing brace). After deletion, the line that was `class HeartbeatController` should immediately follow the imports/constants block.

- [ ] **Step 2: Add `from telemetry import TelemetryThread` to `HeartBeat.py`**

After the existing `import active_balance` line (currently around line 49), add:

```python
from telemetry import TelemetryThread
```

- [ ] **Step 3: Update stats call in `_print_final_summary`**

In `Siclo1Controller._print_final_summary`, replace:

```python
        ts = self.heartbeat.get_statistics()
```

with:

```python
        ts = self._telemetry_thread.get_summary_stats()
```

- [ ] **Step 4: Update `join` timeout in `Siclo1Controller.shutdown`**

In `Siclo1Controller.shutdown`, replace:

```python
        self._telemetry_thread.join(timeout=1.0)
```

with:

```python
        self._telemetry_thread.join(timeout=2.0)
```

- [ ] **Step 5: Smoke-test the import**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -c "from HeartBeat import Siclo1Controller; print('Import OK')"
```

Expected: `Import OK`

- [ ] **Step 6: Run full telemetry suite**

```bash
pytest test_telemetry.py -v
```

Expected: 20 passed

- [ ] **Step 7: Commit**

```bash
git add HeartBeat.py
git commit -m "refactor: replace TelemetryThread in HeartBeat with import from telemetry"
```

---

## Final Verification Checklist

- [ ] `pytest test_telemetry.py -v` → **20 passed, 0 failed**
- [ ] `python -c "from HeartBeat import Siclo1Controller; print('OK')"` → `OK`
- [ ] `grep -n "class TelemetryThread" HeartBeat.py` → no output (class removed)
- [ ] `grep -n "from telemetry import" HeartBeat.py` → import line present
- [ ] `grep -n "get_summary_stats" HeartBeat.py` → updated call present in `_print_final_summary`
- [ ] `grep -n "join(timeout=2" HeartBeat.py` → updated timeout present
- [ ] `grep -c "def step" HeartBeat.py` → `1` (step function present and unchanged)
- [ ] `python -c "import shared_state; print('shared_state unchanged')"` → no errors
