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
from typing import IO, Optional

from shared_state import Siclo1State, ERR_TIMING_VIOLATION


# ============================================================================
# CONSTANTS
# ============================================================================

WARMUP_CYCLES: int = 50  # cycles — excluded from performance statistics

# 16-column header matching TelemetryRingBuffer.COLS layout in shared_state.py
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
        self._csv: Optional[IO[str]] = None

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

    def write_summary(self, stats: dict) -> None:
        """
        Write summary.txt from the stats dict, then close all file handles.
        try/finally guarantees handles are closed even if an exception occurs.

        PASS criteria: mean_dt < 10.5 ms AND jitter_ms < 1.0 ms.
        The 0.5 ms headroom accounts for measurement overhead; per-cycle
        violations are counted separately in 'violations'.
        """
        if not os.path.isdir(self._session_path):
            return  # degraded mode — session folder never created (OSError at init)
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
