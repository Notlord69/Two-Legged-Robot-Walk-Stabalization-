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
