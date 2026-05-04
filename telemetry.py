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
from regime_monitor import RegimeMonitor, PrimaryRegime, Condition


# ============================================================================
# CONSTANTS
# ============================================================================

WARMUP_CYCLES: int = 50  # cycles — excluded from performance statistics

REGIME_CONFIDENCE_SIGNALS: tuple = (
    'base_z', 'angular_velocity',
    'contact_force_left', 'contact_force_right',
    'contact_force_stance', 'contact_force_swing',
    'wbc_tracking_error', 'ramp_gain',
)

REGIME_CSV_HEADER: str = (
    "timestamp_s,cycle,regime,condition,"
    + ",".join(f"conf_{s}" for s in REGIME_CONFIDENCE_SIGNALS)
)

# 72-column header matching TelemetryRingBuffer.COLS layout in shared_state.py
CSV_HEADER: str = (
    # Core timing & state (0-15)
    "timestamp_s,cycle,error_code,"
    "com_x,com_y,com_z,"
    "com_vel_x,com_vel_y,com_vel_z,"
    "left_contact,right_contact,stability_status,"
    "left_force_n,right_force_n,stability_margin_m,"
    "compute_us,"
    # Base angular velocity (16-18)
    "base_ang_vel_x,base_ang_vel_y,base_ang_vel_z,"
    # Joint angles - actual (19-24)
    "L_hip_fwd_rad,L_knee_rad,L_ankle_rad,"
    "R_hip_fwd_rad,R_knee_rad,R_ankle_rad,"
    # IK commanded angles (25-32)
    "ik_L_hip_roll,ik_L_hip_pitch,ik_L_knee,ik_L_ankle,"
    "ik_R_hip_roll,ik_R_hip_pitch,ik_R_knee,ik_R_ankle,"
    # WBC tracking error (33-38)
    "err_L_hip_fwd,err_L_knee,err_L_ankle,"
    "err_R_hip_fwd,err_R_knee,err_R_ankle,"
    # Torque saturation flags (39-44)
    "sat_L_hip_fwd,sat_L_knee,sat_L_ankle,"
    "sat_R_hip_fwd,sat_R_knee,sat_R_ankle,"
    # Applied torques (45-50)
    "tau_L_hip_fwd,tau_L_knee,tau_L_ankle,"
    "tau_R_hip_fwd,tau_R_knee,tau_R_ankle,"
    # Gait state (51-55)
    "step_phase,swing_phase,swing_side,"
    "mission_state,ramp_gain,"
    # Foot positions - actual (56-61)
    "L_foot_x,L_foot_y,L_foot_z,"
    "R_foot_x,R_foot_y,R_foot_z,"
    # Foot targets (62-67)
    "L_target_x,L_target_y,L_target_z,"
    "R_target_x,R_target_y,R_target_z,"
    # Capture point & slip (68-71)
    "capture_pt_x,capture_pt_y,"
    "L_slip,R_slip"
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
        self._regime_csv: Optional[IO[str]] = None

        try:
            os.makedirs(self._session_path, exist_ok=True)
            csv_path = os.path.join(self._session_path, "telemetry.csv")
            self._csv = open(csv_path, 'w', buffering=8192)
            self._csv.write(CSV_HEADER + '\n')

            regime_path = os.path.join(self._session_path, "regime.csv")
            self._regime_csv = open(regime_path, 'w', buffering=8192)
            self._regime_csv.write(REGIME_CSV_HEADER + '\n')
        except OSError:
            self._csv = None
            self._regime_csv = None

    @property
    def session_path(self) -> str:
        return self._session_path

    def append_row(self, row: np.ndarray) -> None:
        """Write one 72-column telemetry row to CSV. No-op if file unavailable."""
        if self._csv is None:
            return
        self._csv.write(','.join(f'{v:.6g}' for v in row) + '\n')

    def append_regime_row(self, ts: float, cycle: int,
                          regime: PrimaryRegime, condition: Condition,
                          conf: dict) -> None:
        """Write one regime classification row. No-op if file unavailable."""
        if self._regime_csv is None:
            return
        fields = [f'{ts:.6g}', str(cycle), regime.name, condition.name]
        for sig in REGIME_CONFIDENCE_SIGNALS:
            val = conf.get(sig)
            fields.append(f'{val:.4f}' if val is not None else '')
        self._regime_csv.write(','.join(fields) + '\n')

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
        cmd    = stats.get('argv_command', '')

        summary_file = None
        try:
            summary_path = os.path.join(self._session_path, "summary.txt")
            summary_file = open(summary_path, 'w')

            lines = [
                f"Session: {session_name}",
                f"Command        : {cmd if cmd else '(not recorded)'}",
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
            if self._regime_csv is not None:
                self._regime_csv.flush()
                self._regime_csv.close()
                self._regime_csv = None
            if summary_file is not None:
                summary_file.close()


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

    argv_command: the full CLI invocation string (e.g. "python3 main.py --gui --walk 2.0")
                  written into summary.txt so every session is self-describing.
    quiet:        when True, flush_to_stdout() and log() are no-ops — all data
                  still goes to the CSV/summary; only terminal output is suppressed.
    """
    daemon = True

    def __init__(self, state: Siclo1State, argv_command: str = "",
                 quiet: bool = True) -> None:
        super().__init__(name="TelemetryConsumer")
        self._state        = state
        self._stop_event   = threading.Event()
        self._log_lines: list = []
        self._argv_command = argv_command   # stored for summary.txt
        self._quiet        = quiet          # suppresses all terminal output
        self._regime_monitor = RegimeMonitor()

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
        try:
            while not self._stop_event.is_set():
                self._drain()
                self._stop_event.wait(timeout=0.1)  # ~10 Hz
        except Exception as exc:
            self.log(f"[FATAL] TelemetryThread crashed: {exc}")
        finally:
            # Tail drain + summary ALWAYS execute, even on exception
            self._drain()
            stats = self.get_summary_stats()
            stats['coded_errors'] = self._state._error_write_idx
            stats['argv_command'] = self._argv_command
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

            # Regime classification (10 Hz observer)
            regime, condition, conf = self._regime_monitor.classify(row)
            self._session.append_regime_row(
                row[0], int(row[1]), regime, condition, conf)

            # Unpack key columns (72-col layout, see shared_state.py)
            ts       = row[0]   # timestamp_s
            cycle    = row[1]   # cycle
            err      = row[2]   # error_code
            cx, cy, cz = row[3], row[4], row[5]    # com position
            lc       = row[9]   # left_contact
            rc       = row[10]  # right_contact
            stab     = row[11]  # stability_status
            lf       = row[12]  # left_force_n
            rf       = row[13]  # right_force_n
            margin   = row[14]  # stability_margin_m
            comp_us  = row[15]  # compute_us
            step_ph  = row[51]  # step_phase
            swing_ph = row[52]  # swing_phase

            # Human-readable in-memory log (key fields only)
            line = (f"[t={ts:7.3f}s c={int(cycle):>6d}] "
                    f"COM=[{cx:+.3f},{cy:+.3f},{cz:+.3f}] "
                    f"L={int(lc)} R={int(rc)} Stab={int(stab)} "
                    f"F=[{lf:.0f},{rf:.0f}] "
                    f"margin={margin:.4f} "
                    f"phase={int(step_ph)} swing={swing_ph:.2f} "
                    f"compute={comp_us:.0f}us "
                    f"regime={regime.name} cond={condition.name}")
            if int(err) > 0:
                line += f" ERR={int(err)}"
            self._log_lines.append(line)

            # Post-warmup accumulation: strict greater-than (cycle 50 is warmup)
            if cycle > WARMUP_CYCLES:
                compute_s = comp_us / 1_000_000.0   # us -> seconds
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
        """Print buffered log lines to terminal, then clear them.
        No-op when quiet=True — lines are still buffered (tests can inspect them)."""
        if not self._quiet:
            for line in self._log_lines:
                print(line)
        self._log_lines.clear()

    def log(self, msg: str) -> None:
        """Compatibility shim for cold-path messages (init, shutdown, tests).
        Always buffers; terminal output suppressed when quiet=True."""
        self._log_lines.append(msg)
