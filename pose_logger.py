"""
================================================================================
PROJECT SICLO1 — POSE LOGGER
================================================================================

Pre-allocated numpy ring buffer that captures full robot pose per cycle.
Zero file I/O during the run — save() writes poses.npy once at shutdown.

Layout per row (19 × float64):
  [0]     sim_time (s)
  [1–3]   base_pos (x, y, z) m
  [4–7]   base_orn (qx, qy, qz, qw)
  [8]     Left_Hip_Forwards (rad)
  [9]     Left_Knee (rad)
  [10]    Left_Ankle (rad)
  [11]    Right_Hip_Fowards (rad)   — note: URDF typo preserved
  [12]    Right_Knee (rad)
  [13]    Right_Ankle (rad)
  [14]    left_foot_force (N)
  [15]    right_foot_force (N)
  [16]    stability_status (StabilityStatus.value)
  [17]    mission_state (MissionState.value)
  [18]    spare (0.0)

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import os
import numpy as np

_COLS: int = 19  # float64 columns per row

_JOINT_COLS = [
    ('Left_Hip_Forwards', 8),
    ('Left_Knee',         9),
    ('Left_Ankle',        10),
    ('Right_Hip_Fowards', 11),   # URDF typo: "Fowards"
    ('Right_Knee',        12),
    ('Right_Ankle',       13),
]


class PoseLogger:
    """Pre-allocated numpy ring buffer for per-cycle robot pose capture.

    record() is a single numpy row write — zero allocation, zero file I/O.
    save() is called once at shutdown; writes poses.npy to the session folder.
    """

    def __init__(self, max_cycles: int = 20_000) -> None:
        self._buf: np.ndarray = np.zeros((max_cycles, _COLS), dtype=np.float64)
        self._idx: int = 0
        self._max: int = max_cycles

    def record(
        self,
        sim_time:         float,
        base_pos:         tuple,
        base_orn:         tuple,
        joint_positions:  dict,
        left_force:       float,
        right_force:      float,
        stability_status: int,
        mission_state:    int,
    ) -> None:
        """Write one pose row into the pre-allocated buffer.

        Silently drops the record if the buffer is full.
        Missing joint names default to 0.0 — no KeyError.
        """
        if self._idx >= self._max:
            return
        row = self._buf[self._idx]
        row[0]  = sim_time
        row[1]  = base_pos[0]
        row[2]  = base_pos[1]
        row[3]  = base_pos[2]
        row[4]  = base_orn[0]   # qx
        row[5]  = base_orn[1]   # qy
        row[6]  = base_orn[2]   # qz
        row[7]  = base_orn[3]   # qw
        for jname, col in _JOINT_COLS:
            row[col] = joint_positions.get(jname, 0.0)
        row[14] = left_force
        row[15] = right_force
        row[16] = float(stability_status)
        row[17] = float(mission_state)
        row[18] = 0.0           # spare
        self._idx += 1

    def save(self, session_path: str) -> str:
        """Write recorded rows to poses.npy in session_path.

        Returns the absolute path to the written file.
        Saves only rows actually recorded (_idx rows).
        Call once at shutdown, after the run loop completes.
        """
        path = os.path.join(session_path, 'poses.npy')
        np.save(path, self._buf[:self._idx])
        return path
