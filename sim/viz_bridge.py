"""Subprocess-based GUI bridge: zero GIL contention, pure process isolation.

The 100 Hz physics loop calls push_pose() to write pose data into a shared
memory buffer. A daemon subprocess (viz/gui_worker.py) reads the buffer at
viz_fps Hz and mirrors the pose to a p.GUI window.

All p.* calls stay in the subprocess — never in this file.
"""
import multiprocessing
import time
import numpy as np
from multiprocessing.shared_memory import SharedMemory
from typing import Optional


class VizBridge:
    """Owns the shared_memory pose buffer and the GUI subprocess lifecycle.

    Buffer layout (float64 array):
        [0]      seq_num      — monotonic counter; GUI skips if unchanged
        [1:4]    base_pos     — (x, y, z) world position, metres
        [4:8]    base_orn     — quaternion (x, y, z, w)
        [8:8+N]  joint_angles — one value per joint in joint_names order
    """

    def __init__(self, joint_names: list, urdf_path: str,
                 viz_fps: int = 30, spawn_z: float = 0.8806) -> None:
        self._joint_names = list(joint_names)
        self._urdf_path = urdf_path
        self._viz_fps = viz_fps
        self._spawn_z = spawn_z
        self._n = len(self._joint_names)

        # Allocate shared memory: [seq(1) + pos(3) + orn(4) + joints(N)] × 8 bytes
        _n_floats = 1 + 3 + 4 + self._n
        self._shm = SharedMemory(create=True, size=_n_floats * 8)
        self._arr = np.ndarray((_n_floats,), dtype=np.float64, buffer=self._shm.buf)
        self._arr[:] = 0.0
        self._arr[0] = -1.0  # ready sentinel: -1 = subprocess not yet ready

        self._process: Optional[multiprocessing.Process] = None
        self._active: bool = False

    def push_pose(self, base_pos, base_orn,
                  joint_positions: dict) -> None:
        """Write latest pose into shared buffer. Non-blocking, no allocation.

        base_pos:        (x, y, z) or array-like, metres, world frame
        base_orn:        (x, y, z, w) or array-like, quaternion
        joint_positions: {joint_name: angle_rad}

        If subprocess has died, sets _active=False silently.
        """
        if not self._active:
            return
        if self._process is not None and not self._process.is_alive():
            self._active = False
            return
        arr = self._arr
        # Write pose fields first (seq_num incremented last — write barrier)
        arr[1] = base_pos[0]
        arr[2] = base_pos[1]
        arr[3] = base_pos[2]
        arr[4] = base_orn[0]
        arr[5] = base_orn[1]
        arr[6] = base_orn[2]
        arr[7] = base_orn[3]
        for i, jname in enumerate(self._joint_names):
            arr[8 + i] = joint_positions.get(jname, 0.0)
        arr[0] += 1.0  # seq_num: increment last

    def start(self, _timeout: float = 5.0) -> None:
        """Launch GUI subprocess. Spin-waits up to _timeout seconds for ready signal.

        On timeout: logs warning, leaves _active=False (physics runs headlessly).
        _timeout is exposed for testing; production code uses the default 5.0 s.
        """
        from viz import gui_worker  # deferred: keeps physics-process imports clean

        ctx = multiprocessing.get_context('spawn')  # 'spawn' avoids fork+OpenGL issues on WSL2
        self._process = ctx.Process(
            target=gui_worker.main,
            args=(
                self._shm.name, self._n, self._joint_names,
                self._urdf_path, self._viz_fps, self._spawn_z,
            ),
            daemon=True,
        )
        self._process.start()

        # Spin-wait: arr[0] changes from -1 to ≥0 once the subprocess is ready
        deadline = time.perf_counter() + _timeout
        while time.perf_counter() < deadline:
            if self._arr[0] >= 0.0:
                self._active = True
                return
            time.sleep(0.05)

        print("[VizBridge][WARN] GUI subprocess did not become ready within "
              f"{_timeout:.1f} s — running headlessly")
        self._active = False

    def stop(self) -> None:
        """Terminate subprocess and release shared memory."""
        if self._process is not None and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.kill()
        try:
            self._shm.close()
        except Exception:
            pass
        try:
            self._shm.unlink()
        except Exception:
            pass
        self._active = False

    @property
    def is_alive(self) -> bool:
        """True if the GUI subprocess is running."""
        return self._process is not None and self._process.is_alive()
