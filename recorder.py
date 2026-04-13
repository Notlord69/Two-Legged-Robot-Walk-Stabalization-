"""
================================================================================
PROJECT SICLO1 — VIDEO RECORDER
================================================================================

Standalone MP4 recorder that works in both DIRECT and GUI mode.
Uses p.getCameraImage() (TinyRenderer, software-only) — no GPU or OpenGL
context required, so it produces a file on every run including headless.

  VideoRecorder  — daemon thread; captures frames at `fps` Hz and writes
                   via OpenCV VideoWriter to <session_path>/walk.mp4

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import os
import time
import threading
from typing import Optional

import numpy as np


# ============================================================================
# CONSTANTS
# ============================================================================

RECORDER_FPS:    int = 15   # Hz — video playback rate and capture rate
RECORDER_WIDTH:  int = 320  # px — TinyRenderer resolution (keep low for speed)
RECORDER_HEIGHT: int = 240  # px
FOURCC:          str = 'mp4v'   # MPEG-4 codec; broad OpenCV support on Linux


# ============================================================================
# VIDEO RECORDER
# ============================================================================

class VideoRecorder(threading.Thread):
    """Captures frames from PyBullet TinyRenderer and writes walk.mp4.

    Runs as a daemon thread independently of the 100 Hz control loop.
    Calls p.getCameraImage() at RECORDER_FPS Hz — safe because PyBullet
    serialises per-client access with an internal mutex.

    Usage:
        rec = VideoRecorder(physics_client, session_path)
        rec.start()
        # ... simulation runs ...
        video_path = rec.stop()
        rec.join(timeout=5.0)
    """

    daemon = True

    def __init__(self, physics_client: int, session_path: str,
                 fps: int = RECORDER_FPS,
                 width: int = RECORDER_WIDTH,
                 height: int = RECORDER_HEIGHT) -> None:
        super().__init__(name='VideoRecorder')
        self._physics_client = physics_client
        self._video_path     = os.path.join(session_path, 'walk.mp4')
        self._fps            = fps       # Hz — capture and playback rate
        self._width          = width     # px
        self._height         = height    # px
        self._stop_event     = threading.Event()
        self._frame_count    = 0         # GIL-safe int; read by tests

    # ------------------------------------------------------------------ #
    @property
    def video_path(self) -> str:
        return self._video_path

    @property
    def frame_count(self) -> int:
        return self._frame_count

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        """Capture loop — exits when stop() is called."""
        import cv2
        import sim.interface
        from shared_state import shared_state

        os.makedirs(os.path.dirname(self._video_path), exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*FOURCC)
        writer = cv2.VideoWriter(
            self._video_path, fourcc, float(self._fps),
            (self._width, self._height),
        )

        period = 1.0 / self._fps          # target seconds per frame
        try:
            while not self._stop_event.is_set():
                t0 = time.perf_counter()

                # Camera target tracks robot COM so it stays in frame.
                # shared_state.com_position is a numpy array; slice copy is GIL-safe.
                target = tuple(float(v) for v in shared_state.com_position[:3])
                rgb = sim.interface.capture_frame(
                    self._physics_client, target, self._width, self._height,
                )
                bgr = rgb[:, :, ::-1]     # RGB → BGR for OpenCV
                writer.write(bgr)
                self._frame_count += 1

                elapsed   = time.perf_counter() - t0
                remaining = period - elapsed
                if remaining > 0:
                    self._stop_event.wait(timeout=remaining)   # interruptible sleep
        finally:
            writer.release()   # flush + close MP4 — always executes

    # ------------------------------------------------------------------ #
    def stop(self) -> str:
        """Signal the thread to stop after finishing the current frame.

        Returns the expected video path.  Call join() after stop() to ensure
        the final writer.release() has executed before reading the file.
        """
        self._stop_event.set()
        return self._video_path
