"""Tests for VideoRecorder in recorder.py."""
import os
import time
import numpy as np
import pytest
from unittest.mock import patch, MagicMock


# ── helpers ──────────────────────────────────────────────────────────────── #

def _fake_rgb(width=320, height=240):
    """Return a solid-colour RGB frame (uint8)."""
    return np.zeros((height, width, 3), dtype=np.uint8)


def _make_recorder(tmp_path, fps=15):
    from recorder import VideoRecorder
    return VideoRecorder(physics_client=0, session_path=str(tmp_path), fps=fps)


# ── tests ─────────────────────────────────────────────────────────────────── #

class TestVideoRecorderLifecycle:

    def test_video_path_in_session_folder(self, tmp_path):
        rec = _make_recorder(tmp_path)
        assert rec.video_path == str(tmp_path / 'walk.mp4')

    def test_initial_frame_count_is_zero(self, tmp_path):
        rec = _make_recorder(tmp_path)
        assert rec.frame_count == 0

    def test_stop_returns_video_path(self, tmp_path):
        rec = _make_recorder(tmp_path)
        path = rec.stop()
        assert path == rec.video_path

    def test_mp4_file_created_after_run(self, tmp_path):
        """VideoRecorder writes a real MP4 file using TinyRenderer frames."""
        import pybullet as p

        physics_client = p.connect(p.DIRECT)
        try:
            from recorder import VideoRecorder
            rec = VideoRecorder(physics_client=physics_client,
                                session_path=str(tmp_path), fps=5)
            rec.start()
            time.sleep(0.5)   # ~2–3 frames at 5fps
            rec.stop()
            rec.join(timeout=5.0)
        finally:
            p.disconnect(physicsClientId=physics_client)

        assert os.path.isfile(tmp_path / 'walk.mp4'), \
            "walk.mp4 was not written — check OpenCV codec or TinyRenderer"
        assert os.path.getsize(tmp_path / 'walk.mp4') > 0, \
            "walk.mp4 is empty — VideoWriter may have silently failed"

    def test_frame_count_increments(self, tmp_path):
        """frame_count increases while the thread is running."""
        import pybullet as p

        physics_client = p.connect(p.DIRECT)
        try:
            from recorder import VideoRecorder
            rec = VideoRecorder(physics_client=physics_client,
                                session_path=str(tmp_path), fps=10)
            rec.start()
            time.sleep(0.4)   # ~4 frames at 10fps
            count = rec.frame_count
            rec.stop()
            rec.join(timeout=5.0)
        finally:
            p.disconnect(physicsClientId=physics_client)

        assert count >= 2, f"Expected ≥2 frames, got {count}"

    def test_recorder_is_daemon(self, tmp_path):
        rec = _make_recorder(tmp_path)
        assert rec.daemon is True

    def test_stop_before_start_does_not_raise(self, tmp_path):
        """stop() sets the event; it is safe to call before start()."""
        rec = _make_recorder(tmp_path)
        rec.stop()   # should not raise


class TestVideoRecorderHeadless:
    """Verify recording works without --gui (DIRECT physics client)."""

    def test_creates_file_on_direct_client(self, tmp_path):
        """Core guarantee: file exists after headless run."""
        import pybullet as p

        client = p.connect(p.DIRECT)
        try:
            from recorder import VideoRecorder
            rec = VideoRecorder(physics_client=client,
                                session_path=str(tmp_path), fps=5)
            rec.start()
            time.sleep(0.4)
            rec.stop()
            rec.join(timeout=5.0)
        finally:
            p.disconnect(physicsClientId=client)

        assert os.path.isfile(tmp_path / 'walk.mp4')
