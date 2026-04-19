"""Tests for replay.py — post-run 3D session replay."""
import os
import tempfile
import numpy as np
import pytest


def _write_poses_npy(tmpdir: str, n_frames: int = 5) -> str:
    """Write a minimal poses.npy (identity pose, z=0.88)."""
    poses = np.zeros((n_frames, 19), dtype=np.float64)
    for i in range(n_frames):
        poses[i, 0] = i * 0.01   # sim_time
        poses[i, 3] = 0.8806     # base_pos.z
        poses[i, 7] = 1.0        # base_orn.qw — identity
    path = os.path.join(tmpdir, 'poses.npy')
    np.save(path, poses)
    return tmpdir


def test_replay_headless_completes_without_error():
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=3)
        from replay import replay
        replay(tmpdir, speed=1000.0, record=False, headless=True)


def test_replay_raises_file_not_found_if_poses_missing():
    with tempfile.TemporaryDirectory() as tmpdir:
        from replay import replay
        with pytest.raises(FileNotFoundError, match='poses.npy'):
            replay(tmpdir, speed=1.0, record=False, headless=True)


def test_replay_loads_correct_frame_count():
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=7)
        from replay import _load_poses
        poses = _load_poses(tmpdir)
        assert poses.shape == (7, 19)


def test_replay_speed_multiplier_accepted():
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_poses_npy(tmpdir, n_frames=2)
        from replay import replay
        for speed in (0.1, 1.0, 10.0, 500.0):
            replay(tmpdir, speed=speed, record=False, headless=True)
