"""Tests for PoseLogger — pre-allocated pose ring buffer."""
import os
import tempfile
import numpy as np
import pytest

_SAMPLE_JOINTS = {
    'Left_Hip_Forwards': 0.5,
    'Left_Knee':         0.6,
    'Left_Ankle':        0.7,
    'Right_Hip_Fowards': 0.8,
    'Right_Knee':        0.9,
    'Right_Ankle':       1.0,
}


def test_record_fills_correct_columns():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=10)
    logger.record(
        sim_time=0.01,
        base_pos=(1.0, 2.0, 3.0),
        base_orn=(0.1, 0.2, 0.3, 0.9),
        joint_positions=_SAMPLE_JOINTS,
        left_force=45.0,
        right_force=50.0,
        stability_status=1,
        mission_state=2,
    )
    row = logger._buf[0]
    assert row[0]  == pytest.approx(0.01)
    assert row[1]  == pytest.approx(1.0)
    assert row[2]  == pytest.approx(2.0)
    assert row[3]  == pytest.approx(3.0)
    assert row[4]  == pytest.approx(0.1)
    assert row[5]  == pytest.approx(0.2)
    assert row[6]  == pytest.approx(0.3)
    assert row[7]  == pytest.approx(0.9)
    assert row[8]  == pytest.approx(0.5)
    assert row[9]  == pytest.approx(0.6)
    assert row[10] == pytest.approx(0.7)
    assert row[11] == pytest.approx(0.8)
    assert row[12] == pytest.approx(0.9)
    assert row[13] == pytest.approx(1.0)
    assert row[14] == pytest.approx(45.0)
    assert row[15] == pytest.approx(50.0)
    assert row[16] == pytest.approx(1.0)
    assert row[17] == pytest.approx(2.0)
    assert row[18] == pytest.approx(0.0)


def test_record_handles_missing_joints_gracefully():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=1)
    logger.record(
        sim_time=0.0, base_pos=(0, 0, 0), base_orn=(0, 0, 0, 1),
        joint_positions={},
        left_force=0.0, right_force=0.0, stability_status=0, mission_state=0,
    )
    row = logger._buf[0]
    for col in range(8, 14):
        assert row[col] == pytest.approx(0.0), f"col {col} should be 0.0"


def test_save_writes_npy_with_correct_shape():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=5)
    for i in range(3):
        logger.record(
            sim_time=float(i) * 0.01,
            base_pos=(0.0, 0.0, 0.88), base_orn=(0.0, 0.0, 0.0, 1.0),
            joint_positions={}, left_force=0.0, right_force=0.0,
            stability_status=0, mission_state=0,
        )
    with tempfile.TemporaryDirectory() as tmpdir:
        path = logger.save(tmpdir)
        assert os.path.isfile(path)
        loaded = np.load(path)
        assert loaded.shape == (3, 19)
        assert loaded.dtype == np.float64


def test_save_returns_poses_npy_path():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=2)
    logger.record(0.0, (0, 0, 0), (0, 0, 0, 1), {}, 0.0, 0.0, 0, 0)
    with tempfile.TemporaryDirectory() as tmpdir:
        path = logger.save(tmpdir)
        assert path == os.path.join(tmpdir, 'poses.npy')


def test_buffer_overflow_is_silent():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=2)
    for _ in range(5):
        logger.record(0.0, (0, 0, 0), (0, 0, 0, 1), {}, 0.0, 0.0, 0, 0)
    assert logger._idx == 2


def test_index_advances_per_record():
    from pose_logger import PoseLogger
    logger = PoseLogger(max_cycles=10)
    for i in range(4):
        logger.record(float(i) * 0.01, (0, 0, 0), (0, 0, 0, 1), {}, 0.0, 0.0, 0, 0)
    assert logger._idx == 4
