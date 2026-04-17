"""Tests for viz/gui_worker.py.

_init_pybullet and _render_loop are tested with mocked pybullet.
The infinite loop is exited via StopIteration injected into time.sleep.
"""
import numpy as np
import pytest
from unittest.mock import patch, MagicMock, call
from multiprocessing.shared_memory import SharedMemory


def _make_shm(n_joints: int):
    """Create shared memory + numpy view with sentinel seq_num=-1."""
    n_floats = 1 + 3 + 4 + n_joints
    shm = SharedMemory(create=True, size=n_floats * 8)
    arr = np.ndarray((n_floats,), dtype=np.float64, buffer=shm.buf)
    arr[:] = 0.0
    arr[0] = -1.0
    return shm, arr


class TestInitPybullet:
    def test_sets_ready_flag(self):
        """After _init_pybullet, arr[0] must be 0.0 (ready signal)."""
        joint_names = ['Left_Hip_Forwards', 'Left_Knee']
        shm, arr = _make_shm(len(joint_names))
        try:
            with patch('viz.gui_worker.p') as mock_p:
                mock_p.connect.return_value = 0
                mock_p.GUI = 1
                mock_p.URDF_USE_INERTIA_FROM_FILE = 0
                mock_p.COV_ENABLE_GUI = 0
                mock_p.loadURDF.return_value = 42
                mock_p.getNumJoints.return_value = 2
                mock_p.getJointInfo.side_effect = [
                    (0, b'Left_Hip_Forwards', *([None] * 10), b'link_a', None),
                    (1, b'Left_Knee',         *([None] * 10), b'link_b', None),
                ]

                from viz.gui_worker import _init_pybullet
                client, robot_id, joint_ids, period = _init_pybullet(
                    shm.name, len(joint_names), joint_names,
                    '/fake/Siclo1.urdf', 30, 0.8806,
                )

            assert arr[0] == 0.0  # ready signal set
            assert robot_id == 42
            assert joint_ids == {'Left_Hip_Forwards': 0, 'Left_Knee': 1}
            assert period == pytest.approx(1.0 / 30)
        finally:
            shm.close()
            shm.unlink()

    def test_builds_joint_id_map(self):
        """_init_pybullet must map joint names to pybullet joint indices."""
        joint_names = ['Left_Ankle']
        shm, arr = _make_shm(len(joint_names))
        try:
            with patch('viz.gui_worker.p') as mock_p:
                mock_p.connect.return_value = 0
                mock_p.GUI = 1
                mock_p.URDF_USE_INERTIA_FROM_FILE = 0
                mock_p.COV_ENABLE_GUI = 0
                mock_p.loadURDF.return_value = 1
                mock_p.getNumJoints.return_value = 1
                mock_p.getJointInfo.return_value = (
                    0, b'Left_Ankle', *([None] * 10), b'link_c', None
                )
                from viz.gui_worker import _init_pybullet
                _, _, joint_ids, _ = _init_pybullet(
                    shm.name, 1, joint_names, '/fake/Siclo1.urdf', 30, 0.8806,
                )
            assert joint_ids == {'Left_Ankle': 0}
        finally:
            shm.close()
            shm.unlink()


class TestRenderLoop:
    def test_applies_pose_when_seq_changes(self):
        joint_names = ['Left_Hip_Forwards']
        shm, arr = _make_shm(1)
        try:
            arr[0] = 1.0   # seq_num = 1 (new data; last_seq starts at -1)
            arr[1], arr[2], arr[3] = 0.1, 0.2, 0.88
            arr[4], arr[5], arr[6], arr[7] = 0.0, 0.0, 0.0, 1.0
            arr[8] = 0.5   # Left_Hip_Forwards angle

            with patch('viz.gui_worker.p') as mock_p, \
                 patch('viz.gui_worker.time') as mock_time:
                mock_time.perf_counter.return_value = 0.0
                mock_time.sleep.side_effect = StopIteration  # exit after 1 iter

                from viz.gui_worker import _render_loop
                try:
                    _render_loop(0, 1, {'Left_Hip_Forwards': 0},
                                 arr, 1.0 / 30, joint_names, 1)
                except StopIteration:
                    pass

            mock_p.resetBasePositionAndOrientation.assert_called_once_with(
                1, [0.1, 0.2, 0.88], [0.0, 0.0, 0.0, 1.0], physicsClientId=0
            )
            mock_p.resetJointState.assert_called_once_with(
                1, 0, 0.5, 0.0, physicsClientId=0
            )
            mock_p.stepSimulation.assert_called_once_with(physicsClientId=0)
        finally:
            shm.close()
            shm.unlink()

    def test_skips_frame_when_seq_unchanged(self):
        """After seq_num is consumed in iter 1, iter 2 must not call reset*.

        Two iterations: first applies (seq 1 != last_seq -1), second skips
        (seq 1 == last_seq 1). resetBasePositionAndOrientation called once total.
        """
        joint_names = ['Left_Hip_Forwards']
        shm, arr = _make_shm(1)
        try:
            arr[0] = 1.0   # seq_num = 1; never changes between iterations
            arr[1], arr[2], arr[3] = 0.1, 0.2, 0.88
            arr[4], arr[5], arr[6], arr[7] = 0.0, 0.0, 0.0, 1.0
            arr[8] = 0.5

            sleep_calls = [0]
            def counting_sleep(t):
                sleep_calls[0] += 1
                if sleep_calls[0] >= 2:
                    raise StopIteration  # exit after 2 iterations

            with patch('viz.gui_worker.p') as mock_p, \
                 patch('viz.gui_worker.time') as mock_time:
                mock_time.perf_counter.return_value = 0.0
                mock_time.sleep.side_effect = counting_sleep

                from viz.gui_worker import _render_loop
                try:
                    _render_loop(0, 1, {'Left_Hip_Forwards': 0},
                                 arr, 1.0 / 30, joint_names, 1)
                except StopIteration:
                    pass

            # Exactly one apply (first iter), zero on second (same seq)
            assert mock_p.resetBasePositionAndOrientation.call_count == 1
            assert mock_p.stepSimulation.call_count == 1
        finally:
            shm.close()
            shm.unlink()
