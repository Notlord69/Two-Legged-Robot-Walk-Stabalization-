"""Tests for VizBridge — shared_memory layout and push_pose behaviour.
Lifecycle tests (start/stop/is_alive) are in Task 4 below.
"""
import numpy as np
import pytest
import time


JOINT_NAMES = [
    'Left_Hip_Forwards', 'Left_Knee', 'Left_Ankle',
    'Right_Hip_Fowards', 'Right_Knee', 'Right_Ankle',
]
N = len(JOINT_NAMES)
URDF_PATH = '/fake/Siclo1.urdf'


@pytest.fixture
def bridge():
    from sim.viz_bridge import VizBridge
    b = VizBridge(joint_names=JOINT_NAMES, urdf_path=URDF_PATH)
    yield b
    b.stop()


class TestSharedMemoryLayout:
    def test_buffer_size(self, bridge):
        """Buffer must be exactly (1 + 3 + 4 + N) × 8 bytes."""
        assert bridge._shm.size == (1 + 3 + 4 + N) * 8

    def test_initial_seq_is_negative(self, bridge):
        """seq_num = -1 signals 'not ready' before start()."""
        assert bridge._arr[0] == -1.0

    def test_initial_active_false(self, bridge):
        assert bridge._active is False


class TestPushPose:
    def test_increments_seq(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[0] == pytest.approx(1.0)

    def test_writes_base_position(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((1.1, 2.2, 3.3), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[1] == pytest.approx(1.1)
        assert bridge._arr[2] == pytest.approx(2.2)
        assert bridge._arr[3] == pytest.approx(3.3)

    def test_writes_base_orientation(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        bridge.push_pose((0.0, 0.0, 0.0), (0.1, 0.2, 0.3, 0.9),
                         {j: 0.0 for j in JOINT_NAMES})
        assert bridge._arr[4] == pytest.approx(0.1)
        assert bridge._arr[5] == pytest.approx(0.2)
        assert bridge._arr[6] == pytest.approx(0.3)
        assert bridge._arr[7] == pytest.approx(0.9)

    def test_writes_joints_in_order(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        joints = {j: float(i) * 0.1 for i, j in enumerate(JOINT_NAMES)}
        bridge.push_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), joints)
        for i, jname in enumerate(JOINT_NAMES):
            assert bridge._arr[8 + i] == pytest.approx(joints[jname])

    def test_seq_incremented_last(self, bridge):
        """seq_num must be written after joint data (write-barrier semantic)."""
        bridge._active = True
        bridge._arr[0] = 5.0
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {'Left_Hip_Forwards': 0.5})
        assert bridge._arr[0] == pytest.approx(6.0)  # incremented by 1

    def test_noop_when_inactive(self, bridge):
        """push_pose must not modify buffer when _active is False."""
        bridge._active = False
        bridge.push_pose((9.9, 9.9, 9.9), (9.9, 9.9, 9.9, 9.9), {})
        assert bridge._arr[0] == -1.0   # unchanged
        assert bridge._arr[1] == 0.0    # unchanged

    def test_nonblocking(self, bridge):
        bridge._active = True
        bridge._arr[0] = 0.0
        t0 = time.perf_counter()
        bridge.push_pose((0.0, 0.0, 0.88), (0.0, 0.0, 0.0, 1.0),
                         {j: 0.0 for j in JOINT_NAMES})
        assert time.perf_counter() - t0 < 0.001  # < 1 ms
