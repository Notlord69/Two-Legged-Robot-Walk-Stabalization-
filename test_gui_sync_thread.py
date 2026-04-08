"""Tests for PoseSnapshot and GUISyncThread in HeartBeat.py."""
import threading
import time
import pytest
from unittest.mock import patch, MagicMock

from shared_state import MissionState


def _make_snapshot(mission_state=MissionState.IDLE, emergency=False):
    from HeartBeat import PoseSnapshot
    return PoseSnapshot(
        base_pos=(0.0, 0.0, 0.88),
        base_orn=(0.0, 0.0, 0.0, 1.0),
        joint_states={'Left_Hip_Forwards': (0.1, 0.0)},
        link_positions={'Left_Upper_Leg_1': [0.0, 0.0, 0.5]},
        mission_state=mission_state,
        emergency_stop=emergency,
    )


def _make_thread(walk_active=False, viz_fps=30, session_path='/tmp/test_session'):
    from HeartBeat import GUISyncThread
    with patch('HeartBeat.p'):
        t = GUISyncThread(
            gui_client=5,
            gui_robot_id=1,
            joint_list=[('Left_Hip_Forwards', 0)],
            session_path=session_path,
            viz_fps=viz_fps,
            walk_active=walk_active,
            visualizer=None,
            left_hip_link='Left_Upper_Leg_1',
            right_hip_link='Right_Upper_Leg_1',
        )
    return t


# --------------------------------------------------------------------------- #
class TestPoseSnapshot:
    def test_fields_accessible(self):
        snap = _make_snapshot()
        assert snap.base_pos == (0.0, 0.0, 0.88)
        assert snap.base_orn == (0.0, 0.0, 0.0, 1.0)
        assert snap.joint_states == {'Left_Hip_Forwards': (0.1, 0.0)}
        assert snap.mission_state == MissionState.IDLE
        assert snap.emergency_stop is False

    def test_ramp_state(self):
        snap = _make_snapshot(MissionState.RAMP)
        assert snap.mission_state == MissionState.RAMP

    def test_emergency_flag(self):
        snap = _make_snapshot(emergency=True)
        assert snap.emergency_stop is True


# --------------------------------------------------------------------------- #
class TestGUISyncThreadSlot:
    def test_initial_frame_count_is_zero(self):
        t = _make_thread()
        assert t.video_frame_count == 0

    def test_initial_slot_is_none(self):
        t = _make_thread()
        assert t._slot is None

    def test_push_pose_updates_slot(self):
        t = _make_thread()
        snap = _make_snapshot(MissionState.WALK)
        t.push_pose(snap)
        assert t._slot is snap

    def test_push_pose_skips_when_lock_held(self):
        t = _make_thread()
        original = _make_snapshot(MissionState.IDLE)
        t._slot = original
        t._lock.acquire()
        try:
            new_snap = _make_snapshot(MissionState.WALK)
            t.push_pose(new_snap)   # lock busy — must skip
        finally:
            t._lock.release()
        assert t._slot is original  # unchanged

    def test_push_pose_overwrites_when_lock_free(self):
        t = _make_thread()
        snap1 = _make_snapshot(MissionState.IDLE)
        snap2 = _make_snapshot(MissionState.WALK)
        t.push_pose(snap1)
        t.push_pose(snap2)
        assert t._slot is snap2

    def test_stop_returns_metadata_no_walk(self):
        t = _make_thread(walk_active=False)
        meta = t.stop()
        assert meta['video_path'] is None
        assert meta['video_fps'] == 30
        assert meta['video_frames'] == 0

    def test_stop_returns_video_path_when_walk(self, tmp_path):
        t = _make_thread(walk_active=True, session_path=str(tmp_path))
        meta = t.stop()
        assert meta['video_path'] is not None
        assert meta['video_path'].endswith('walk.mp4')
        assert meta['video_fps'] == 30
        assert meta['video_frames'] == 0
