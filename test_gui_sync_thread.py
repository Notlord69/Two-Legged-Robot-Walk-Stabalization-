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


# --------------------------------------------------------------------------- #
class TestGUISyncThreadMP4Lifecycle:

    def test_start_logging_called_on_ramp_entry(self):
        """startStateLogging fires exactly once on IDLE → RAMP."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            mock_p.startStateLogging.return_value = 42
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.IDLE
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_called_once_with(
                1, t._video_path, physicsClientId=5
            )
            assert t._log_id == 42

    def test_start_logging_not_called_when_already_ramp(self):
        """No duplicate startStateLogging if already in RAMP."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.RAMP   # already in RAMP
            t._log_id = 10                               # already recording
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_not_called()

    def test_stop_logging_called_on_idle_reentry(self):
        """stopStateLogging fires on STOP → IDLE."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 42
            t._prev_mission_state = MissionState.STOP
            snap = _make_snapshot(MissionState.IDLE)
            t._handle_mp4_lifecycle(snap)

            mock_p.stopStateLogging.assert_called_once_with(42, physicsClientId=5)
            assert t._log_id is None

    def test_stop_logging_called_on_emergency(self):
        """stopStateLogging fires immediately when emergency_stop=True."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 77
            t._prev_mission_state = MissionState.WALK
            snap = _make_snapshot(MissionState.WALK, emergency=True)
            t._handle_mp4_lifecycle(snap)

            mock_p.stopStateLogging.assert_called_once_with(77, physicsClientId=5)
            assert t._log_id is None

    def test_no_recording_when_walk_inactive(self):
        """No startStateLogging when walk_active=False."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.STATE_LOGGING_VIDEO_MP4 = 1
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=False, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._prev_mission_state = MissionState.IDLE
            snap = _make_snapshot(MissionState.RAMP)
            t._handle_mp4_lifecycle(snap)

            mock_p.startStateLogging.assert_not_called()

    def test_stop_safety_net_closes_open_log(self):
        """stop() calls stopStateLogging if _log_id is still open."""
        with patch('HeartBeat.p') as mock_p:
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=5, gui_robot_id=1,
                joint_list=[], session_path='/tmp/ts', viz_fps=30,
                walk_active=True, visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t._log_id = 99
            t.stop()
            mock_p.stopStateLogging.assert_called_once_with(99, physicsClientId=5)
            assert t._log_id is None
