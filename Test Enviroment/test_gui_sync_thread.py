"""Tests for PoseSnapshot and GUISyncThread in HeartBeat.py.

MP4 recording is no longer part of GUISyncThread — it moved to VideoRecorder
(recorder.py).  These tests cover the display-mirror path only.
"""
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


def _make_thread(viz_fps=30):
    from HeartBeat import GUISyncThread
    with patch('HeartBeat.p'):
        t = GUISyncThread(
            gui_client=5,
            gui_robot_id=1,
            joint_list=[('Left_Hip_Forwards', 0)],
            viz_fps=viz_fps,
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
        assert t._video_frame_count == 0

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

    def test_stop_does_not_raise(self):
        t = _make_thread()
        t.stop()   # should complete without error


# --------------------------------------------------------------------------- #
class TestGUISyncThreadRunLoop:

    def test_run_increments_frame_count(self):
        """Thread increments _video_frame_count on each render cycle."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            mock_p.stepSimulation = MagicMock()
            mock_p.resetBasePositionAndOrientation = MagicMock()
            mock_p.resetJointState = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=0, gui_robot_id=0,
                joint_list=[('Left_Hip_Forwards', 2)],
                viz_fps=60,
                visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.push_pose(_make_snapshot())
            t.start()
            time.sleep(0.15)   # allow ~9 frames at 60fps
            t.stop()
            t.join(timeout=1.0)

            assert t._video_frame_count >= 4

    def test_run_calls_step_simulation(self):
        """Thread calls p.stepSimulation(gui_client) each render cycle."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            mock_p.stepSimulation = MagicMock()
            mock_p.resetBasePositionAndOrientation = MagicMock()
            mock_p.resetJointState = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=7, gui_robot_id=1,
                joint_list=[],
                viz_fps=60,
                visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.push_pose(_make_snapshot())
            t.start()
            time.sleep(0.1)
            t.stop()
            t.join(timeout=1.0)

            calls = mock_p.stepSimulation.call_args_list
            assert any(c == ((), {'physicsClientId': 7}) for c in calls)

    def test_run_stops_cleanly_when_no_snapshot(self):
        """Thread exits cleanly even if no snapshot was ever pushed."""
        with patch('HeartBeat.p') as mock_p:
            mock_p.setRealTimeSimulation = MagicMock()
            from HeartBeat import GUISyncThread
            t = GUISyncThread(
                gui_client=0, gui_robot_id=0,
                joint_list=[], viz_fps=60,
                visualizer=None,
                left_hip_link='L', right_hip_link='R',
            )
            t.start()
            time.sleep(0.05)
            t.stop()
            t.join(timeout=1.0)
            assert not t.is_alive()
