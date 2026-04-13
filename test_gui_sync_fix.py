"""Tests for GUISyncThread mutex-contention fix."""
from unittest.mock import MagicMock, patch
import time


def test_gui_sync_thread_never_calls_step_simulation():
    from HeartBeat import GUISyncThread, PoseSnapshot, MissionState

    mock_p = MagicMock()

    with patch('HeartBeat.p', mock_p):
        thread = GUISyncThread(
            gui_client=0,
            gui_robot_id=0,
            joint_list=[],
            viz_fps=200,
            visualizer=None,
            left_hip_link='left',
            right_hip_link='right',
        )
        snap = PoseSnapshot(
            base_pos=(0.0, 0.0, 0.88),
            base_orn=(0.0, 0.0, 0.0, 1.0),
            joint_states={},
            link_positions={},
            mission_state=MissionState.IDLE,
            emergency_stop=False,
        )
        thread.push_pose(snap)
        thread.start()
        time.sleep(0.05)
        thread.stop()
        thread.join(timeout=1.0)

    mock_p.stepSimulation.assert_not_called()


def test_gui_sync_fps_constant_exists_and_is_reasonable():
    import HeartBeat
    assert hasattr(HeartBeat, 'GUI_SYNC_FPS')
    assert 1 <= HeartBeat.GUI_SYNC_FPS <= 30
