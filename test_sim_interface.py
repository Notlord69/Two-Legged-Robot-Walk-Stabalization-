"""Tests for sim/interface.py debug line wrappers."""
import pytest
from unittest.mock import patch, MagicMock, call


def test_add_debug_line_calls_pybullet_add():
    with patch('pybullet.addUserDebugLine', return_value=42) as mock_add:
        from sim.interface import add_debug_line
        result = add_debug_line([0, 0, 0], [1, 1, 1], [1, 0, 0])
    assert result == 42
    mock_add.assert_called_once()


def test_add_debug_line_passes_replace_id_when_non_negative():
    with patch('pybullet.addUserDebugLine', return_value=5) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [0, 1, 0], replace_id=3)
    kwargs = mock_add.call_args[1]
    assert kwargs.get('replaceItemUniqueId') == 3


def test_add_debug_line_omits_replace_id_when_negative():
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [0, 1, 0], replace_id=-1)
    kwargs = mock_add.call_args[1]
    assert 'replaceItemUniqueId' not in kwargs


def test_remove_debug_line_calls_pybullet_remove():
    with patch('pybullet.removeUserDebugItem') as mock_remove:
        from sim.interface import remove_debug_line
        remove_debug_line(7)
    mock_remove.assert_called_once_with(7, physicsClientId=0)


def test_remove_debug_line_noop_for_negative_id():
    with patch('pybullet.removeUserDebugItem') as mock_remove:
        from sim.interface import remove_debug_line
        remove_debug_line(-1)
    mock_remove.assert_not_called()


def test_add_debug_line_passes_physics_client():
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [1, 1, 0], physics_client=3)
    kwargs = mock_add.call_args[1]
    assert kwargs['physicsClientId'] == 3
