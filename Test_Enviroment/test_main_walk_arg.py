"""Tests for --walk CLI argument in main.py — no simulation required."""
import pytest


def test_walk_arg_accepted():
    """--walk 2.0 is accepted without error."""
    import main
    args = main._make_parser().parse_args(['--walk', '2.0'])
    assert args.walk == 2.0


def test_walk_arg_default_is_none():
    """Omitting --walk yields walk=None."""
    import main
    args = main._make_parser().parse_args([])
    assert args.walk is None


def test_walk_arg_combined_with_gui():
    """--gui --walk 1.5 both accepted."""
    import main
    args = main._make_parser().parse_args(['--gui', '--walk', '1.5'])
    assert args.gui is True
    assert args.walk == 1.5


def test_walk_arg_combined_with_duration():
    """--duration 500 --walk 1.0 both accepted."""
    import main
    args = main._make_parser().parse_args(['--duration', '500', '--walk', '1.0'])
    assert args.duration == 500
    assert args.walk == 1.0
