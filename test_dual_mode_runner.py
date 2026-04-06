"""Tests for main.py argparse entry point and decimation computation."""
import pytest
from unittest.mock import patch, MagicMock
from main import _make_parser, _viz_decimation, main


# ── _viz_decimation ────────────────────────────────────────────────────────── #

def test_decimation_default_10hz():
    assert _viz_decimation(10) == 10  # 100 Hz / 10 Hz = 10 cycles/render


def test_decimation_33hz():
    assert _viz_decimation(33) == 3   # 100 // 33 = 3 cycles/render


def test_decimation_clamp_above_100():
    assert _viz_decimation(200) == 1  # clamped to 100 Hz → 100 // 100 = 1


def test_decimation_clamp_below_1():
    assert _viz_decimation(0) == 100  # clamped to 1 Hz → 100 // 1 = 100


def test_decimation_at_1hz():
    assert _viz_decimation(1) == 100  # 100 Hz / 1 Hz = 100 cycles/render


# ── argparse ──────────────────────────────────────────────────────────────── #

def test_default_no_gui():
    args = _make_parser().parse_args([])
    assert args.gui is False


def test_gui_flag():
    args = _make_parser().parse_args(["--gui"])
    assert args.gui is True


def test_default_viz_hz():
    args = _make_parser().parse_args([])
    assert args.viz_hz == 10


def test_viz_hz_parsed():
    args = _make_parser().parse_args(["--viz-hz", "25"])
    assert args.viz_hz == 25


# ── main() integration ────────────────────────────────────────────────────── #

def test_main_passes_decimation_to_controller():
    """main() converts --viz-hz to decimation and passes it to Siclo1Controller."""
    mock_ctrl = MagicMock()
    with patch('main.Siclo1Controller', return_value=mock_ctrl) as mock_cls:
        main(["--viz-hz", "20"])
    mock_cls.assert_called_once_with(use_gui=False, viz_decimation=5)
    mock_ctrl.run.assert_called_once_with(duration=30.0)
    mock_ctrl.shutdown.assert_called_once()
