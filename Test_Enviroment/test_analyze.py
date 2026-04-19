"""Tests for analyze.py — post-run telemetry plot generator."""
import os
import tempfile
import pytest

_CSV_HEADER = (
    "timestamp_s,cycle,error_code,com_x,com_y,com_z,"
    "left_contact,right_contact,stability_status,"
    "left_force_n,right_force_n,stability_margin_m,"
    "compute_us,extra_0,extra_1,extra_2"
)


def _write_telemetry_csv(tmpdir: str, n_rows: int = 10) -> str:
    rows = []
    for i in range(n_rows):
        rows.append(','.join([
            f'{i * 0.01:.6g}', f'{i}', '0',
            f'{i * 0.001:.6g}', '0.0', '0.88',
            '3', '3', '1',
            '39.0', '39.0', '0.02',
            '4500', '0', '0', '0',
        ]))
    path = os.path.join(tmpdir, 'telemetry.csv')
    with open(path, 'w') as f:
        f.write(_CSV_HEADER + '\n')
        f.write('\n'.join(rows) + '\n')
    return tmpdir


def test_analyze_creates_four_png_files():
    with tempfile.TemporaryDirectory() as tmpdir:
        _write_telemetry_csv(tmpdir)
        from analyze import analyze
        analyze(tmpdir, show=False)
        for name in ('com_trajectory.png', 'contact_forces.png',
                     'timing.png', 'stability.png'):
            assert os.path.isfile(os.path.join(tmpdir, name)), f"Missing {name}"


def test_analyze_raises_if_csv_missing():
    with tempfile.TemporaryDirectory() as tmpdir:
        from analyze import analyze
        with pytest.raises(FileNotFoundError, match='telemetry.csv'):
            analyze(tmpdir, show=False)


def test_analyze_handles_violation_rows():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, 'telemetry.csv')
        with open(path, 'w') as f:
            f.write(_CSV_HEADER + '\n')
            for err in ('0', '1', '0'):
                f.write(f'0.01,1,{err},0.0,0.0,0.88,3,3,1,39.0,39.0,0.02,5000,0,0,0\n')
        from analyze import analyze
        analyze(tmpdir, show=False)
        assert os.path.isfile(os.path.join(tmpdir, 'timing.png'))


def test_analyze_handles_single_row_csv():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, 'telemetry.csv')
        with open(path, 'w') as f:
            f.write(_CSV_HEADER + '\n')
            f.write('0.01,1,0,0.0,0.0,0.88,3,3,1,39.0,39.0,0.02,4500,0,0,0\n')
        from analyze import analyze
        analyze(tmpdir, show=False)
