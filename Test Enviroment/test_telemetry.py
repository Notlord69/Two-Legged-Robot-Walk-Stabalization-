"""Tests for telemetry.py — SessionLogger and TelemetryThread."""
import os
import numpy as np
import pytest
from telemetry import SessionLogger, CSV_HEADER


def test_session_logger_creates_folder(tmp_path):
    logger = SessionLogger(str(tmp_path))
    assert os.path.isdir(logger.session_path)
    assert 'sessions' in logger.session_path


def test_session_logger_writes_csv_header(tmp_path):
    logger = SessionLogger(str(tmp_path))
    logger._csv.flush()
    csv_path = os.path.join(logger.session_path, "telemetry.csv")
    with open(csv_path) as f:
        first_line = f.readline().strip()
    assert first_line == CSV_HEADER


def test_session_logger_append_row_writes_data(tmp_path):
    logger = SessionLogger(str(tmp_path))
    row = np.zeros(16, dtype=np.float64)
    row[0] = 1.23     # timestamp_s
    row[1] = 7.0      # cycle
    row[12] = 8500.0  # compute_us
    logger.append_row(row)
    logger._csv.flush()
    csv_path = os.path.join(logger.session_path, "telemetry.csv")
    with open(csv_path) as f:
        lines = f.readlines()
    assert len(lines) == 2  # header + 1 data row
    assert '1.23' in lines[1]
    assert '7' in lines[1]
    assert '8500' in lines[1]


def test_session_logger_session_path_is_string(tmp_path):
    logger = SessionLogger(str(tmp_path))
    assert isinstance(logger.session_path, str)


def test_write_summary_pass(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.005, 'std_dt': 0.0003, 'min_dt': 0.003, 'max_dt': 0.009,
        'jitter_ms': 0.3, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 1000, 'analyzed_cycles': 950,
        'warmup_cycles': 50, 'coded_errors': 0,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'PASS' in content
    assert 'Total cycles   : 1000' in content
    assert 'Analyzed cycles: 950' in content
    assert logger._csv is None  # closed by write_summary


def test_write_summary_fail(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.012, 'std_dt': 0.002, 'min_dt': 0.008, 'max_dt': 0.020,
        'jitter_ms': 2.0, 'violations': 10, 'violation_rate': 0.011,
        'total_cycles': 950, 'analyzed_cycles': 900,
        'warmup_cycles': 50, 'coded_errors': 2,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'FAIL' in content
    assert 'Violations   : 10' in content
    assert 'Coded errors in ring: 2' in content


def test_write_summary_zero_analyzed_cycles(tmp_path):
    logger = SessionLogger(str(tmp_path))
    stats = {
        'mean_dt': 0.0, 'std_dt': 0.0, 'min_dt': 0.0, 'max_dt': 0.0,
        'jitter_ms': 0.0, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 30, 'analyzed_cycles': 0,
        'warmup_cycles': 50, 'coded_errors': 0,
    }
    logger.write_summary(stats)
    content = open(os.path.join(logger.session_path, "summary.txt")).read()
    assert 'No analyzed cycles' in content
    assert 'Total cycles   : 30' in content
    assert logger._csv is None


def test_session_logger_oserror_sets_csv_none(monkeypatch, tmp_path):
    import telemetry as _tel

    def _raise(*a, **k):
        raise OSError("disk full")

    monkeypatch.setattr(_tel.os, 'makedirs', _raise)
    logger = SessionLogger(str(tmp_path))
    assert logger._csv is None


def test_session_logger_append_row_noop_when_csv_none(monkeypatch, tmp_path):
    import telemetry as _tel

    def _raise(*a, **k):
        raise OSError()

    monkeypatch.setattr(_tel.os, 'makedirs', _raise)
    logger = SessionLogger(str(tmp_path))
    row = np.zeros(16, dtype=np.float64)
    logger.append_row(row)  # must not raise


def test_session_logger_write_summary_noop_when_folder_missing(monkeypatch, tmp_path):
    import telemetry as _tel

    def _raise(*a, **k):
        raise OSError()

    monkeypatch.setattr(_tel.os, 'makedirs', _raise)
    logger = SessionLogger(str(tmp_path))
    # write_summary must return silently, not raise FileNotFoundError
    logger.write_summary({
        'mean_dt': 0.0, 'std_dt': 0.0, 'min_dt': 0.0, 'max_dt': 0.0,
        'jitter_ms': 0.0, 'violations': 0, 'violation_rate': 0.0,
        'total_cycles': 0, 'analyzed_cycles': 0,
        'warmup_cycles': 50, 'coded_errors': 0,
    })
    # reaching here without raising is the assertion
    assert logger._csv is None  # handle must remain None in degraded mode
