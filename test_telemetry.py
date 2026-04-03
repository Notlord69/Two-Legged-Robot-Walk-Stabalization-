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
