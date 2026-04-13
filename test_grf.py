"""Unit tests for grf._spring_damper_fz equilibrium and spring behaviour.

These tests use the NEW API (leg_ext parameter, GRAVITY_COMP constant).
They will FAIL before the fix is applied — that is expected and correct.
"""
import pytest
from grf import _spring_damper_fz, GRAVITY_COMP, Z_REST, K_SPRING


def test_grf_equilibrium():
    """At nominal leg extension (Z_REST), spring term = 0; F_z = GRAVITY_COMP only."""
    fz = _spring_damper_fz(leg_ext=Z_REST, z_dot_foot=0.0, k_spring=K_SPRING)
    assert abs(fz - GRAVITY_COMP) < 1.0  # N — within 1 N of half body-weight (39.2 N)


def test_grf_compression():
    """Leg 3 cm shorter than rest → spring adds force above gravity comp."""
    fz = _spring_damper_fz(leg_ext=Z_REST - 0.03, z_dot_foot=0.0, k_spring=K_SPRING)
    assert fz > GRAVITY_COMP


def test_grf_extension():
    """Leg 3 cm longer than rest → spring subtracts from gravity comp."""
    fz = _spring_damper_fz(leg_ext=Z_REST + 0.03, z_dot_foot=0.0, k_spring=K_SPRING)
    assert fz < GRAVITY_COMP
