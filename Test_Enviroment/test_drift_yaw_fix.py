"""Tests for Drift & Yaw Fix (2026-04-28 design spec).

Covers:
  4.1 URDF symmetry (static verification)
  4.2 Yaw stabilizer (unit)
  4.3 I-term balance (unit + integration)
"""
import math
import pytest
import numpy as np
import xml.etree.ElementTree as ET
import os

from shared_state import (
    shared_state, ContactState, StepPhase, MissionState,
    DEFAULT_LINK_DATA, URDF_JOINT_LIMITS,
)
from balance_controller import (
    update_balance, reset_balance, _controller,
    LATERAL_KI, LATERAL_I_MAX,
    SAGITTAL_KI, SAGITTAL_I_MAX,
    EMERGENCY_THRESHOLD,
)


# ============================================================================
# FIXTURES
# ============================================================================

URDF_PATH = os.path.join(os.path.dirname(__file__), '..', 'Siclo1.urdf')


@pytest.fixture
def urdf_root():
    tree = ET.parse(URDF_PATH)
    return tree.getroot()


def _reset_for_balance():
    """Configure shared_state for nominal standing."""
    shared_state.reset()
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = 1.0
    shared_state.freeze_robot = False
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.stance_side = "right"
    shared_state.com_position = np.array([0.0, 0.25, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_position = np.array([-0.1, 0.25, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.25, 0.0])
    shared_state.stance_foot_world_pos = np.array([0.1, 0.25, 0.0])
    shared_state.capture_point = np.array([0.0, 0.25])
    reset_balance()


# ============================================================================
# 4.1 URDF SYMMETRY (STATIC VERIFICATION)
# ============================================================================

def _find_joint(root, name):
    for j in root.findall('joint'):
        if j.get('name') == name:
            return j
    return None


class TestURDFSymmetry:

    def test_left_knee_origin_matches_right(self, urdf_root):
        l_knee = _find_joint(urdf_root, 'Left_Knee')
        r_knee = _find_joint(urdf_root, 'Right_Knee')
        l_xyz = [float(v) for v in l_knee.find('origin').get('xyz').split()]
        r_xyz = [float(v) for v in r_knee.find('origin').get('xyz').split()]
        assert l_xyz == r_xyz, f"Left_Knee {l_xyz} != Right_Knee {r_xyz}"

    def test_left_ankle_origin_matches_right(self, urdf_root):
        l_ankle = _find_joint(urdf_root, 'Left_Ankle')
        r_ankle = _find_joint(urdf_root, 'Right_Ankle')
        l_xyz = [float(v) for v in l_ankle.find('origin').get('xyz').split()]
        r_xyz = [float(v) for v in r_ankle.find('origin').get('xyz').split()]
        assert l_xyz == r_xyz, f"Left_Ankle {l_xyz} != Right_Ankle {r_xyz}"

    def test_hip_twist_limits_symmetric(self, urdf_root):
        l_twist = _find_joint(urdf_root, 'Left_Hip_Twist')
        r_twist = _find_joint(urdf_root, 'Right_Hip_Twist')
        l_lower = float(l_twist.find('limit').get('lower'))
        r_lower = float(r_twist.find('limit').get('lower'))
        l_upper = float(l_twist.find('limit').get('upper'))
        r_upper = float(r_twist.find('limit').get('upper'))
        assert abs(l_lower - r_lower) < 1e-6, f"lower: L={l_lower} R={r_lower}"
        assert abs(l_upper - r_upper) < 1e-6, f"upper: L={l_upper} R={r_upper}"

    def test_hip_twist_limit_value(self, urdf_root):
        r_twist = _find_joint(urdf_root, 'Right_Hip_Twist')
        lower = float(r_twist.find('limit').get('lower'))
        assert abs(lower - (-0.349066)) < 1e-6

    def test_default_link_data_symmetric_thigh(self):
        assert DEFAULT_LINK_DATA['l_thigh']['length'] == DEFAULT_LINK_DATA['r_thigh']['length']
        assert DEFAULT_LINK_DATA['l_thigh']['com_local'] == DEFAULT_LINK_DATA['r_thigh']['com_local']

    def test_default_link_data_symmetric_shank(self):
        assert DEFAULT_LINK_DATA['l_shank']['length'] == DEFAULT_LINK_DATA['r_shank']['length']
        assert DEFAULT_LINK_DATA['l_shank']['com_local'] == DEFAULT_LINK_DATA['r_shank']['com_local']

    def test_urdf_joint_limits_hip_twist_matches(self):
        assert URDF_JOINT_LIMITS['Right_Hip_Twist']['lower'] == pytest.approx(-0.349066)
        assert URDF_JOINT_LIMITS['Left_Hip_Twist']['lower'] == pytest.approx(-0.349066)

    def test_left_hip_twist_z_compensated(self, urdf_root):
        l_twist = _find_joint(urdf_root, 'Left_Hip_Twist')
        z = float(l_twist.find('origin').get('xyz').split()[2])
        assert abs(z - 1.018094) < 1e-6


# ============================================================================
# 4.2 YAW STABILIZER (UNIT)
# ============================================================================

class TestYawStabilizer:

    def test_hip_twist_excluded_from_wbc(self):
        """Hip_Twist joints must not appear in WBC joint lists."""
        from HeartBeat import _WBC_LEFT_JOINTS, _WBC_RIGHT_JOINTS
        wbc_names = {j[1] for j in _WBC_LEFT_JOINTS} | {j[1] for j in _WBC_RIGHT_JOINTS}
        assert 'Left_Hip_Twist' not in wbc_names
        assert 'Right_Hip_Twist' not in wbc_names

    def test_yaw_hold_in_apply_control(self):
        """apply_control code path includes Hip_Twist joints."""
        import inspect
        from HeartBeat import PyBulletInterface
        source = inspect.getsource(PyBulletInterface.apply_control)
        assert 'Left_Hip_Twist' in source
        assert 'Right_Hip_Twist' in source


# ============================================================================
# 4.3 I-TERM BALANCE (UNIT + INTEGRATION)
# ============================================================================

class TestBalanceITerm:

    def test_integrator_accumulates_lateral(self):
        """Constant lateral CP error for 200 cycles increases integrator beyond zero."""
        _reset_for_balance()
        shared_state.com_position = np.array([0.01, 0.25, 0.8806])
        shared_state.capture_point = np.array([0.01, 0.25])
        for _ in range(200):
            update_balance()
        assert abs(_controller._integral_lateral) > 0.0

    def test_integrator_accumulates_sagittal(self):
        """Constant sagittal CP error for 200 cycles increases integrator beyond zero."""
        _reset_for_balance()
        shared_state.com_position = np.array([0.0, 0.26, 0.8806])
        shared_state.capture_point = np.array([0.0, 0.26])
        for _ in range(200):
            update_balance()
        assert abs(_controller._integral_sagittal) > 0.0

    def test_pid_output_exceeds_pd_only(self):
        """With constant error, PID output should exceed PD-only magnitude."""
        _reset_for_balance()
        # Run PD-only baseline (single cycle — integrator near zero)
        shared_state.com_position = np.array([0.01, 0.25, 0.8806])
        shared_state.capture_point = np.array([0.01, 0.25])
        update_balance()
        pd_roll = abs(shared_state.balance_hip_roll_right)

        # Now accumulate I-term over 200 cycles
        _reset_for_balance()
        shared_state.com_position = np.array([0.01, 0.25, 0.8806])
        shared_state.capture_point = np.array([0.01, 0.25])
        for _ in range(200):
            update_balance()
        pid_roll = abs(shared_state.balance_hip_roll_right)

        assert pid_roll > pd_roll

    def test_lateral_antiwindup_clamp(self):
        """Integrator output never exceeds LATERAL_I_MAX."""
        _reset_for_balance()
        shared_state.com_position = np.array([5.0, 0.25, 0.8806])
        shared_state.capture_point = np.array([5.0, 0.25])
        for _ in range(500):
            update_balance()
        i_output = abs(LATERAL_KI * _controller._integral_lateral)
        assert i_output <= LATERAL_I_MAX + 1e-9

    def test_sagittal_antiwindup_clamp(self):
        """Integrator output never exceeds SAGITTAL_I_MAX."""
        _reset_for_balance()
        # Keep error below emergency threshold to avoid reset
        shared_state.com_position = np.array([0.0, 0.32, 0.8806])
        shared_state.capture_point = np.array([0.0, 0.32])
        for _ in range(500):
            update_balance()
        i_output = abs(SAGITTAL_KI * _controller._integral_sagittal)
        assert i_output <= SAGITTAL_I_MAX + 1e-9

    def test_integrator_unwinds_on_error_reversal(self):
        """Integrator decreases when error reverses direction."""
        _reset_for_balance()
        # Wind up positive
        shared_state.com_position = np.array([0.05, 0.25, 0.8806])
        shared_state.capture_point = np.array([0.05, 0.25])
        for _ in range(100):
            update_balance()
        wound_up = _controller._integral_lateral

        # Reverse error
        shared_state.com_position = np.array([-0.05, 0.25, 0.8806])
        shared_state.capture_point = np.array([-0.05, 0.25])
        for _ in range(100):
            update_balance()
        unwound = _controller._integral_lateral

        assert abs(unwound) < abs(wound_up)

    def test_reset_zeros_integrators(self):
        """reset() clears both integrators to zero."""
        _reset_for_balance()
        shared_state.com_position = np.array([0.05, 0.30, 0.8806])
        shared_state.capture_point = np.array([0.05, 0.30])
        for _ in range(50):
            update_balance()
        assert _controller._integral_lateral != 0.0

        reset_balance()
        assert _controller._integral_lateral == 0.0
        assert _controller._integral_sagittal == 0.0

    def test_freeze_zeros_integrators(self):
        """freeze_robot zeroes integrators via _write_zero_outputs."""
        _reset_for_balance()
        shared_state.com_position = np.array([0.05, 0.30, 0.8806])
        shared_state.capture_point = np.array([0.05, 0.30])
        for _ in range(50):
            update_balance()
        assert _controller._integral_lateral != 0.0

        shared_state.freeze_robot = True
        update_balance()
        assert _controller._integral_lateral == 0.0
        assert _controller._integral_sagittal == 0.0

    def test_emergency_resets_sagittal_integrator(self):
        """Entering emergency mode resets sagittal integrator."""
        _reset_for_balance()
        # Accumulate sagittal integrator (below emergency threshold)
        shared_state.com_position = np.array([0.0, 0.32, 0.8806])
        shared_state.capture_point = np.array([0.0, 0.32])
        for _ in range(50):
            update_balance()
        assert abs(_controller._integral_sagittal) > 0.0

        # Jump past emergency threshold
        shared_state.com_position = np.array([0.0, 0.50, 0.8806])
        shared_state.capture_point = np.array([0.0, 0.50])
        update_balance()
        assert _controller._integral_sagittal == 0.0
