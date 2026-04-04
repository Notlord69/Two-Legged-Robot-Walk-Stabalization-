"""Tests for Siclo1 kinematics — TDD, no PyBullet required.

Kinematics imports are deferred into each test so pytest can collect this file
before kinematics.py and sim/interface.py exist.
"""
import math
import time
import xml.etree.ElementTree as ET
import pytest

URDF_PATH = "/home/notlord/ros2_ws/Siclo1_V1/Siclo1.urdf"
L_THIGH_CANONICAL = 0.060661  # m, Left leg reference
L_SHANK_CANONICAL = 0.686961  # m, Left leg reference
TOL = 1e-4                    # m, acceptable symmetry tolerance


# ── URDF symmetry tests ──────────────────────────────────────────────────────

def _joint_origin_norm(joint_name: str) -> float:
    tree = ET.parse(URDF_PATH)
    for joint in tree.getroot().iter("joint"):
        # Skip <joint> children inside <transmission> — they have no <origin>
        if joint.get("type") is None:
            continue
        if joint.attrib["name"] == joint_name:
            xyz = [float(v) for v in joint.find("origin").attrib["xyz"].split()]
            return math.sqrt(sum(v**2 for v in xyz))
    raise KeyError(joint_name)


def test_left_thigh_length():
    assert abs(_joint_origin_norm("Left_Knee") - L_THIGH_CANONICAL) < TOL


def test_left_shank_length():
    assert abs(_joint_origin_norm("Left_Ankle") - L_SHANK_CANONICAL) < TOL


def test_right_thigh_matches_left():
    assert abs(_joint_origin_norm("Right_Knee") - L_THIGH_CANONICAL) < TOL


def test_right_shank_matches_left():
    assert abs(_joint_origin_norm("Right_Ankle") - L_SHANK_CANONICAL) < TOL


# ── sim/interface.py tests ───────────────────────────────────────────────────

def test_get_joint_limits_contains_all_ten_joints():
    from sim.interface import get_joint_limits
    limits = get_joint_limits()
    expected = {
        "Left_Hip_Twist", "Left_Hip_Inwards", "Left_Hip_Forwards",
        "Left_Knee", "Left_Ankle",
        "Right_Hip_Twist", "Right_Hip_Inwards", "Right_Hip_Fowards",
        "Right_Knee", "Right_Ankle",
    }
    assert expected.issubset(limits.keys())


def test_get_joint_limits_left_knee_symmetric():
    from sim.interface import get_joint_limits
    lim = get_joint_limits()["Left_Knee"]
    assert abs(lim["lower"] + 1.570796) < 1e-4
    assert abs(lim["upper"] - 1.570796) < 1e-4


def test_get_segment_lengths_left_canonical():
    from sim.interface import get_segment_lengths
    segs = get_segment_lengths()
    assert abs(segs["left"]["thigh"] - L_THIGH_CANONICAL) < TOL
    assert abs(segs["left"]["shank"] - L_SHANK_CANONICAL) < TOL


def test_get_segment_lengths_right_matches_left_after_patch():
    from sim.interface import get_segment_lengths
    segs = get_segment_lengths()
    assert abs(segs["right"]["thigh"] - segs["left"]["thigh"]) < TOL
    assert abs(segs["right"]["shank"] - segs["left"]["shank"]) < TOL


# ── kinematics.py — clamp_foot_target ───────────────────────────────────────

class TestClampFootTarget:
    def test_inside_annulus_unchanged(self):
        from kinematics import R_MIN, R_MAX, clamp_foot_target
        d_mid = (R_MIN + R_MAX) / 2.0
        x_in, z_in = clamp_foot_target(0.0, -d_mid)
        assert abs(x_in - 0.0) < 1e-9
        assert abs(z_in - (-d_mid)) < 1e-9

    def test_below_r_min_clamped_to_r_min(self):
        from kinematics import R_MIN, clamp_foot_target
        x_c, z_c = clamp_foot_target(0.0, -(R_MIN - 0.01))
        d = math.sqrt(x_c**2 + z_c**2)
        assert abs(d - R_MIN) < 1e-6

    def test_above_r_max_clamped_to_r_max(self):
        from kinematics import R_MAX, clamp_foot_target
        x_c, z_c = clamp_foot_target(0.0, -(R_MAX + 0.05))
        d = math.sqrt(x_c**2 + z_c**2)
        assert abs(d - R_MAX) < 1e-6

    def test_direction_preserved_after_clamp(self):
        from kinematics import R_MAX, clamp_foot_target
        x_raw, z_raw = 0.1, -(R_MAX + 0.1)
        x_c, z_c = clamp_foot_target(x_raw, z_raw)
        assert abs(x_c / z_c - x_raw / z_raw) < 1e-6

    def test_at_r_min_boundary_unchanged(self):
        from kinematics import R_MIN, clamp_foot_target
        x_c, z_c = clamp_foot_target(0.0, -R_MIN)
        assert abs(math.sqrt(x_c**2 + z_c**2) - R_MIN) < 1e-9

    def test_at_r_max_boundary_unchanged(self):
        from kinematics import R_MAX, clamp_foot_target
        x_c, z_c = clamp_foot_target(0.0, -R_MAX)
        assert abs(math.sqrt(x_c**2 + z_c**2) - R_MAX) < 1e-9

    def test_degenerate_zero_input_returns_r_min_down(self):
        from kinematics import R_MIN, clamp_foot_target
        x_c, z_c = clamp_foot_target(0.0, 0.0)
        assert abs(x_c) < 1e-9
        assert abs(z_c - (-R_MIN)) < 1e-9


# ── kinematics.py — solve_ik ─────────────────────────────────────────────────

class TestSolveIK:
    def _assert_within_urdf_limits(self, hip, knee, ankle, side):
        from sim.interface import get_joint_limits
        limits = get_joint_limits()
        hip_name   = "Left_Hip_Forwards" if side == "left" else "Right_Hip_Fowards"
        knee_name  = "Left_Knee"         if side == "left" else "Right_Knee"
        ankle_name = "Left_Ankle"        if side == "left" else "Right_Ankle"
        assert limits[hip_name]["lower"]   <= hip   <= limits[hip_name]["upper"],   f"hip {hip:.4f} out of range"
        assert limits[knee_name]["lower"]  <= knee  <= limits[knee_name]["upper"],  f"knee {knee:.4f} out of range"
        assert limits[ankle_name]["lower"] <= ankle <= limits[ankle_name]["upper"], f"ankle {ankle:.4f} out of range"

    def test_foot_directly_below_hip_left(self):
        # z=-0.72 gives d=0.72 > sqrt(L_THIGH²+L_SHANK²)=0.6897 — knee within ±π/2
        from kinematics import solve_ik
        hip, knee, ankle = solve_ik((0.0, 0.0, -0.72), "left")
        self._assert_within_urdf_limits(hip, knee, ankle, "left")

    def test_foot_directly_below_hip_right(self):
        from kinematics import solve_ik
        hip, knee, ankle = solve_ik((0.0, 0.0, -0.72), "right")
        self._assert_within_urdf_limits(hip, knee, ankle, "right")

    def test_left_and_right_hip_signs_opposite_for_same_target(self):
        # Same foot target → left and right URDF-signed hip angles are opposite
        from kinematics import solve_ik
        target = (0.05, 0.0, -0.72)
        hip_l, knee_l, _ = solve_ik(target, "left")
        hip_r, knee_r, _ = solve_ik(target, "right")
        assert hip_l * hip_r < 0 or (abs(hip_l) < 1e-9 and abs(hip_r) < 1e-9)
        assert knee_l * knee_r < 0 or (abs(knee_l) < 1e-9 and abs(knee_r) < 1e-9)

    def test_ankle_always_zero(self):
        from kinematics import solve_ik
        for side in ("left", "right"):
            _, _, ankle = solve_ik((0.05, 0.0, -0.68), side)
            assert ankle == 0.0

    def test_out_of_workspace_target_clamped_not_error(self):
        from kinematics import solve_ik
        hip, knee, ankle = solve_ik((0.0, 0.0, -1.5), "left")
        assert isinstance(hip, float)
        assert isinstance(knee, float)

    def test_within_urdf_limits_left_forward_target(self):
        # z=-0.72 ensures d > 0.6897 m so knee stays within ±π/2
        from kinematics import solve_ik
        hip, knee, ankle = solve_ik((0.05, 0.0, -0.72), "left")
        self._assert_within_urdf_limits(hip, knee, ankle, "left")

    def test_within_urdf_limits_right_forward_target(self):
        from kinematics import solve_ik
        hip, knee, ankle = solve_ik((0.05, 0.0, -0.72), "right")
        self._assert_within_urdf_limits(hip, knee, ankle, "right")

    def test_invalid_side_raises(self):
        from kinematics import solve_ik
        with pytest.raises(ValueError):
            solve_ik((0.0, 0.0, -0.68), "center")


# ── kinematics.py — swing_trajectory ────────────────────────────────────────

class TestSwingTrajectory:
    def test_x_at_phi_zero_equals_x_start(self):
        from kinematics import swing_trajectory
        x, z = swing_trajectory(0.0, 0.1, 0.3, 0.04)
        assert abs(x - 0.1) < 1e-9

    def test_x_at_phi_one_equals_x_end(self):
        from kinematics import swing_trajectory
        x, z = swing_trajectory(1.0, 0.1, 0.3, 0.04)
        assert abs(x - 0.3) < 1e-9

    def test_z_at_phi_zero_is_zero(self):
        from kinematics import swing_trajectory
        _, z = swing_trajectory(0.0, 0.0, 0.1, 0.04)
        assert abs(z) < 1e-9

    def test_z_at_phi_one_is_zero(self):
        from kinematics import swing_trajectory
        _, z = swing_trajectory(1.0, 0.0, 0.1, 0.04)
        assert abs(z) < 1e-9

    def test_z_at_phi_half_equals_H(self):
        from kinematics import swing_trajectory
        H = 0.04
        _, z = swing_trajectory(0.5, 0.0, 0.1, H)
        assert abs(z - H) < 1e-9

    def test_zero_velocity_at_liftoff(self):
        from kinematics import swing_trajectory
        eps = 1e-7
        x0, z0 = swing_trajectory(0.0, 0.0, 0.1, 0.04)
        x1, z1 = swing_trajectory(eps, 0.0, 0.1, 0.04)
        assert abs((x1 - x0) / eps) < 1e-4
        assert abs((z1 - z0) / eps) < 1e-4

    def test_zero_velocity_at_touchdown(self):
        from kinematics import swing_trajectory
        eps = 1e-7
        x0, z0 = swing_trajectory(1.0 - eps, 0.0, 0.1, 0.04)
        x1, z1 = swing_trajectory(1.0, 0.0, 0.1, 0.04)
        assert abs((x1 - x0) / eps) < 1e-4
        assert abs((z1 - z0) / eps) < 1e-4

    def test_phi_out_of_range_raises(self):
        from kinematics import swing_trajectory
        with pytest.raises(ValueError):
            swing_trajectory(-0.01, 0.0, 0.1, 0.04)
        with pytest.raises(ValueError):
            swing_trajectory(1.01, 0.0, 0.1, 0.04)


# ── kinematics.py — angular_momentum_correction ──────────────────────────────

class TestAngularMomentumCorrection:
    def test_zero_hip_deviation_gives_zero_correction(self):
        from kinematics import angular_momentum_correction
        assert angular_momentum_correction(0.0, 2.0, 8.0) == 0.0

    def test_positive_hip_deviation_gives_negative_correction(self):
        from kinematics import angular_momentum_correction
        corr = angular_momentum_correction(0.3, 2.0, 8.0)
        assert corr < 0.0

    def test_formula_matches_expected_value(self):
        from kinematics import angular_momentum_correction
        corr = angular_momentum_correction(0.5, 2.0, 8.0)
        expected = -(2.0 / 8.0) * 0.5
        assert abs(corr - expected) < 1e-9

    def test_zero_m_total_raises(self):
        from kinematics import angular_momentum_correction
        with pytest.raises(ValueError):
            angular_momentum_correction(0.1, 2.0, 0.0)


# ── Timing guard ─────────────────────────────────────────────────────────────

def test_solve_ik_under_2ms_per_call():
    """solve_ik must complete in < 2 ms — 100 Hz loop budget constraint."""
    from kinematics import solve_ik
    target = (0.05, 0.0, -0.72)
    N = 1000
    start = time.perf_counter()
    for _ in range(N):
        solve_ik(target, "left")
    elapsed_s = time.perf_counter() - start
    avg_ms = (elapsed_s / N) * 1000.0
    assert avg_ms < 2.0, f"solve_ik averaged {avg_ms:.3f} ms — exceeds 2 ms budget"
