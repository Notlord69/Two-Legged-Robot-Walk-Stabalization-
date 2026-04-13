# Week 3 Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `kinematics.py` (IK solver, cycloidal swing, angular momentum feedforward) with a `sim/interface.py` URDF abstraction layer, unblocked by the Right-leg symmetry patch.

**Architecture:** Left leg is the canonical segment-length reference. A URDF patch corrects the Right-leg joint origins before any IK constants are written. `sim/interface.py` owns all URDF parsing and all PyBullet calls; `kinematics.py` is pure math with no PyBullet import.

**Tech Stack:** Python 3.10+, math (stdlib), xml.etree.ElementTree (stdlib), pybullet (sim layer only), pytest

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `Siclo1.urdf` | Modify | Fix Right_Knee and Right_Ankle `<origin xyz>` |
| `urdf_change/urdf_changes.md` | Modify | Log ITER-001 and ITER-002 as APPLIED; fix R_MIN typo |
| `sim/__init__.py` | Create | Package marker (empty) |
| `sim/interface.py` | Create | All URDF parsing + all PyBullet wrappers |
| `shared_state.py` | Modify | Add kinematics output fields to `Siclo1State` |
| `kinematics.py` | Create | Pure math: clamp, IK, swing trajectory, ang. momentum |
| `test_kinematics.py` | Create | pytest unit tests for all public functions |

---

## Task 1: URDF Symmetry Patch

**Files:**
- Modify: `Siclo1.urdf:456-464`
- Modify: `urdf_change/urdf_changes.md`

### Background

Right_Knee origin `xyz="-0.106 -0.013692 -0.00859"` has X as dominant axis (−106 mm) and Z = −9 mm — a Fusion 360 frame-orientation export artifact. Correct values set Z-dominant offsets equal to the Left canonical norms. The urdf_changes.md log also has a typo: R_MIN is listed as 0.621305 m but the correct value is 0.631300 m.

- [ ] **Step 1: Write the failing URDF symmetry test**

Create `test_kinematics.py`:

```python
"""Tests for Siclo1 kinematics — TDD, no PyBullet required."""
import math
import xml.etree.ElementTree as ET
import pytest

URDF_PATH = "/home/notlord/ros2_ws/Siclo1_V1/Siclo1.urdf"
L_THIGH_CANONICAL = 0.060661  # m, Left leg reference
L_SHANK_CANONICAL = 0.686961  # m, Left leg reference
TOL = 1e-4                    # m, acceptable symmetry tolerance


def _joint_origin_norm(joint_name: str) -> float:
    tree = ET.parse(URDF_PATH)
    for joint in tree.getroot().iter("joint"):
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
```

- [ ] **Step 2: Run to confirm tests 3 and 4 fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_kinematics.py::test_right_thigh_matches_left test_kinematics.py::test_right_shank_matches_left -v
```

Expected output:
```
FAILED test_kinematics.py::test_right_thigh_matches_left
FAILED test_kinematics.py::test_right_shank_matches_left
```

- [ ] **Step 3: Apply URDF patch — Right_Knee origin**

In `Siclo1.urdf` at line 457, change:
```xml
    <origin rpy="0 0 0" xyz="-0.106 -0.013692 -0.00859"/>
```
to:
```xml
    <origin rpy="0 0 0" xyz="0.0 0.0 -0.060661"/>
```

- [ ] **Step 4: Apply URDF patch — Right_Ankle origin**

In `Siclo1.urdf` at line 464, change:
```xml
    <origin rpy="0 0 0" xyz="-0.025 0.010133 -0.758742"/>
```
to:
```xml
    <origin rpy="0 0 0" xyz="0.0 0.0 -0.686961"/>
```

- [ ] **Step 5: Update urdf_changes.md — mark APPLIED + fix R_MIN typo**

In `urdf_change/urdf_changes.md`:

1. Change `R_min = |L_thigh − L_shank| + 0.005 = 0.621305 m` → `0.631300 m`

2. Change ITER-001 status line:
   ```
   **Status:** PENDING — awaiting explicit URDF modification approval
   ```
   to:
   ```
   **Status:** APPLIED — 2026-04-04
   ```

3. Change ITER-002 status line the same way.

4. Move both ITER entries from `## Pending Changes` to `## Applied Changes`.

- [ ] **Step 6: Run all four URDF tests — confirm they pass**

```bash
pytest test_kinematics.py::test_left_thigh_length \
       test_kinematics.py::test_left_shank_length \
       test_kinematics.py::test_right_thigh_matches_left \
       test_kinematics.py::test_right_shank_matches_left -v
```

Expected: 4 passed

- [ ] **Step 7: Commit**

```bash
git add Siclo1.urdf urdf_change/urdf_changes.md test_kinematics.py
git commit -m "fix: correct Right_Knee and Right_Ankle URDF origins (ITER-001, ITER-002)

Right_Knee X-dominant origin (-0.106 m) was a Fusion 360 frame export artifact.
Both right-leg joints now use Left-canonical Z-dominant norms:
  thigh = 0.060661 m, shank = 0.686961 m. Logged in urdf_change/urdf_changes.md."
```

---

## Task 2: sim/interface.py — URDF Abstraction Layer

**Files:**
- Create: `sim/__init__.py`
- Create: `sim/interface.py`
- Modify: `test_kinematics.py`

### Background

CLAUDE.md requires all PyBullet calls to go through `sim/interface.py`. This task creates the abstraction. The URDF parsers (`get_joint_limits`, `get_segment_lengths`) are pure stdlib and fully testable without PyBullet. The PyBullet wrappers (`get_joint_state`, `set_joint_position_target`) are thin and not unit-tested (they delegate directly to `p.*`).

- [ ] **Step 1: Write the failing interface tests**

Append to `test_kinematics.py`:

```python
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
```

- [ ] **Step 2: Run to confirm they fail**

```bash
pytest test_kinematics.py -k "interface or segment or joint_limits" -v
```

Expected: 4 FAILED with `ModuleNotFoundError: No module named 'sim'`

- [ ] **Step 3: Create sim/__init__.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/sim/__init__.py` with empty content (just a newline).

- [ ] **Step 4: Create sim/interface.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/sim/interface.py`:

```python
"""PyBullet / URDF abstraction layer for Siclo1.

ALL PyBullet calls go through this module. Direct p.* calls in control logic
are prohibited. This isolation enables future Gazebo swap by changing only
this file.

URDF parsers (get_joint_limits, get_segment_lengths) are pure stdlib —
no PyBullet import required to call them.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pybullet as p

_URDF_PATH = Path(__file__).parent.parent / "Siclo1.urdf"


def get_joint_limits() -> dict[str, dict[str, float]]:
    """Parse Siclo1.urdf, return joint limits by exact URDF joint name.

    Returns: {joint_name: {'lower': float, 'upper': float}}  angles in rad.
    Includes only joints with a <limit> element (revolute joints).
    """
    tree = ET.parse(_URDF_PATH)
    limits: dict[str, dict[str, float]] = {}
    for joint in tree.getroot().iter("joint"):
        limit_el = joint.find("limit")
        if limit_el is not None:
            limits[joint.attrib["name"]] = {
                "lower": float(limit_el.attrib["lower"]),
                "upper": float(limit_el.attrib["upper"]),
            }
    return limits


def get_segment_lengths() -> dict[str, dict[str, float]]:
    """Parse Siclo1.urdf joint origins, return IK segment lengths (m).

    Segment length = Euclidean norm of joint <origin xyz=...>.
    Left leg is the canonical reference (verified and patched 2026-04-04).

    Returns:
        {'left':  {'thigh': m, 'shank': m},
         'right': {'thigh': m, 'shank': m}}
    """
    tree = ET.parse(_URDF_PATH)
    joints = {j.attrib["name"]: j for j in tree.getroot().iter("joint")}

    def _norm(name: str) -> float:
        origin = joints[name].find("origin")
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        return math.sqrt(sum(v * v for v in xyz))

    return {
        "left":  {"thigh": _norm("Left_Knee"),  "shank": _norm("Left_Ankle")},
        "right": {"thigh": _norm("Right_Knee"), "shank": _norm("Right_Ankle")},
    }


# ── PyBullet wrappers ────────────────────────────────────────────────────────
# Never call p.* directly in control logic. Use these instead.

def get_joint_state(body_id: int, joint_index: int) -> tuple[float, float]:
    """Return (position_rad, velocity_rad_s) for one joint.

    Wraps p.getJointState. Keep all p.* calls in this file.
    """
    state = p.getJointState(body_id, joint_index)
    return float(state[0]), float(state[1])


def set_joint_position_target(body_id: int, joint_index: int,
                               target_rad: float, kp: float,
                               kd: float, max_torque: float) -> None:
    """Apply PD position control to one joint.

    Wraps p.setJointMotorControl2. Keep all p.* calls in this file.
    kp: position gain (N·m/rad)
    kd: velocity gain (N·m·s/rad)
    max_torque: effort limit (N·m)
    """
    p.setJointMotorControl2(
        body_id,
        joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_rad,
        positionGain=kp,
        velocityGain=kd,
        force=max_torque,
    )
```

- [ ] **Step 5: Run interface tests — confirm they pass**

```bash
pytest test_kinematics.py -k "interface or segment or joint_limits" -v
```

Expected: 4 passed

- [ ] **Step 6: Commit**

```bash
git add sim/__init__.py sim/interface.py test_kinematics.py
git commit -m "feat: add sim/interface.py URDF abstraction layer

Isolates all pybullet calls and URDF parsing. get_joint_limits() and
get_segment_lengths() are pure stdlib. Enables future Gazebo swap."
```

---

## Task 3: Add Kinematics Fields to shared_state.py

**Files:**
- Modify: `shared_state.py`

### Background

All inter-loop state lives in `Siclo1State` (CLAUDE.md). New fields: IK output angles per leg, current swing phase, and the angular momentum torso correction. No module-local persistence is permitted.

- [ ] **Step 1: Add fields to Siclo1State.__init__**

In `shared_state.py`, after the `# RECOVERY STATE` block (around line 242), insert a new section:

```python
        # ====================================================================
        # KINEMATICS STATE (Written by kinematics.py, read by WBC / HeartBeat)
        # ====================================================================
        # IK output angles, ready for URDF joint targets (rad, URDF-signed).
        # Tuple: (hip_pitch, knee, ankle) per side.
        self.ik_left_angles:  tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.ik_right_angles: tuple[float, float, float] = (0.0, 0.0, 0.0)

        # Swing trajectory state — must persist across loop iterations.
        self.swing_phase: float = 0.0           # normalized phase ∈ [0, 1]
        self.swing_foot_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        # m, (x, y, z) touchdown target relative to hip-pitch joint, world frame

        # Feedforward torso pitch correction from angular momentum (rad).
        self.torso_pitch_correction: float = 0.0
```

- [ ] **Step 2: Commit**

```bash
git add shared_state.py
git commit -m "feat: add kinematics output fields to Siclo1State

ik_left_angles, ik_right_angles, swing_phase, swing_foot_target,
torso_pitch_correction — all loop-persistent state lives in shared_state."
```

---

## Task 4: kinematics.py — Constants, Workspace Clamp, and IK Solver

**Files:**
- Create: `kinematics.py`
- Modify: `test_kinematics.py`

- [ ] **Step 1: Write failing tests for clamp_foot_target**

Append to `test_kinematics.py`:

```python
# ── kinematics.py tests ──────────────────────────────────────────────────────
from kinematics import (
    R_MIN, R_MAX, L_THIGH, L_SHANK,
    clamp_foot_target, solve_ik,
    swing_trajectory, angular_momentum_correction,
)


class TestClampFootTarget:
    def test_inside_annulus_unchanged(self):
        d_mid = (R_MIN + R_MAX) / 2.0
        x_in, z_in = clamp_foot_target(0.0, -d_mid)
        assert abs(x_in - 0.0) < 1e-9
        assert abs(z_in - (-d_mid)) < 1e-9

    def test_below_r_min_clamped_to_r_min(self):
        x_c, z_c = clamp_foot_target(0.0, -(R_MIN - 0.01))
        d = math.sqrt(x_c**2 + z_c**2)
        assert abs(d - R_MIN) < 1e-6

    def test_above_r_max_clamped_to_r_max(self):
        x_c, z_c = clamp_foot_target(0.0, -(R_MAX + 0.05))
        d = math.sqrt(x_c**2 + z_c**2)
        assert abs(d - R_MAX) < 1e-6

    def test_direction_preserved_after_clamp(self):
        x_raw, z_raw = 0.1, -(R_MAX + 0.1)
        x_c, z_c = clamp_foot_target(x_raw, z_raw)
        # Clamped vector must point in same direction as original
        assert abs(x_c / z_c - x_raw / z_raw) < 1e-6

    def test_at_r_min_boundary_unchanged(self):
        x_c, z_c = clamp_foot_target(0.0, -R_MIN)
        assert abs(math.sqrt(x_c**2 + z_c**2) - R_MIN) < 1e-9

    def test_at_r_max_boundary_unchanged(self):
        x_c, z_c = clamp_foot_target(0.0, -R_MAX)
        assert abs(math.sqrt(x_c**2 + z_c**2) - R_MAX) < 1e-9

    def test_degenerate_zero_input_returns_r_min_down(self):
        x_c, z_c = clamp_foot_target(0.0, 0.0)
        assert abs(x_c) < 1e-9
        assert abs(z_c - (-R_MIN)) < 1e-9
```

- [ ] **Step 2: Write failing tests for solve_ik**

Append to `test_kinematics.py`:

```python
class TestSolveIK:
    def _assert_within_urdf_limits(self, hip, knee, ankle, side):
        from sim.interface import get_joint_limits
        limits = get_joint_limits()
        hip_name    = "Left_Hip_Forwards" if side == "left" else "Right_Hip_Fowards"
        knee_name   = "Left_Knee"         if side == "left" else "Right_Knee"
        ankle_name  = "Left_Ankle"        if side == "left" else "Right_Ankle"
        assert limits[hip_name]["lower"]   <= hip   <= limits[hip_name]["upper"],   f"hip out of range: {hip}"
        assert limits[knee_name]["lower"]  <= knee  <= limits[knee_name]["upper"],  f"knee out of range: {knee}"
        assert limits[ankle_name]["lower"] <= ankle <= limits[ankle_name]["upper"], f"ankle out of range: {ankle}"

    def test_foot_directly_below_hip_left(self):
        # Foot straight below hip at mid-reach — expect near-zero hip angle
        d_mid = (R_MIN + R_MAX) / 2.0
        hip, knee, ankle = solve_ik((0.0, 0.0, -d_mid), "left")
        self._assert_within_urdf_limits(hip, knee, ankle, "left")
        assert abs(hip) < 0.3  # small hip angle for straight-down foot

    def test_foot_directly_below_hip_right(self):
        d_mid = (R_MIN + R_MAX) / 2.0
        hip, knee, ankle = solve_ik((0.0, 0.0, -d_mid), "right")
        self._assert_within_urdf_limits(hip, knee, ankle, "right")
        assert abs(hip) < 0.3

    def test_left_and_right_hip_signs_opposite_for_same_target(self):
        # Left and Right have mirrored axes: same geometric target → opposite URDF signs
        target = (0.05, 0.0, -(R_MIN + R_MAX) / 2.0)
        hip_l, knee_l, _ = solve_ik(target, "left")
        hip_r, knee_r, _ = solve_ik(target, "right")
        assert hip_l * hip_r < 0 or (abs(hip_l) < 1e-9 and abs(hip_r) < 1e-9)
        assert knee_l * knee_r < 0 or (abs(knee_l) < 1e-9 and abs(knee_r) < 1e-9)

    def test_ankle_always_zero(self):
        # Ankle axis ≈ -Z (yaw, not sagittal pitch); IK sets it to neutral
        for side in ("left", "right"):
            _, _, ankle = solve_ik((0.05, 0.0, -0.68), side)
            assert ankle == 0.0

    def test_out_of_workspace_target_clamped_not_error(self):
        # Target beyond R_MAX must succeed (clamped) rather than raise
        hip, knee, ankle = solve_ik((0.0, 0.0, -1.5), "left")
        assert isinstance(hip, float)
        assert isinstance(knee, float)

    def test_within_urdf_limits_left_forward_target(self):
        hip, knee, ankle = solve_ik((0.05, 0.0, -0.68), "left")
        self._assert_within_urdf_limits(hip, knee, ankle, "left")

    def test_within_urdf_limits_right_forward_target(self):
        hip, knee, ankle = solve_ik((0.05, 0.0, -0.68), "right")
        self._assert_within_urdf_limits(hip, knee, ankle, "right")
```

- [ ] **Step 3: Run all new tests to confirm they fail**

```bash
pytest test_kinematics.py -k "TestClamp or TestSolveIK" -v
```

Expected: all FAILED with `ImportError: cannot import name 'clamp_foot_target' from 'kinematics'`

- [ ] **Step 4: Create kinematics.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/kinematics.py`:

```python
"""Kinematics for Siclo1 bipedal robot.

Pure math module — no PyBullet import. All sim calls go through sim/interface.py.
Left leg is the canonical segment-length reference (verified 2026-04-04).

Axis-sign convention (URDF):
  Left  hip/knee axis = -X  →  return negated geometric angles
  Right hip/knee axis = +X  →  return geometric angles unchanged
Ankle axis ≈ -Z (yaw, not sagittal pitch) → set to 0.0 by IK.
"""
import math

# ── Canonical segment lengths (Left leg, patched ITER-001/002) ────────────────
L_THIGH            = 0.060661  # m, hip-pitch axis to knee pivot
L_SHANK            = 0.686961  # m, knee pivot to ankle pivot
SINGULARITY_BUFFER = 0.005     # m, workspace annulus margin (avoid lock/collapse)
R_MIN = abs(L_THIGH - L_SHANK) + SINGULARITY_BUFFER  # m, 0.631300
R_MAX = L_THIGH + L_SHANK - SINGULARITY_BUFFER        # m, 0.742622

SWING_HEIGHT = 0.04  # m, default foot clearance above ground during swing


def clamp_foot_target(x_f: float, z_f: float) -> tuple[float, float]:
    """Radially clamp (x_f, z_f) to the reachable annulus [R_MIN, R_MAX].

    x_f: sagittal forward offset from hip-pitch joint (m)
    z_f: vertical offset from hip-pitch joint (m, negative = below hip)
    Returns clamped (x_f, z_f) — direction preserved, magnitude bounded.
    """
    d = math.sqrt(x_f * x_f + z_f * z_f)
    if d < 1e-9:                          # degenerate: foot at hip origin
        return 0.0, -R_MIN
    if d < R_MIN:
        scale = R_MIN / d
        return x_f * scale, z_f * scale
    if d > R_MAX:
        scale = R_MAX / d
        return x_f * scale, z_f * scale
    return x_f, z_f


def solve_ik(foot_xyz: tuple[float, float, float],
             side: str) -> tuple[float, float, float]:
    """2-link planar IK in the sagittal plane.

    foot_xyz: (x, y, z) foot target relative to hip-pitch joint (m, world frame).
              The y component is ignored — this is a sagittal-plane solver.
    side:     'left' or 'right'

    Returns (hip_pitch, knee, ankle) in rad, URDF-signed and ready for joint targets:
      hip_pitch  positive = leg swings forward
      knee       positive = flexion (knee bends)
      ankle      0.0 (ankle URDF axis ≈ -Z = yaw, not sagittal pitch)

    Raises ValueError if side is not 'left' or 'right'.
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    x_f, _y, z_f = foot_xyz                    # project to sagittal plane
    x_f, z_f = clamp_foot_target(x_f, z_f)
    d = math.sqrt(x_f * x_f + z_f * z_f)

    # ── Knee angle (law of cosines, interior angle at knee) ───────────────────
    cos_gamma = (L_THIGH ** 2 + L_SHANK ** 2 - d ** 2) / (2.0 * L_THIGH * L_SHANK)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))  # clamp for floating-point safety
    gamma = math.acos(cos_gamma)                # interior angle: π = straight, 0 = collapsed
    theta_knee_geo = math.pi - gamma            # joint angle: 0 = straight, +ve = flexion

    # ── Hip pitch angle ───────────────────────────────────────────────────────
    alpha = math.atan2(x_f, -z_f)              # angle from vertical (down) to foot direction
    cos_beta = (L_THIGH ** 2 + d ** 2 - L_SHANK ** 2) / (2.0 * L_THIGH * d)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)                  # triangle angle at hip
    theta_hip_geo = alpha - beta                # hip pitch: +ve = leg forward

    # ── Ankle: axis ≈ -Z (yaw) → neutral ─────────────────────────────────────
    theta_ankle = 0.0

    # ── Apply URDF axis-sign rule ──────────────────────────────────────────────
    if side == "left":                          # Left axis = -X → negate
        return -theta_hip_geo, -theta_knee_geo, theta_ankle
    return theta_hip_geo, theta_knee_geo, theta_ankle  # Right axis = +X → keep
```

- [ ] **Step 5: Run clamp and IK tests — confirm they pass**

```bash
pytest test_kinematics.py -k "TestClamp or TestSolveIK" -v
```

Expected: all passed

- [ ] **Step 6: Commit**

```bash
git add kinematics.py test_kinematics.py
git commit -m "feat: add clamp_foot_target and solve_ik to kinematics.py

2-link planar IK with workspace clamping [R_MIN=0.6313m, R_MAX=0.7426m].
Axis-sign rule applied per URDF: left negated, right unchanged."
```

---

## Task 5: kinematics.py — Swing Trajectory and Angular Momentum Correction

**Files:**
- Modify: `kinematics.py` (functions already stubbed below — add them)
- Modify: `test_kinematics.py`

- [ ] **Step 1: Write failing tests for swing_trajectory**

Append to `test_kinematics.py`:

```python
class TestSwingTrajectory:
    def test_x_at_phi_zero_equals_x_start(self):
        x, z = swing_trajectory(0.0, 0.1, 0.3, 0.04)
        assert abs(x - 0.1) < 1e-9

    def test_x_at_phi_one_equals_x_end(self):
        x, z = swing_trajectory(1.0, 0.1, 0.3, 0.04)
        assert abs(x - 0.3) < 1e-9

    def test_z_at_phi_zero_is_zero(self):
        _, z = swing_trajectory(0.0, 0.0, 0.1, 0.04)
        assert abs(z) < 1e-9

    def test_z_at_phi_one_is_zero(self):
        _, z = swing_trajectory(1.0, 0.0, 0.1, 0.04)
        assert abs(z) < 1e-9

    def test_z_at_phi_half_equals_H(self):
        H = 0.04
        _, z = swing_trajectory(0.5, 0.0, 0.1, H)
        assert abs(z - H) < 1e-9

    def test_zero_velocity_at_liftoff(self):
        # dx/dphi = (x_end - x_start) * (1 - cos(2π*phi)); at phi=0: 1-cos(0)=0
        # dz/dphi = H*π*sin(2π*phi); at phi=0: sin(0)=0
        eps = 1e-7
        x0, z0 = swing_trajectory(0.0, 0.0, 0.1, 0.04)
        x1, z1 = swing_trajectory(eps, 0.0, 0.1, 0.04)
        assert abs((x1 - x0) / eps) < 1e-4   # m/phase_unit
        assert abs((z1 - z0) / eps) < 1e-4

    def test_zero_velocity_at_touchdown(self):
        eps = 1e-7
        x0, z0 = swing_trajectory(1.0 - eps, 0.0, 0.1, 0.04)
        x1, z1 = swing_trajectory(1.0, 0.0, 0.1, 0.04)
        assert abs((x1 - x0) / eps) < 1e-4
        assert abs((z1 - z0) / eps) < 1e-4

    def test_phi_out_of_range_raises(self):
        with pytest.raises(ValueError):
            swing_trajectory(-0.01, 0.0, 0.1, 0.04)
        with pytest.raises(ValueError):
            swing_trajectory(1.01, 0.0, 0.1, 0.04)
```

- [ ] **Step 2: Write failing tests for angular_momentum_correction**

Append to `test_kinematics.py`:

```python
class TestAngularMomentumCorrection:
    def test_zero_hip_deviation_gives_zero_correction(self):
        assert angular_momentum_correction(0.0, 2.0, 8.0) == 0.0

    def test_positive_hip_deviation_gives_negative_correction(self):
        # Swing leg deviates forward → torso must pitch backward to compensate
        corr = angular_momentum_correction(0.3, 2.0, 8.0)
        assert corr < 0.0

    def test_formula_matches_expected_value(self):
        # Δθ_torso = -(m_leg / m_total) * Δθ_hip
        corr = angular_momentum_correction(0.5, 2.0, 8.0)
        expected = -(2.0 / 8.0) * 0.5
        assert abs(corr - expected) < 1e-9

    def test_zero_m_total_raises(self):
        with pytest.raises(ValueError):
            angular_momentum_correction(0.1, 2.0, 0.0)
```

- [ ] **Step 3: Run to confirm they fail**

```bash
pytest test_kinematics.py -k "TestSwing or TestAngular" -v
```

Expected: all FAILED with `ImportError`

- [ ] **Step 4: Add swing_trajectory and angular_momentum_correction to kinematics.py**

Append to `/home/notlord/ros2_ws/Siclo1_V1/kinematics.py`:

```python

def swing_trajectory(phi: float,
                     x_start: float,
                     x_end: float,
                     H: float) -> tuple[float, float]:
    """Cycloidal foot trajectory — zero velocity at liftoff and touchdown.

    phi:     normalized phase [0, 1] (0 = liftoff, 1 = touchdown)
    x_start: foot X at liftoff (m, world frame)
    x_end:   foot X at touchdown (m, world frame)
    H:       maximum swing clearance height (m, e.g. SWING_HEIGHT = 0.04 m)

    Returns (x, z) foot position (m). z = 0 at phi=0 and phi=1 by construction.

    Raises ValueError if phi is outside [0, 1].
    """
    if not 0.0 <= phi <= 1.0:
        raise ValueError(f"phi must be in [0, 1], got {phi!r}")
    two_pi_phi = 2.0 * math.pi * phi
    x = x_start + (x_end - x_start) * (phi - math.sin(two_pi_phi) / (2.0 * math.pi))
    z = H * (1.0 - math.cos(two_pi_phi)) / 2.0
    return x, z


def angular_momentum_correction(delta_theta_hip_swing: float,
                                m_leg: float,
                                m_total: float) -> float:
    """Feedforward torso pitch correction for angular momentum during swing.

    Derived from conservation of angular momentum:
      Δθ_torso = -(m_leg / m_total) × Δθ_hip_swing

    delta_theta_hip_swing: swing hip deviation from neutral (rad)
    m_leg:   swing leg mass (kg)
    m_total: total robot mass (kg, 8.0 kg nominal)

    Returns Δθ_torso (rad) — apply as feedforward on torso pitch joint.
    Raises ValueError if m_total ≤ 0.
    """
    if m_total <= 0.0:
        raise ValueError(f"m_total must be positive, got {m_total!r}")
    return -(m_leg / m_total) * delta_theta_hip_swing  # rad, feedforward torso pitch
```

- [ ] **Step 5: Run all new tests — confirm they pass**

```bash
pytest test_kinematics.py -k "TestSwing or TestAngular" -v
```

Expected: all passed

- [ ] **Step 6: Run full test suite — confirm nothing broken**

```bash
pytest test_kinematics.py -v
```

Expected: all passed

- [ ] **Step 7: Commit**

```bash
git add kinematics.py test_kinematics.py
git commit -m "feat: add swing_trajectory and angular_momentum_correction

Cycloidal profile guarantees zero velocity at liftoff/touchdown.
Feedforward torso pitch: Δθ = -(m_leg/m_total) × Δθ_hip."
```

---

## Task 6: Timing Guard Verification

**Files:**
- Modify: `test_kinematics.py`

### Background

The 100 Hz loop has a 10 ms hard limit (CLAUDE.md). A single `solve_ik` call must complete in ≤ 2 ms (leaving budget for balance, WBC, and telemetry). Verify this over 1 000 iterations.

- [ ] **Step 1: Write the timing test**

Append to `test_kinematics.py`:

```python
import time


def test_solve_ik_under_2ms_per_call():
    """solve_ik must complete in < 2 ms. 100 Hz loop budget constraint."""
    target = (0.05, 0.0, -0.68)
    N = 1000
    start = time.perf_counter()
    for _ in range(N):
        solve_ik(target, "left")
    elapsed_s = time.perf_counter() - start
    avg_ms = (elapsed_s / N) * 1000.0
    assert avg_ms < 2.0, f"solve_ik averaged {avg_ms:.3f} ms — exceeds 2 ms budget"
```

- [ ] **Step 2: Run the timing test**

```bash
pytest test_kinematics.py::test_solve_ik_under_2ms_per_call -v
```

Expected: PASSED (pure Python `math` functions run well under 0.1 ms each)

- [ ] **Step 3: Commit**

```bash
git add test_kinematics.py
git commit -m "test: verify solve_ik timing under 2ms per call (100Hz budget)"
```

---

## Self-Review

**Spec coverage check:**

| Spec item (handoff doc) | Task that implements it |
|---|---|
| Q1 — Workspace clamping, annulus [R_min, R_max], buffer=0.005m | Task 4 — `clamp_foot_target` |
| Q2 — Cycloidal swing trajectory, zero velocity at endpoints | Task 5 — `swing_trajectory` |
| Q3 — Angular momentum feedforward torso pitch correction | Task 5 — `angular_momentum_correction` |
| Q4 — `assert_ik_within_urdf_limits` test pattern | Task 4 — `TestSolveIK._assert_within_urdf_limits` |
| ITER-001 — Right_Knee origin fix | Task 1 |
| ITER-002 — Right_Ankle origin fix | Task 1 |
| `sim/interface.py` abstraction layer (CLAUDE.md) | Task 2 |
| `Siclo1State` fields for kinematics (CLAUDE.md) | Task 3 |
| 100 Hz timing guard | Task 6 |
| urdf_changes.md documentation protocol | Task 1 (log updated) |

**Placeholder scan:** None found. Every step contains complete code or an exact command.

**Type consistency check:**
- `clamp_foot_target(x_f, z_f) → tuple[float, float]` — used correctly in `solve_ik`
- `solve_ik(foot_xyz, side) → tuple[float, float, float]` — matches test import and Siclo1State field types
- `swing_trajectory(phi, x_start, x_end, H) → tuple[float, float]` — matches test calls
- `angular_momentum_correction(delta, m_leg, m_total) → float` — matches test calls
- `get_segment_lengths() → dict[str, dict[str, float]]` — matches test assertions
