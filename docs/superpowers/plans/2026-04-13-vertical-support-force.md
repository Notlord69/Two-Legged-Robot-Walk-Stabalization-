# Vertical Support Force Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the GRF spring formula so the robot produces ~39 N per leg at standing
instead of 1192 N, eliminating the COM_Z-rising collapse pattern.

**Architecture:** Three independent fixes in two files. (1) `grf.py` — change the
spring's compression variable from absolute foot z to leg extension (`z_hip - z_foot`)
and add a gravity feedforward constant so equilibrium force equals half body-weight.
(2) `HeartBeat.py` — disable PyBullet's default velocity motors at init so
`TORQUE_CONTROL` has full authority; reset `target_torques` to `{}` each WBC cycle
to stop torque accumulation.

**Tech Stack:** Python 3.10, PyBullet, pytest, numpy

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `grf.py` | Modify | Spring formula, gravity feedforward, z_hip plumbing |
| `HeartBeat.py` | Modify | Motor disable at init, torque dict reset in WBC |
| `test_grf.py` | Create | 3 unit tests for spring equilibrium / compression / extension |

---

## Task 1: Write failing GRF unit tests

**Files:**
- Create: `test_grf.py`

- [ ] **Step 1: Create test file**

```python
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
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_grf.py -v
```

Expected: 3 FAILED or ERROR — either `ImportError: cannot import name 'GRAVITY_COMP'`
or wrong values (old formula produces `F_z=0` at `Z_REST`, not `GRAVITY_COMP`).

---

## Task 2: Add GRAVITY_COMP constant and fix `_spring_damper_fz`

**Files:**
- Modify: `grf.py:56-88`

- [ ] **Step 1: Add GRAVITY_COMP constant after existing constants**

In `grf.py`, in the `PHYSICAL CONSTANTS` section (around line 63), add two lines after
the existing `Z_REST`, `K_SPRING`, `B_DAMPER` block:

```python
# Per-leg gravity feedforward — ensures F_z = half body-weight at nominal stance.
# At equilibrium (leg_ext == Z_REST), spring term = 0; GRAVITY_COMP carries the load.
ROBOT_MASS:   float = 8.0                       # kg, total robot mass (URDF-derived)
GRAVITY_COMP: float = ROBOT_MASS * 9.81 / 2.0   # N·per·leg, feedforward at standing
```

The block should look like this after the edit:
```python
Z_REST:       float = 0.75     # m,     nominal standing leg extension (hip to foot, vertical)
K_SPRING:     float = 1589.0   # N/m,   supports 8.1 kg with max 5 cm compression
B_DAMPER:     float = 94.0     # N·s/m, ζ=0.7 critical damping ratio for impact absorption
DECEL_SPRING_BOOST: float = 1.2   # dimensionless, applied to K_SPRING in DECEL state

ROBOT_MASS:   float = 8.0                       # kg, total robot mass (URDF-derived)
GRAVITY_COMP: float = ROBOT_MASS * 9.81 / 2.0   # N·per·leg, feedforward at standing
```

- [ ] **Step 2: Fix `_spring_damper_fz` signature and formula**

Replace the existing function (lines ~78-88):
```python
def _spring_damper_fz(z_foot: float, z_dot_foot: float,
                      k_spring: float) -> float:
    """Compute desired vertical support force via virtual spring-damper.

    z_foot    : foot z position in world frame (m)
    z_dot_foot: foot z velocity in world frame (m/s)
    k_spring  : spring constant to use (N/m) — may be boosted for DECEL
    Returns F_z (N), positive = upward support.
    """
    return k_spring * (Z_REST - z_foot) - B_DAMPER * z_dot_foot
```

With:
```python
def _spring_damper_fz(leg_ext: float, z_dot_foot: float,
                      k_spring: float) -> float:
    """Compute desired vertical support force via virtual spring-damper.

    leg_ext   : z_hip - z_foot (m); equals Z_REST at nominal standing posture.
                Positive = leg extended; negative = impossible geometry.
    z_dot_foot: foot z velocity in world frame (m/s), positive = moving up
    k_spring  : spring constant (N/m) — may be boosted for DECEL
    Returns F_z (N), positive = upward support.

    At equilibrium (leg_ext == Z_REST): F_z = GRAVITY_COMP (half body-weight).
    Compression (leg_ext < Z_REST): F_z > GRAVITY_COMP (extra support).
    Extension  (leg_ext > Z_REST): F_z < GRAVITY_COMP (less support).
    """
    return GRAVITY_COMP + k_spring * (Z_REST - leg_ext) - B_DAMPER * z_dot_foot
```

- [ ] **Step 3: Run tests — first two should pass now**

```bash
python -m pytest test_grf.py -v
```

Expected output:
```
test_grf.py::test_grf_equilibrium  PASSED
test_grf.py::test_grf_compression  PASSED
test_grf.py::test_grf_extension    PASSED
```

All 3 pass because `_spring_damper_fz` is now correct. If any fail, verify the
formula matches Step 2 exactly.

- [ ] **Step 4: Commit**

```bash
git add grf.py test_grf.py
git commit -m "fix: correct GRF spring formula to use leg extension + gravity feedforward

At standing (foot on ground, z_foot≈0), the old formula produced
F_z = K_SPRING * Z_REST = 1192 N, extending legs until lockout.
New formula: F_z = GRAVITY_COMP + K_SPRING*(Z_REST - leg_ext)
produces 39.2 N at nominal stance, matching half body-weight."
```

---

## Task 3: Fix `_compute_leg_correction` to accept and use `z_hip`

**Files:**
- Modify: `grf.py:119-152`

- [ ] **Step 1: Add `z_hip` parameter and compute `leg_ext` internally**

Replace the existing `_compute_leg_correction` function:

```python
def _compute_leg_correction(
    z_foot: float,
    z_dot_foot: float,
    q_hip_urdf: float,
    q_knee_urdf: float,
    urdf_sign: float,
    hip_key: str,
    knee_key: str,
    k_spring: float,
    ramp_gain: float,
) -> Dict[str, float]:
    """Compute GRF torque corrections for one leg.
    ...
    """
    theta_hip_geo  = urdf_sign * q_hip_urdf
    theta_knee_geo = urdf_sign * q_knee_urdf

    fz = _spring_damper_fz(z_foot, z_dot_foot, k_spring)
    ...
```

With:

```python
def _compute_leg_correction(
    z_foot: float,
    z_hip: float,
    z_dot_foot: float,
    q_hip_urdf: float,
    q_knee_urdf: float,
    urdf_sign: float,
    hip_key: str,
    knee_key: str,
    k_spring: float,
    ramp_gain: float,
) -> Dict[str, float]:
    """Compute GRF torque corrections for one leg.

    z_foot    : foot z in world frame (m), from getLinkState
    z_hip     : hip link z in world frame (m), from link_positions
    z_dot_foot: foot z velocity (m/s)
    urdf_sign : +1.0 for right (axis=+X), -1.0 for left (axis=-X).
    URDF → geometric: θ_geo = urdf_sign * q_urdf
    Geometric → URDF: τ_urdf = urdf_sign * τ_geo

    Returns {hip_key: τ, knee_key: τ} — URDF-signed, clipped, gain-scaled.
    """
    theta_hip_geo  = urdf_sign * q_hip_urdf
    theta_knee_geo = urdf_sign * q_knee_urdf

    leg_ext = z_hip - z_foot  # m, actual leg extension; Z_REST at nominal stance
    fz = _spring_damper_fz(leg_ext, z_dot_foot, k_spring)
    tau_hip_geo, tau_knee_geo = _jacobian_torques(fz, theta_hip_geo, theta_knee_geo)

    tau_hip_urdf  = urdf_sign * tau_hip_geo
    tau_knee_urdf = urdf_sign * tau_knee_geo

    return {
        hip_key:  _clip(hip_key,  tau_hip_urdf  * ramp_gain),
        knee_key: _clip(knee_key, tau_knee_urdf * ramp_gain),
    }
```

- [ ] **Step 2: Run existing tests to confirm no regression**

```bash
python -m pytest test_grf.py -v
```

Expected: 3 PASSED (same as before — tests call `_spring_damper_fz` directly and
are unaffected by `_compute_leg_correction` signature change).

- [ ] **Step 3: Commit**

```bash
git add grf.py
git commit -m "fix: add z_hip parameter to _compute_leg_correction for leg extension"
```

---

## Task 4: Fix `GRFController.update` to look up `z_hip` per leg

**Files:**
- Modify: `grf.py:159-232`

- [ ] **Step 1: Add module-level fallback constant and z_hip lookup in `update`**

Add this constant just above the `GRFController` class definition (around line 159):

```python
# Fallback z_hip when link_positions not yet populated (first cycle).
# Produces leg_ext = Z_REST → spring term = 0 → F_z = GRAVITY_COMP. Safe default.
_Z_HIP_DEFAULT: float = Z_REST   # m
```

- [ ] **Step 2: Add z_hip lookups in `update`, before the left-leg block**

In `GRFController.update`, after the `jp = shared_state.joint_positions` line and
before the left-leg eligibility check, add:

```python
lp = shared_state.link_positions
z_hip_left  = float(lp['Left_Upper_Leg_1'][2])  if 'Left_Upper_Leg_1'  in lp else _Z_HIP_DEFAULT
z_hip_right = float(lp['Right_Upper_Leg_1'][2]) if 'Right_Upper_Leg_1' in lp else _Z_HIP_DEFAULT
```

- [ ] **Step 3: Pass `z_hip` to both `_compute_leg_correction` calls**

Left leg call — add `z_hip=z_hip_left` after `z_foot`:
```python
if (left_eligible and
        shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED):
    result.update(_compute_leg_correction(
        z_foot      = float(shared_state.left_foot_position[2]),
        z_hip       = z_hip_left,
        z_dot_foot  = float(shared_state.left_foot_velocity[2]),
        q_hip_urdf  = jp.get('Left_Hip_Forwards', 0.0),
        q_knee_urdf = jp.get('Left_Knee', 0.0),
        urdf_sign   = -1.0,
        hip_key     = 'Left_Hip_Forwards',
        knee_key    = 'Left_Knee',
        k_spring    = k_spring,
        ramp_gain   = ramp_gain,
    ))
```

Right leg call — add `z_hip=z_hip_right` after `z_foot`:
```python
if (right_eligible and
        shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED):
    result.update(_compute_leg_correction(
        z_foot      = float(shared_state.right_foot_position[2]),
        z_hip       = z_hip_right,
        z_dot_foot  = float(shared_state.right_foot_velocity[2]),
        q_hip_urdf  = jp.get('Right_Hip_Fowards', 0.0),
        q_knee_urdf = jp.get('Right_Knee', 0.0),
        urdf_sign   = +1.0,
        hip_key     = 'Right_Hip_Fowards',
        knee_key    = 'Right_Knee',
        k_spring    = k_spring,
        ramp_gain   = ramp_gain,
    ))
```

- [ ] **Step 4: Run all tests**

```bash
python -m pytest test_grf.py -v
```

Expected: 3 PASSED.

- [ ] **Step 5: Commit**

```bash
git add grf.py
git commit -m "fix: wire z_hip into GRFController.update via link_positions lookup"
```

---

## Task 5: Disable default PyBullet velocity motors at init

**Files:**
- Modify: `HeartBeat.py:302-318` (`PyBulletInterface._build_joint_map`)

- [ ] **Step 1: Append motor-disable loop after the joint-map loop**

In `_build_joint_map`, after the line `self._joint_list = list(self.joint_ids.items())`,
add:

```python
        # Disable default velocity motors so TORQUE_CONTROL has full authority.
        # PyBullet loads every joint with an internal velocity motor that fights
        # TORQUE_CONTROL commands unless zeroed here. Called once at init.
        for jid in self.joint_ids.values():
            p.setJointMotorControl2(
                self.robot_id, jid,
                controlMode=p.VELOCITY_CONTROL,
                force=0,
                physicsClientId=self.pc,
            )
```

The full end of `_build_joint_map` should look like:

```python
        # Freeze iteration order
        self._joint_list = list(self.joint_ids.items())

        # Disable default velocity motors so TORQUE_CONTROL has full authority.
        # PyBullet loads every joint with an internal velocity motor that fights
        # TORQUE_CONTROL commands unless zeroed here. Called once at init.
        for jid in self.joint_ids.values():
            p.setJointMotorControl2(
                self.robot_id, jid,
                controlMode=p.VELOCITY_CONTROL,
                force=0,
                physicsClientId=self.pc,
            )

        # Joint map info is logged via TelemetryThread after init
```

- [ ] **Step 2: Confirm existing tests still pass**

```bash
python -m pytest test_grf.py test_gait_planner_fsm.py test_stance_anchor.py test_step_phase_guards.py -v
```

Expected: all PASSED. The motor-disable loop only runs in the live PyBullet context
(real `p.*` calls); unit tests mock or skip `_build_joint_map`.

- [ ] **Step 3: Commit**

```bash
git add HeartBeat.py
git commit -m "fix: disable default PyBullet velocity motors before TORQUE_CONTROL loop

Without this, each joint's default velocity motor resists WBC torque
commands, reducing effective torque delivery by ~30-60%."
```

---

## Task 6: Fix WBC torque accumulation — reset `target_torques` each cycle

**Files:**
- Modify: `HeartBeat.py:753-779` (`Siclo1Controller._wbc_step`)

- [ ] **Step 1: Replace the dict-read with a fresh dict**

In `_wbc_step`, replace:
```python
        torques = getattr(shared_state, 'target_torques', {})
```
With:
```python
        torques = {}   # reset each cycle; GRF corrections are merged in apply_control
```

The full method should now start with:
```python
    def _wbc_step(self) -> None:
        """Joint-space PD controller: IK angles → additive torques in target_torques.

        Reads ik_left_angles and ik_right_angles (URDF-signed rad).
        Computes PD torque for each leg joint and writes to shared_state.target_torques.
        GRF corrections are NOT merged here — handled in apply_control().
        """
        torques = {}   # reset each cycle; GRF corrections are merged in apply_control

        jp = shared_state.joint_positions
        jv = shared_state.joint_velocities
        ...
```

- [ ] **Step 2: Run all tests**

```bash
python -m pytest test_grf.py test_gait_planner_fsm.py test_stance_anchor.py test_step_phase_guards.py -v
```

Expected: all PASSED.

- [ ] **Step 3: Commit**

```bash
git add HeartBeat.py
git commit -m "fix: reset target_torques to {} each WBC cycle to stop accumulation

_wbc_step was reading the previous cycle's torques and adding to them,
causing torques to grow N× over N cycles. GRF corrections are merged
separately in apply_control and are unaffected by this reset."
```

---

## Task 7: Smoke test — run the simulation and verify telemetry

**Files:** none (run-only verification)

- [ ] **Step 1: Run headless for 300 cycles and capture GATE log**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python main.py --walk 2.0 --duration 300 --on 2>&1 | grep -E "GATE|COM="
```

- [ ] **Step 2: Verify COM_Z is stable**

Look for `COM=[x, y, z]` in telemetry. `z` should stay within `0.84–0.92 m`
for all 300 cycles. Any value rising past `0.93 m` indicates the spring is still
over-extended.

- [ ] **Step 3: Verify forces are non-zero and stable**

Look for `F=[L,R]` in the GATE log. After cycle 5 both `L` and `R` should
read `25–55 N` (varying with gait phase). Values of `F=[0,0]` after the
warmup period means contact confirmation is still failing — check that
`is_flat` passes by printing `left_foot_flat` in `read_sensors`.

- [ ] **Step 4: Verify DS exits to COM_SHIFT**

In the GATE log look for `both=True`. If it appears within the first 200 cycles
the DS gate cleared and COM_SHIFT is running. If `both=True` never appears,
check `ramp_gain` — it must reach `1.0` before the gate opens.

- [ ] **Step 5: Final commit if simulation passes**

```bash
git add .
git commit -m "test: verify vertical support force fix via 300-cycle smoke test"
```
