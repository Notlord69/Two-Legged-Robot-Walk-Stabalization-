# Dual-Mode Runner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `--gui` / `--viz-hz` CLI flags, fix three HeartBeat.py bugs, and implement a live IK debug visualiser (sagittal-plane annulus arcs + life-limited hip→foot vectors).

**Architecture:** Physics always runs headless (`p.DIRECT`). An optional second PyBullet client (`p.GUI`) mirrors joint state at a user-defined render rate. `viz/debug_markers.py` owns all drawing state; all `p.*` calls go through `sim/interface.py` wrappers (CLAUDE.md requirement). `main.py` is the sole argparse entry point.

**Tech Stack:** Python 3.10+, pybullet, argparse (stdlib), unittest.mock (stdlib), pytest

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `HeartBeat.py` | Modify | Fix 3 bugs; remove `main()`; add `viz_decimation` param; add `_warmup()`; wire `DebugVisualizer` |
| `sim/interface.py` | Modify | Add `add_debug_line()` and `remove_debug_line()` wrappers |
| `shared_state.py` | Modify | Add `left_foot_target`, `right_foot_target` fields |
| `viz/__init__.py` | Create | Package marker (empty) |
| `viz/debug_markers.py` | Create | `DebugVisualizer`: annulus arcs + life-limited vectors |
| `main.py` | Create | argparse entry point |
| `test_sim_interface.py` | Create | Tests for new sim/interface wrappers |
| `test_debug_markers.py` | Create | Tests for arc geometry + life-limited line behaviour |
| `test_dual_mode_runner.py` | Create | Tests for argparse and decimation computation |
| `Progress/Custom_Command.md` | Create | CLI reference |

---

## Task 1: HeartBeat.py — Fix 3 bugs + remove main()

**Files:**
- Modify: `HeartBeat.py:386`, `HeartBeat.py:516-521`, `HeartBeat.py:656-689`

**Background:** Three bugs exist before any new code is added. Fix them first so the baseline is clean.

- [ ] **Step 1: Confirm baseline tests pass**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest test_kinematics.py -v --tb=short
```

Expected: 36 passed.

- [ ] **Step 2: Fix Bug 1 — physics client must be p.DIRECT**

In `HeartBeat.py` at line 386, change:

```python
        self.physics_client = p.connect(p.GUI)
```

to:

```python
        self.physics_client = p.connect(p.DIRECT)
```

- [ ] **Step 3: Fix Bug 2 — self.cycle_count → shared_state.cycle_count**

In `HeartBeat.py` at line 516, change:

```python
        if self.cycle_count % 10 == 0:
```

to:

```python
        if shared_state.cycle_count % VIZ_DECIMATION == 0:
```

- [ ] **Step 4: Fix Bug 3 — remove dead strobe block**

In `HeartBeat.py`, delete lines 515-521 (the strobe block). The block to remove is:

```python
        # 11. Selective Visual Update (The "Strobe Light" Method)
        if shared_state.cycle_count % VIZ_DECIMATION == 0:
            # Briefly enable rendering to draw the current state
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            
            # CRITICAL: Disable it immediately so the NEXT 9 cycles are fast
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
```

Note: after Bug 2 is fixed, the condition at line 516 now reads `shared_state.cycle_count % VIZ_DECIMATION == 0` — delete the whole block including that updated line.

- [ ] **Step 5: Remove main() from HeartBeat.py**

Delete the entire `main()` function and the `if __name__ == "__main__":` block at the bottom of `HeartBeat.py` (approximately lines 656–689):

```python
def main():
    print("""
╔════════════════════════════════════════════════════════════════════════╗
...
╚════════════════════════════════════════════════════════════════════════╝
    """)

    # Parse --gui flag
    use_gui = '--gui' in sys.argv

    controller = Siclo1Controller(use_gui=use_gui)

    try:
        controller.run(duration=10.0, print_interval=1.0)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted by user.")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Run baseline tests again — confirm still green**

```bash
python3 -m pytest test_kinematics.py -v --tb=short
```

Expected: 36 passed.

- [ ] **Step 7: Commit**

```bash
git add HeartBeat.py
git commit -m "fix: correct HeartBeat.py bugs — p.DIRECT physics, cycle_count ref, remove dead strobe

Bug 1: physics client was p.GUI (caused render stall in headless mode).
Bug 2: self.cycle_count does not exist; counter is on shared_state.
Bug 3: p.configureDebugVisualizer strobe is dead code on p.DIRECT; removed."
```

---

## Task 2: sim/interface.py — add_debug_line + remove_debug_line

**Files:**
- Modify: `sim/interface.py`
- Create: `test_sim_interface.py`

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_sim_interface.py`:

```python
"""Tests for sim/interface.py debug line wrappers."""
import pytest
from unittest.mock import patch, MagicMock, call


def test_add_debug_line_calls_pybullet_add():
    with patch('pybullet.addUserDebugLine', return_value=42) as mock_add:
        from sim.interface import add_debug_line
        result = add_debug_line([0, 0, 0], [1, 1, 1], [1, 0, 0])
    assert result == 42
    mock_add.assert_called_once()


def test_add_debug_line_passes_replace_id_when_non_negative():
    with patch('pybullet.addUserDebugLine', return_value=5) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [0, 1, 0], replace_id=3)
    kwargs = mock_add.call_args[1]
    assert kwargs.get('replaceItemUniqueId') == 3


def test_add_debug_line_omits_replace_id_when_negative():
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [0, 1, 0], replace_id=-1)
    kwargs = mock_add.call_args[1]
    assert 'replaceItemUniqueId' not in kwargs


def test_remove_debug_line_calls_pybullet_remove():
    with patch('pybullet.removeUserDebugItem') as mock_remove:
        from sim.interface import remove_debug_line
        remove_debug_line(7)
    mock_remove.assert_called_once_with(7, physicsClientId=0)


def test_remove_debug_line_noop_for_negative_id():
    with patch('pybullet.removeUserDebugItem') as mock_remove:
        from sim.interface import remove_debug_line
        remove_debug_line(-1)
    mock_remove.assert_not_called()


def test_add_debug_line_passes_physics_client():
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add:
        from sim.interface import add_debug_line
        add_debug_line([0, 0, 0], [1, 0, 0], [1, 1, 0], physics_client=3)
    kwargs = mock_add.call_args[1]
    assert kwargs['physicsClientId'] == 3
```

- [ ] **Step 2: Run to confirm they fail**

```bash
python3 -m pytest test_sim_interface.py -v
```

Expected: 6 FAILED with `ImportError: cannot import name 'add_debug_line' from 'sim.interface'`

- [ ] **Step 3: Add wrappers to sim/interface.py**

Append to `/home/notlord/ros2_ws/Siclo1_V1/sim/interface.py`:

```python


def add_debug_line(from_xyz, to_xyz, color_rgb,
                   width: float = 1.0,
                   replace_id: int = -1,
                   physics_client: int = 0) -> int:
    """Add or replace a PyBullet debug line. Returns item ID.

    from_xyz:      (x, y, z) start point (m, world frame)
    to_xyz:        (x, y, z) end point (m, world frame)
    color_rgb:     [r, g, b] each in [0, 1]
    width:         line width in pixels
    replace_id:    item ID to update in-place; pass -1 to create a new line
    physics_client: PyBullet client ID (must be a GUI client to be visible)

    All p.* calls are confined to this file (CLAUDE.md).
    """
    import pybullet as p
    kwargs = dict(lineColorRGB=color_rgb, lineWidth=width,
                  physicsClientId=physics_client)
    if replace_id >= 0:
        kwargs['replaceItemUniqueId'] = replace_id
    return p.addUserDebugLine(list(from_xyz), list(to_xyz), **kwargs)


def remove_debug_line(item_id: int, physics_client: int = 0) -> None:
    """Remove a PyBullet debug line by item ID. No-op if item_id < 0.

    All p.* calls are confined to this file (CLAUDE.md).
    """
    import pybullet as p
    if item_id >= 0:
        p.removeUserDebugItem(item_id, physicsClientId=physics_client)
```

- [ ] **Step 4: Run tests — confirm they pass**

```bash
python3 -m pytest test_sim_interface.py -v
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add sim/interface.py test_sim_interface.py
git commit -m "feat: add add_debug_line and remove_debug_line to sim/interface.py

Thin wrappers around p.addUserDebugLine / p.removeUserDebugItem.
replace_id >= 0 triggers in-place update (life-limited line pattern).
All p.* calls confined to sim/interface.py per CLAUDE.md."
```

---

## Task 3: shared_state.py — foot target fields

**Files:**
- Modify: `shared_state.py`
- Modify: `test_kinematics.py`

- [ ] **Step 1: Write the failing test**

Append to `/home/notlord/ros2_ws/Siclo1_V1/test_kinematics.py`:

```python
# ── shared_state.py — foot target fields ─────────────────────────────────────

def test_siclo1state_has_left_foot_target():
    from shared_state import Siclo1State
    s = Siclo1State()
    assert hasattr(s, 'left_foot_target')
    assert s.left_foot_target == (0.0, 0.0, 0.0)


def test_siclo1state_has_right_foot_target():
    from shared_state import Siclo1State
    s = Siclo1State()
    assert hasattr(s, 'right_foot_target')
    assert s.right_foot_target == (0.0, 0.0, 0.0)
```

- [ ] **Step 2: Run to confirm they fail**

```bash
python3 -m pytest test_kinematics.py::test_siclo1state_has_left_foot_target \
                  test_kinematics.py::test_siclo1state_has_right_foot_target -v
```

Expected: 2 FAILED with `AttributeError`.

- [ ] **Step 3: Add fields to Siclo1State**

In `shared_state.py`, find the KINEMATICS STATE block (the one containing `ik_left_angles`, `swing_phase`, `torso_pitch_correction`). After the `torso_pitch_correction` line, add:

```python
        # Per-leg foot targets (m, world frame).
        # Written by gait planner; read by DebugVisualizer.
        # Default (0,0,0) = no active target; visualiser skips red line.
        self.left_foot_target:  tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.right_foot_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
```

- [ ] **Step 4: Run tests — confirm they pass**

```bash
python3 -m pytest test_kinematics.py -v --tb=short
```

Expected: 38 passed (36 existing + 2 new).

- [ ] **Step 5: Commit**

```bash
git add shared_state.py test_kinematics.py
git commit -m "feat: add left_foot_target and right_foot_target to Siclo1State

Per-leg world-frame foot targets (m). Default (0,0,0) means no active
target — DebugVisualizer skips red line when target is zero vector."
```

---

## Task 4: viz/debug_markers.py — DebugVisualizer

**Files:**
- Create: `viz/__init__.py`
- Create: `viz/debug_markers.py`
- Create: `test_debug_markers.py`

**Background:** `DebugVisualizer` draws four sagittal-plane semicircle arcs (R_min and R_max for each hip) and four life-limited vectors (green = hip→actual foot, red = hip→foot target). The arc geometry is a pure function `_arc_points` — tested without PyBullet. The line-update behaviour is tested with mocked `pybullet`.

Hip positions in world space come from `shared_state.link_positions["Left_Upper_Leg_1"]` and `shared_state.link_positions["Right_Upper_Leg_1"]` (child links of the hip-pitch joints, confirmed from the URDF).

- [ ] **Step 1: Write the failing arc geometry tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_debug_markers.py`:

```python
"""Tests for viz/debug_markers.py — arc geometry and life-limited lines."""
import math
import pytest
from unittest.mock import patch, MagicMock, call


# ── Arc geometry (pure function) ─────────────────────────────────────────────

def test_arc_points_returns_n_segs_plus_one():
    from viz.debug_markers import _arc_points
    pts = _arc_points((0.0, 0.0, 1.0), 0.5, 18)
    assert len(pts) == 19  # n_segs + 1 boundary points


def test_arc_midpoint_is_straight_below_hip():
    from viz.debug_markers import _arc_points
    hip = (1.0, 2.0, 3.0)
    r = 0.5
    pts = _arc_points(hip, r, 18)
    mid = pts[9]  # midpoint: theta=0, straight down
    assert abs(mid[0] - hip[0]) < 1e-9   # x = hip_x
    assert abs(mid[1] - hip[1]) < 1e-9   # y = hip_y (sagittal plane)
    assert abs(mid[2] - (hip[2] - r)) < 1e-9  # z = hip_z - r


def test_arc_endpoints_are_at_hip_height():
    from viz.debug_markers import _arc_points
    hip = (0.0, 0.0, 1.0)
    r = 0.5
    pts = _arc_points(hip, r, 18)
    # theta = -pi/2: cos(-pi/2) = 0, so z = hip_z - r*0 = hip_z
    assert abs(pts[0][2] - hip[2]) < 1e-6
    # theta = +pi/2: same
    assert abs(pts[-1][2] - hip[2]) < 1e-6


def test_arc_all_points_at_correct_radius():
    from viz.debug_markers import _arc_points
    hip = (1.0, 2.0, 3.0)
    r = 0.6313
    pts = _arc_points(hip, r, 18)
    for pt in pts:
        dx = pt[0] - hip[0]
        dz = pt[2] - hip[2]
        dist = math.sqrt(dx * dx + dz * dz)
        assert abs(dist - r) < 1e-9


def test_arc_y_coordinate_equals_hip_y():
    from viz.debug_markers import _arc_points
    hip = (0.0, 0.75, 1.0)
    pts = _arc_points(hip, 0.5, 18)
    for pt in pts:
        assert abs(pt[1] - hip[1]) < 1e-9
```

- [ ] **Step 2: Write the failing life-limited line tests**

Append to `test_debug_markers.py`:

```python

# ── Life-limited line behaviour ───────────────────────────────────────────────

def _make_state(left_target=(0.0, 0.0, 0.0), right_target=(0.0, 0.0, 0.0)):
    """Return a minimal mock shared state for DebugVisualizer.update()."""
    s = MagicMock()
    s.left_foot_position  = (0.1, 0.0, 0.1)
    s.right_foot_position = (0.1, 0.5, 0.1)
    s.left_foot_target    = left_target
    s.right_foot_target   = right_target
    return s


def test_green_line_uses_replace_id_on_second_update():
    """Second update must pass the ID from the first as replaceItemUniqueId."""
    with patch('pybullet.addUserDebugLine', return_value=10) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0)
        mock_add.reset_mock()
        mock_add.return_value = 55  # first update returns ID 55 for each line

        vis.update(_make_state(), (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        mock_add.reset_mock()
        mock_add.return_value = 99

        vis.update(_make_state(), (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        # Every green-line call in the second update must include replaceItemUniqueId=55
        green_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [0.0, 1.0, 0.0]
        ]
        assert len(green_calls) == 2  # left + right actual
        for c in green_calls:
            assert c[1].get('replaceItemUniqueId') == 55


def test_red_line_not_drawn_when_target_is_zero_vector():
    """Red lines must be skipped when foot target is (0,0,0)."""
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0)
        mock_add.reset_mock()

        vis.update(_make_state(left_target=(0.0, 0.0, 0.0),
                               right_target=(0.0, 0.0, 0.0)),
                   (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        red_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [1.0, 0.0, 0.0]
        ]
        assert len(red_calls) == 0


def test_red_line_drawn_when_target_is_nonzero():
    """Red line must be drawn when foot target is set."""
    with patch('pybullet.addUserDebugLine', return_value=1) as mock_add, \
         patch('pybullet.removeUserDebugItem'):
        from viz.debug_markers import DebugVisualizer
        vis = DebugVisualizer(physics_client=0)
        mock_add.reset_mock()

        vis.update(_make_state(left_target=(0.05, 0.0, -0.72)),
                   (0.0, 0.0, 1.0), (0.0, 0.5, 1.0))

        red_calls = [
            c for c in mock_add.call_args_list
            if list(c[1]['lineColorRGB']) == [1.0, 0.0, 0.0]
        ]
        assert len(red_calls) == 1  # left leg only
```

- [ ] **Step 3: Run to confirm they all fail**

```bash
python3 -m pytest test_debug_markers.py -v
```

Expected: all FAILED with `ModuleNotFoundError: No module named 'viz'`

- [ ] **Step 4: Create viz/__init__.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/viz/__init__.py` with a single newline.

- [ ] **Step 5: Create viz/debug_markers.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/viz/debug_markers.py`:

```python
"""IK debug visualisation for Siclo1.

Draws in the PyBullet GUI client (not the physics client).
All p.* calls go through sim.interface — never import pybullet here.

Draws:
  - Sagittal-plane semicircle arcs at R_min and R_max for each hip (light blue)
  - Green lines: hip → actual foot position (always drawn)
  - Red lines:   hip → foot target (skipped when target is (0,0,0))

Call update() from _sync_gui() only — never from the 100 Hz physics loop.
"""
import math
from sim.interface import add_debug_line
from kinematics import R_MIN, R_MAX

_ARC_SEGS     = 18                   # line segments per semicircle arc
_ARC_COLOR    = [0.6, 0.85, 1.0]    # light blue
_ACTUAL_COLOR = [0.0, 1.0, 0.0]     # green — hip → actual foot
_TARGET_COLOR = [1.0, 0.0, 0.0]     # red   — hip → foot target
_ZERO_VEC     = (0.0, 0.0, 0.0)     # sentinel: no active target


def _arc_points(hip_pos: tuple, radius: float,
                n_segs: int) -> list[tuple[float, float, float]]:
    """Return n_segs+1 world-space points for a sagittal-plane semicircle.

    The arc spans theta in [-pi/2, +pi/2] around the downward vertical:
      x = hip_x + radius * sin(theta)   (positive = forward)
      y = hip_y                          (constant: sagittal plane)
      z = hip_z - radius * cos(theta)   (theta=0 => straight below hip)

    hip_pos: (x, y, z) world position of the hip-pitch joint (m)
    radius:  arc radius (m)
    n_segs:  number of line segments (returns n_segs+1 points)
    """
    hx, hy, hz = hip_pos
    pts = []
    for i in range(n_segs + 1):
        theta = -math.pi / 2.0 + math.pi * i / n_segs
        pts.append((
            hx + radius * math.sin(theta),
            hy,
            hz - radius * math.cos(theta),
        ))
    return pts


class DebugVisualizer:
    """Manages PyBullet debug lines for IK workspace visualisation.

    Instantiate once after the GUI client is connected.
    Call update() from _sync_gui() every render tick.
    """

    def __init__(self, physics_client: int):
        self._pc = physics_client
        # Arc segment IDs: 4 arcs × _ARC_SEGS segments each
        # Order: L_Rmin, L_Rmax, R_Rmin, R_Rmax
        self._arc_ids: list[int] = [-1] * (4 * _ARC_SEGS)
        # Life-limited vector IDs
        self._left_actual_id:  int = -1
        self._right_actual_id: int = -1
        self._left_target_id:  int = -1
        self._right_target_id: int = -1

    def update(self, state, left_hip: tuple, right_hip: tuple) -> None:
        """Redraw all debug geometry for this render tick.

        state:     Siclo1State (reads left/right_foot_position and _target)
        left_hip:  world pos of left  hip-pitch joint (m)
        right_hip: world pos of right hip-pitch joint (m)
        """
        self._update_annulus(left_hip, right_hip)
        self._left_actual_id  = self._draw_vector(
            left_hip,  state.left_foot_position,
            _ACTUAL_COLOR, self._left_actual_id)
        self._right_actual_id = self._draw_vector(
            right_hip, state.right_foot_position,
            _ACTUAL_COLOR, self._right_actual_id)
        if state.left_foot_target != _ZERO_VEC:
            self._left_target_id = self._draw_vector(
                left_hip,  state.left_foot_target,
                _TARGET_COLOR, self._left_target_id)
        if state.right_foot_target != _ZERO_VEC:
            self._right_target_id = self._draw_vector(
                right_hip, state.right_foot_target,
                _TARGET_COLOR, self._right_target_id)

    # ── private helpers ───────────────────────────────────────────────────────

    def _draw_vector(self, from_pos: tuple, to_pos: tuple,
                     color: list, old_id: int) -> int:
        """Draw or replace one debug line. Returns the new item ID."""
        return add_debug_line(
            from_pos, to_pos, color,
            width=2.0,
            replace_id=old_id,
            physics_client=self._pc,
        )

    def _update_annulus(self, left_hip: tuple, right_hip: tuple) -> None:
        """Redraw the four semicircle arcs following the robot's hips."""
        arcs = [
            (left_hip,  R_MIN, 0),
            (left_hip,  R_MAX, _ARC_SEGS),
            (right_hip, R_MIN, 2 * _ARC_SEGS),
            (right_hip, R_MAX, 3 * _ARC_SEGS),
        ]
        for hip, radius, offset in arcs:
            pts = _arc_points(hip, radius, _ARC_SEGS)
            for i in range(_ARC_SEGS):
                idx = offset + i
                self._arc_ids[idx] = add_debug_line(
                    pts[i], pts[i + 1],
                    _ARC_COLOR,
                    width=1.0,
                    replace_id=self._arc_ids[idx],
                    physics_client=self._pc,
                )
```

- [ ] **Step 6: Run tests — confirm they pass**

```bash
python3 -m pytest test_debug_markers.py -v
```

Expected: all passed.

- [ ] **Step 7: Run full test suite — confirm nothing broken**

```bash
python3 -m pytest test_kinematics.py test_sim_interface.py test_debug_markers.py -v
```

Expected: all passed.

- [ ] **Step 8: Commit**

```bash
git add viz/__init__.py viz/debug_markers.py test_debug_markers.py
git commit -m "feat: add DebugVisualizer — sagittal annulus arcs and life-limited hip→foot vectors

_arc_points: pure sagittal-plane semicircle geometry (18 segs, theta in [-pi/2, pi/2]).
Green lines: hip → actual foot (always). Red lines: hip → foot target (skipped at zero).
All p.* calls via sim.interface.add_debug_line per CLAUDE.md."
```

---

## Task 5: HeartBeat.py — Siclo1Controller refactor

**Files:**
- Modify: `HeartBeat.py`

**Background:** Add `viz_decimation` param, resolve hip link names in `_build_joint_map`, add `_warmup()`, update the init sequence (Direct: 50-cycle warmup; GUI: 2 s sleep + 5-cycle warmup + DebugVisualizer), wire visualiser into `_sync_gui()`, and replace the `VIZ_DECIMATION` constant reference in `step()` with `self.viz_decimation`.

- [ ] **Step 1: Remove the VIZ_DECIMATION module constant**

In `HeartBeat.py` at line 66, delete:

```python
# Visualisation decimation: update GUI every N physics cycles (10 Hz viz)
VIZ_DECIMATION: int = 10
```

- [ ] **Step 2: Update Siclo1Controller.__init__ signature**

Change:

```python
    def __init__(self, use_gui: bool = False):
```

to:

```python
    def __init__(self, use_gui: bool = False, viz_decimation: int = 10):
```

Immediately inside `__init__`, after `self.use_gui = use_gui`, add:

```python
        self.viz_decimation: int = viz_decimation  # cycles between GUI renders
        self._visualizer = None                    # set after warmup if GUI mode
```

- [ ] **Step 3: Fix physics connect (already done in Task 1) + add GUI sleep**

The physics connect was fixed in Task 1. Now add `time.sleep(2.0)` to the GUI setup block in `__init__`. Find the block starting with `# 7. If GUI, mirror the scene` and add the sleep immediately after `self.gui_client = p.connect(p.GUI)` succeeds:

```python
        # 2. Optional GUI viewer
        self.gui_client: Optional[int] = None
        if use_gui:
            try:
                self.gui_client = p.connect(p.GUI)
                time.sleep(2.0)  # X-server buffer — wait for window to appear (WSL)
            except Exception:
                self.gui_client = None
```

- [ ] **Step 4: Add hip link name resolution to _build_joint_map**

In `PyBulletInterface._build_joint_map`, after the `self._joint_list = list(self.joint_ids.items())` line, the method currently ends. However, the hip link names are needed by `Siclo1Controller._sync_gui()`, not by `PyBulletInterface`. Store them on `Siclo1Controller` directly.

Add these two lines at the very end of `Siclo1Controller.__init__`, after `self.pybullet.load_robot(urdf_path=urdf_file)`:

```python
        # Hip-pitch child link names (child of Left_Hip_Forwards / Right_Hip_Fowards).
        # Used by DebugVisualizer to read world-space hip positions.
        self._left_hip_link  = "Left_Upper_Leg_1"   # URDF-verified 2026-04-05
        self._right_hip_link = "Right_Upper_Leg_1"  # URDF-verified 2026-04-05
```

- [ ] **Step 5: Add _warmup method to Siclo1Controller**

Add the following method to `Siclo1Controller`, just before the `step()` method:

```python
    def _warmup(self, cycles: int) -> None:
        """Run full control pipeline for N cycles without real-time timing.

        Lets the robot settle under gravity before the 100 Hz loop starts.
        Gait commands are not issued (no gait planner integrated yet).
        The 10 ms timing guard does NOT apply here.
        """
        for _ in range(cycles):
            self.pybullet.read_sensors()
            self.pybullet.update_link_positions()
            perception.update_perception()
            stability.update_stability(dt=TARGET_DT)
            active_balance.update_active_balance()
            recovery.update_recovery()
            self.pybullet.apply_control()
            p.stepSimulation(physicsClientId=self.physics_client)
```

- [ ] **Step 6: Add warmup + DebugVisualizer creation to __init__**

At the end of `Siclo1Controller.__init__`, after the telemetry log lines and before the closing of `__init__`, add:

```python
        # Warmup: settle physics before real-time loop.
        # GUI mode gets fewer cycles (window already visible); Direct gets more.
        warmup_cycles = 5 if self.use_gui else 50
        self._warmup(warmup_cycles)
        self._telemetry_thread.log(f"  Warmup cycles: {warmup_cycles}")

        # Debug visualiser — GUI mode only
        if self.gui_client is not None:
            from viz.debug_markers import DebugVisualizer
            self._visualizer = DebugVisualizer(self.gui_client)
```

- [ ] **Step 7: Update step() to use self.viz_decimation**

In `step()`, find the GUI sync block (near the end of step, after telemetry):

```python
        # 15. Optional GUI sync (decimated — every VIZ_DECIMATION cycles)
        if (self.gui_client is not None and
                shared_state.cycle_count % VIZ_DECIMATION == 0):
            self._sync_gui()
```

Change to:

```python
        # 15. Optional GUI sync (decimated)
        if (self.gui_client is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._sync_gui()
```

- [ ] **Step 8: Wire DebugVisualizer into _sync_gui()**

At the end of `_sync_gui()`, after the joint mirroring loop, add:

```python
        # Update debug visualisation (annulus arcs + hip→foot vectors)
        if self._visualizer is not None:
            lp = self.shared_state.link_positions
            left_hip  = tuple(lp.get(self._left_hip_link,  [0.0, 0.0, 0.0]))
            right_hip = tuple(lp.get(self._right_hip_link, [0.0, 0.0, 0.0]))
            self._visualizer.update(self.shared_state, left_hip, right_hip)
```

- [ ] **Step 9: Run full test suite — confirm nothing broken**

```bash
python3 -m pytest test_kinematics.py test_sim_interface.py test_debug_markers.py -v
```

Expected: all passed.

- [ ] **Step 10: Commit**

```bash
git add HeartBeat.py
git commit -m "feat: refactor Siclo1Controller — viz_decimation, warmup, DebugVisualizer wiring

viz_decimation replaces VIZ_DECIMATION constant; passed from main.py.
_warmup(cycles): full pipeline, no real-time constraint.
  Direct: 50 cycles, GUI: 2 s X-server sleep + 5 cycles.
_sync_gui now calls DebugVisualizer.update() with hip world positions."
```

---

## Task 6: main.py — argparse entry point

**Files:**
- Create: `main.py`
- Create: `test_dual_mode_runner.py`

- [ ] **Step 1: Write the failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_dual_mode_runner.py`:

```python
"""Tests for main.py argument parsing and decimation computation."""
import pytest


def _parser():
    from main import _make_parser
    return _make_parser()


def test_default_no_gui():
    args = _parser().parse_args([])
    assert args.gui is False


def test_gui_flag_sets_true():
    args = _parser().parse_args(['--gui'])
    assert args.gui is True


def test_default_viz_hz_is_10():
    args = _parser().parse_args([])
    assert args.viz_hz == 10


def test_viz_hz_parsed_correctly():
    args = _parser().parse_args(['--gui', '--viz-hz', '33'])
    assert args.viz_hz == 33


def test_viz_decimation_33hz():
    from main import _viz_decimation
    assert _viz_decimation(33) == 3   # 100 // 33 = 3


def test_viz_decimation_10hz():
    from main import _viz_decimation
    assert _viz_decimation(10) == 10  # 100 // 10 = 10


def test_viz_decimation_clamps_above_100():
    from main import _viz_decimation
    assert _viz_decimation(200) == 1  # clamped to 100 → 100//100 = 1


def test_viz_decimation_clamps_below_1():
    from main import _viz_decimation
    assert _viz_decimation(0) == 100  # clamped to 1 → 100//1 = 100


def test_viz_decimation_1hz_gives_100():
    from main import _viz_decimation
    assert _viz_decimation(1) == 100
```

- [ ] **Step 2: Run to confirm they fail**

```bash
python3 -m pytest test_dual_mode_runner.py -v
```

Expected: all FAILED with `ModuleNotFoundError: No module named 'main'`

- [ ] **Step 3: Create main.py**

Create `/home/notlord/ros2_ws/Siclo1_V1/main.py`:

```python
"""Siclo1 simulation entry point.

Usage:
  python3 main.py                    # headless, 50-cycle warmup
  python3 main.py --gui              # GUI at 10 Hz render, 5-cycle warmup
  python3 main.py --gui --viz-hz 33  # GUI at ~33 Hz render
"""
import argparse
import sys
from HeartBeat import Siclo1Controller


def _make_parser() -> argparse.ArgumentParser:
    """Return the configured ArgumentParser. Separated for testability."""
    parser = argparse.ArgumentParser(
        description="Siclo1 bipedal robot simulation — 100 Hz physics heartbeat"
    )
    parser.add_argument(
        "--gui", action="store_true",
        help="Enable PyBullet GUI viewer and debug visualisation",
    )
    parser.add_argument(
        "--viz-hz", type=int, default=10, metavar="HZ",
        help="GUI render rate Hz, integer only (default: 10, range: 1-100)",
    )
    return parser


def _viz_decimation(viz_hz: int) -> int:
    """Compute cycle decimation from requested render rate.

    viz_hz: desired render rate (Hz). Clamped silently to [1, 100].
    Returns: physics cycles between GUI renders (= 100 // clamped_hz).
    """
    hz = max(1, min(100, viz_hz))
    return max(1, 100 // hz)


def main(argv=None) -> None:
    args = _make_parser().parse_args(argv)
    decimation = _viz_decimation(args.viz_hz)

    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation)
    try:
        controller.run(duration=30.0)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted.")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests — confirm they pass**

```bash
python3 -m pytest test_dual_mode_runner.py -v
```

Expected: all passed.

- [ ] **Step 5: Run full test suite**

```bash
python3 -m pytest test_kinematics.py test_sim_interface.py \
        test_debug_markers.py test_dual_mode_runner.py -v
```

Expected: all passed.

- [ ] **Step 6: Commit**

```bash
git add main.py test_dual_mode_runner.py
git commit -m "feat: add main.py — argparse entry point with --gui and --viz-hz flags

_make_parser(): testable parser factory.
_viz_decimation(hz): clamps [1,100], returns 100//hz.
Replaces the ad-hoc '--gui' in sys.argv check that was in HeartBeat.main()."
```

---

## Task 7: Progress/Custom_Command.md

**Files:**
- Create: `Progress/Custom_Command.md`

- [ ] **Step 1: Confirm Progress/ directory exists**

```bash
ls /home/notlord/ros2_ws/Siclo1_V1/Progress/
```

Expected: directory listing (it already exists from earlier sessions).

- [ ] **Step 2: Create the documentation file**

Create `/home/notlord/ros2_ws/Siclo1_V1/Progress/Custom_Command.md`:

```markdown
# Siclo1 — Custom Commands

## Entry Point

```bash
python3 main.py [FLAGS]
```

---

## Flags

### `--gui`

| Property | Value |
|---|---|
| Type | boolean (store_true) |
| Default | off — runs headless (`p.DIRECT`) |

**Effect when set:**
- Launches a second PyBullet client (`p.GUI`) as a visual window
- Enables debug visualisation: sagittal-plane annulus arcs (R_min / R_max) and hip→foot vectors
- Reduces physics warmup from 50 cycles to 5 cycles
- Adds a 2-second startup pause for the X-server window to appear (WSL)

---

### `--viz-hz HZ`

| Property | Value |
|---|---|
| Type | integer |
| Default | `10` |
| Range | 1–100 (silently clamped outside this range) |
| Requires | `--gui` (ignored in headless mode) |

**Effect:** Controls how often the GUI window refreshes.

The physics loop always runs at 100 Hz. The GUI renders every `100 ÷ viz_hz` physics cycles.

| `--viz-hz` | Render every N cycles | Effective GUI Hz |
|---|---|---|
| `1`  | 100 cycles | 1 Hz (slow-motion debug) |
| `10` | 10 cycles  | 10 Hz (default) |
| `33` | 3 cycles   | ~33 Hz (smooth) |
| `100`| 1 cycle    | 100 Hz (every cycle) |

---

## Examples

```bash
# Headless — maximum physics throughput, 50-cycle warmup
python3 main.py

# GUI at default 10 Hz render rate
python3 main.py --gui

# GUI at ~33 Hz render rate (smoother, higher GPU load)
python3 main.py --gui --viz-hz 33

# GUI at 1 Hz — slow-motion, useful for inspecting debug markers
python3 main.py --gui --viz-hz 1
```

---

## Debug Visualisation (GUI mode only)

When `--gui` is active, three overlays appear in the PyBullet window:

| Overlay | Colour | Meaning |
|---|---|---|
| Annulus arcs | Light blue | R_min (0.631 m) and R_max (0.743 m) reachable workspace per hip |
| Hip → Foot (actual) | Green | Current foot position relative to hip-pitch joint |
| Hip → Foot (target) | Red | IK foot target (hidden until gait planner sets a target) |

The arcs follow the robot as it moves. Lines are replaced in-place each render tick (no accumulation).
```

- [ ] **Step 3: Commit**

```bash
git add Progress/Custom_Command.md
git commit -m "docs: add Progress/Custom_Command.md — CLI reference for --gui and --viz-hz"
```

---

## Self-Review

**Spec coverage:**

| Spec requirement | Task |
|---|---|
| `--gui` flag (p.GUI vs p.DIRECT) | Task 6 (`main.py`) + Task 5 (`__init__`) |
| `--viz-hz` flag with decimation | Task 6 |
| `--viz-hz` silently clamped [1,100] | Task 6 |
| Direct mode: 50-cycle warmup | Task 5 |
| GUI mode: 5-cycle warmup + 2 s sleep | Task 5 |
| Frame dropper: `viz_decimation` in `step()` | Task 5 |
| Bug 1: physics must be `p.DIRECT` | Task 1 |
| Bug 2: `self.cycle_count` → `shared_state.cycle_count` | Task 1 |
| Bug 3: dead strobe block removed | Task 1 |
| `add_debug_line` + `remove_debug_line` in `sim/interface.py` | Task 2 |
| `left_foot_target`, `right_foot_target` in `Siclo1State` | Task 3 |
| `DebugVisualizer`: annulus arcs + life-limited vectors | Task 4 |
| Annulus: sagittal-plane, light blue, follows robot | Task 4 |
| Red lines skipped when target is zero vector | Task 4 |
| `viz/debug_markers.py` must not import pybullet directly | Task 4 (`_arc_points` is pure; drawing goes via `sim.interface.add_debug_line`) |
| `Progress/Custom_Command.md` | Task 7 |

**Placeholder scan:** None found. Every step has complete code.

**Type consistency:**
- `add_debug_line(from_xyz, to_xyz, color_rgb, width, replace_id, physics_client) → int` — defined Task 2, used in Task 4 correctly
- `DebugVisualizer(physics_client: int)` — defined Task 4, instantiated in Task 5 correctly
- `DebugVisualizer.update(state, left_hip: tuple, right_hip: tuple)` — defined Task 4, called in Task 5 with `tuple(lp.get(...))` correctly
- `_viz_decimation(viz_hz: int) → int` — defined Task 6, tested Task 6 correctly
- `Siclo1Controller(use_gui, viz_decimation)` — signature updated Task 5, called from `main.py` Task 6 correctly
