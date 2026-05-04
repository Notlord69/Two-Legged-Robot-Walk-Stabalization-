# Leg Geometry Redesign — Thigh/Shank Ratio Correction

**Date:** 2026-05-04  
**Status:** Approved — pending implementation  
**Scope:** `Siclo1.urdf` joint origins only (Left_Knee, Left_Ankle, Right_Knee, Right_Ankle)

---

## Problem

The URDF leg chain is severely disproportionate as exported from Fusion 360:

| Segment | Current length | Current ratio |
|---------|---------------|---------------|
| Thigh (hip_fwd → knee) | 0.060661 m (6.07 cm) | 1 |
| Shank (knee → ankle)   | 0.686961 m (68.70 cm) | 11.3 |

A 1:11.3 thigh-to-shank ratio is an export artifact. It produces:
- Knee that operates near full extension at all times (degenerate IK workspace)
- No meaningful hip torque leverage for stair climbing
- LIPM and WBC equations that assume roughly equal segments fail to converge at realistic joint angles

---

## Goals

The robot must support three locomotion modes:

1. **Walking** — LIPM-based balance; benefits from near-equal segments
2. **Running** — SLIP-style spring leg; benefits from low distal inertia (short shank) and equal swing mass
3. **Stair climbing** — step height ~18 cm; benefits from longer thigh (hip extension leverage) and sufficient shank for swing clearance

---

## Decision: Option B — Moderate thigh-dominant (1.083:1)

**Thigh = 39 cm (0.39 m) | Shank = 36 cm (0.36 m) | Total = 75 cm (0.75 m)**

### Rationale

| Option | Ratio | Total | Walking | Running | Stairs |
|--------|-------|-------|---------|---------|--------|
| A — Equal | 1:1 | 70 cm | Best | Best | Adequate |
| **B — Moderate thigh-dominant** | **1.083:1** | **75 cm** | **Very good** | **Good** | **Best** |
| C — Strong thigh-dominant | 1.25:1 | 72 cm | Acceptable | Degraded | Partial |

Option B is selected because:
- **Stair push-up**: hip flexion for 18 cm step ≈ 28° (vs 31° for equal segments) — more hip extension torque budget
- **Running swing**: shank is the shorter segment (36 cm), keeping distal inertia low; net swing efficiency near Option A
- **Walking/LIPM**: 1.083:1 ratio is within WBC compensation tolerance
- **Standing height preserved**: total 75.00 cm vs current 74.76 cm — delta of 2.4 mm
- **Closest to human reference**: femur:tibia ≈ 1.05:1; Option B at 1.083:1 matches the biomechanical archetype
- **Whole-integer cm segments**: both 39 cm and 36 cm are clean values

### IK Workspace After Option B

| Metric | Current | After Option B |
|--------|---------|----------------|
| R_min (with 5 mm margin) | 0.631 m | 0.035 m |
| R_max (with 5 mm margin) | 0.742 m | 0.745 m |
| Knee angle at ~0.55 m hip height | ~167° (near-singular) | ~88° (mid-range healthy) |

---

## URDF Changes

All four joint origins are set to a clean vertical Z-axis vector. X and Y components are zeroed (consistent with ITER-001/002 convention).

| Joint | Before | After |
|-------|--------|-------|
| `Left_Knee` origin xyz | `0.0  0.0  -0.060661` | `0.0  0.0  -0.390000` |
| `Right_Knee` origin xyz | `0.0  0.0  -0.060661` | `0.0  0.0  -0.390000` |
| `Left_Ankle` origin xyz | `0.0  0.0  -0.686961` | `0.0  0.0  -0.360000` |
| `Right_Ankle` origin xyz | `0.0  0.0  -0.686961` | `0.0  0.0  -0.360000` |

### What Does NOT Change

- Joint axes, joint limits (effort, velocity, angular range)
- Link masses and inertia tensors
- Hip joint origins (Yaw, Inwards, Forwards) and ankle axis vectors
- STL mesh files (visual mismatch is expected until Fusion 360 meshes are regenerated)
- `shared_state.py` field names

### Downstream Code Impact

Any code that hardcodes thigh/shank lengths must be updated:

| Constant | Old value | New value |
|----------|-----------|-----------|
| `L_THIGH` (or equivalent) | 0.060661 m | 0.390000 m |
| `L_SHANK` (or equivalent) | 0.686961 m | 0.360000 m |
| `R_min` | 0.631300 m | 0.035000 m |
| `R_max` | 0.742622 m | 0.745000 m |

---

## Change Log Entries

Two entries are added to `urdf_change/urdf_changes.md`:

- **ITER-003**: Left_Knee + Right_Knee origin xyz (thigh length correction)
- **ITER-004**: Left_Ankle + Right_Ankle origin xyz (shank length correction)

A copy of the modified `Siclo1.urdf` is placed in `urdf_change/` alongside the log.

---

## Out of Scope

- Mesh regeneration (Fusion 360 work, separate task)
- Gait controller re-tuning (separate task after URDF is validated in simulation)
- Inertia tensor recalculation (tensors reflect physical hardware; segment lengths are kinematic)
