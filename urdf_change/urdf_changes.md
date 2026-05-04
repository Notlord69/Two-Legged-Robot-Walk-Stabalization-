# Siclo1 URDF Change Log

**Reference Side:** Left Leg is the primary reference for all kinematic calculations and link length measurements.  
All Right-leg joint origins, limits, and inertial properties must be brought into symmetry with the Left-leg values unless an explicit asymmetry is documented and justified below.

**Scope:** `Siclo1.urdf` only. No mesh files, no Fusion 360 sources.  
**Policy:** Every structural change (joint limit, mass, inertia, origin xyz/rpy, visual/collision tag) must produce one entry in this log before the change is committed.

---

## Log Format

```
### [ITER-NNN] <Joint or Link Name> — <Field Changed>
| Field        | Original Value | Updated Value | Unit |
|---|---|---|---|
| xyz          | ...            | ...           | m    |
**Technical Justification:** ...
**Timestamp:** YYYY-MM-DD | Iteration: NNN
**Status:** PENDING | APPLIED | REVERTED
```

---

## Canonical Segment Lengths (Reference — Left Leg)

Derived from URDF `<origin>` Euclidean norm. Updated 2026-05-04 (ITER-003/004).

| Segment      | Joint                | xyz (in parent frame)  | ‖xyz‖ (m)  |
|---|---|---|---|
| Thigh        | Left_Knee            | 0.0, 0.0, -0.390000    | 0.390000   |
| Shank        | Left_Ankle           | 0.0, 0.0, -0.360000    | 0.360000   |

Derived IK constants:
- `R_min = |L_thigh − L_shank| + 0.005 = 0.035000 m`
- `R_max = L_thigh + L_shank − 0.005 = 0.745000 m`
- Ratio: 1.083:1 (thigh-dominant)
- Total leg span: 0.750 m (75 cm)

---

## Pending Changes

_(none)_

---

### [ITER-003] Left_Knee + Right_Knee — origin xyz (thigh length)
| Field | Original Value | Updated Value | Unit |
|---|---|---|---|
| xyz (both knees) | `0.0  0.0  -0.060661` | `0.0  0.0  -0.390000` | m |

**Technical Justification:** Previous thigh length of 0.060661 m (6.07 cm) was a Fusion 360 export artifact yielding a 1:11.3 thigh-to-shank ratio. Corrected to 0.390 m (39 cm) for a biomechanically valid 1.083:1 ratio optimised for walking, running, and stair climbing. Total leg length: 75 cm. Hip flexion for 18 cm stair step = 28° (vs 31° at equal segments). Design spec: `docs/superpowers/specs/2026-05-04-leg-geometry-redesign.md`.  
**Timestamp:** 2026-05-04 | Iteration: 003  
**Status:** APPLIED — 2026-05-04

---

### [ITER-004] Left_Ankle + Right_Ankle — origin xyz (shank length)
| Field | Original Value | Updated Value | Unit |
|---|---|---|---|
| xyz (both ankles) | `0.0  0.0  -0.686961` | `0.0  0.0  -0.360000` | m |

**Technical Justification:** Previous shank length of 0.686961 m (68.70 cm) was the counterpart to the disproportionate thigh. Corrected to 0.360 m (36 cm). Shorter distal segment reduces swing-phase rotational inertia for running. R_min = 0.035 m, R_max = 0.745 m. Design spec: `docs/superpowers/specs/2026-05-04-leg-geometry-redesign.md`.  
**Timestamp:** 2026-05-04 | Iteration: 004  
**Status:** APPLIED — 2026-05-04

---

### [ITER-005] Left_Upper_Leg_1 + Right_Upper_Leg_1 — visual/collision geometry
| Field | Original Value | Updated Value | Unit |
|---|---|---|---|
| geometry | `mesh Left_Upper_Leg_1.stl scale=0.001` | `cylinder length=0.390 radius=0.025` | m |
| visual origin xyz | `-0.86875 -0.2 -0.879703` / `-0.23125 -0.2 -0.917797` | `0 0 -0.195` | m |

**Technical Justification:** STL mesh Z-span was 29 cm at scale 0.001 — already mismatched to the 6 cm kinematic chain and now mismatched to the 39 cm chain. Replaced with cylinder primitive centred at Z=-0.195 (midpoint of thigh). Inertial mass and tensor unchanged. Mesh will be restored when Fusion 360 re-exports the updated part.  
**Timestamp:** 2026-05-04 | Iteration: 005  
**Status:** APPLIED — 2026-05-04

---

### [ITER-006] Left_Lower_Leg_1 + Right_Lower_Leg_1 — visual/collision geometry
| Field | Original Value | Updated Value | Unit |
|---|---|---|---|
| geometry | `mesh Left_Lower_Leg_1.stl scale=0.001` | `cylinder length=0.360 radius=0.020` | m |
| visual origin xyz | `-0.89975 -0.200969 -0.82757` / `-0.12525 -0.186308 -0.909207` | `0 0 -0.180` | m |

**Technical Justification:** STL mesh Z-span was 61 cm at scale 0.001 — mismatched to both the old 69 cm and new 36 cm chain. Replaced with cylinder primitive centred at Z=-0.180 (midpoint of shank). Radius 20 mm (slightly thinner than thigh). Inertial mass and tensor unchanged.  
**Timestamp:** 2026-05-04 | Iteration: 006  
**Status:** APPLIED — 2026-05-04

---

## Applied Changes

### [ITER-001] Right_Knee — origin xyz
| Field | Original Value              | Updated Value         | Unit |
|---|---|---|---|
| xyz   | -0.106, -0.013692, -0.00859 | 0.0, 0.0, -0.060661  | m    |

**Technical Justification:** Right_Upper_Leg_1 frame was exported from Fusion 360 with X-axis aligned to the leg's "down" direction instead of Z. The resulting ‖xyz‖ = 0.107225 m deviates from the Left reference thigh (0.060661 m) by 46.6 mm — an asymmetric export artifact, not a design intent. Corrected origin sets Z-dominant offset equal to canonical thigh length. IK workspace R_max_Right will match R_max_Left after correction.  
**Timestamp:** 2026-04-04 | Iteration: 001  
**Status:** APPLIED — 2026-04-04

---

### [ITER-002] Right_Ankle — origin xyz
| Field | Original Value              | Updated Value          | Unit |
|---|---|---|---|
| xyz   | -0.025, 0.010133, -0.758742 | 0.0, 0.0, -0.686961   | m    |

**Technical Justification:** Right_Lower_Leg_1 shank ‖xyz‖ = 0.759221 m deviates from Left reference shank (0.686961 m) by 72.3 mm. Correcting to canonical Left shank length ensures equal R_max on both sides (0.742622 m). Z-dominant placement mirrors Left_Ankle convention.  
**Timestamp:** 2026-04-04 | Iteration: 002  
**Status:** APPLIED — 2026-04-04

---

## Reverted Changes

_(none yet)_

## 2026-04-28 — Drift & Yaw Fix (Symmetry Correction)

**Design spec:** `docs/superpowers/specs/2026-04-28-drift-yaw-fix-design.md`

### Problem

Left leg had spurious X/Y offsets on Knee and Ankle joint origins (Fusion 360 export artifact). Right leg geometry was clean. `Right_Hip_Twist` had an asymmetric lower limit (-2 deg instead of -20 deg).

### Joint Origin Changes

| Joint | Field | Before | After | Rationale |
|-------|-------|--------|-------|-----------|
| `Left_Knee` | origin xyz | `0.031 0.000969 -0.052133` | `0.0 0.0 -0.060661` | Match right leg's clean vertical geometry |
| `Left_Ankle` | origin xyz | `0.1 0.024043 -0.679218` | `0.0 0.0 -0.686961` | Match right leg's clean vertical geometry |
| `Right_Hip_Twist` | limit lower | `-0.034907` | `-0.349066` | Match left hip twist symmetric +/-20 deg range |
| `Left_Hip_Twist` | origin xyz Z | `0.98` | `1.018094` | Compensate for left foot height shift (+0.038094 m) |

### Height Compensation

Both Hip_Forwards frames land at Z = 0.917797 m. Both ankles at Z = 0.170175 m.

### What Was NOT Changed

- Joint axes, link masses, link inertia tensors
- Hip_Inwards / Hip_Forwards origins (within tolerance)
- Ankle axis Y-component (+/-0.005365, already mirrored correctly)
