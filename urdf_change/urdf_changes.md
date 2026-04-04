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

Derived from URDF `<origin>` Euclidean norm. Verified 2026-04-04.

| Segment      | Joint                | xyz (in parent frame)       | ‖xyz‖ (m)  |
|---|---|---|---|
| Thigh        | Left_Knee            | 0.031, 0.000969, -0.052133  | 0.060661   |
| Shank        | Left_Ankle           | 0.1, 0.024043, -0.679218    | 0.686961   |

Derived IK constants:
- `R_min = |L_thigh − L_shank| + 0.005 = 0.631300 m`
- `R_max = L_thigh + L_shank − 0.005 = 0.742622 m`

---

## Pending Changes

_(none)_

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
