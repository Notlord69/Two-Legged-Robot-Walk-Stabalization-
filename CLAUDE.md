# Siclo1 — Project Context

**Scope:** Work only inside `/home/notlord/ros2_ws/Siclo1_V1`. Ignore everything outside.

---

## What This Project Is
Bipedal robot simulation (8 kg, custom Fusion 360 URDF). Simulation only — no hardware, no embedded targets. Every implementation must be physically valid in the real world. Flag anything that isn't.

---

## Simulator & Language
- **Now:** PyBullet | **Future:** ROS2 + Gazebo
- All PyBullet API calls must go through an abstraction layer (`sim/interface.py`). Never call `p.getJointState()` or `p.applyExternalTorque()` inline in control logic. This is what enables the future Gazebo swap.
- **Primary language:** Python 3.10+. C++ only when Python can't meet timing requirements.

---

## Core Assets — Do Not Break
- `Siclo1.urdf` — Do not regenerate or restructure. Changes need explicit confirmation.
- `shared_state.py` — Single source of truth (`Siclo1State`). All inter-module data lives here. Do not rename existing fields. New fields are fine but must be documented.
- Joint names, link masses, and COM values must be derived from the URDF — never hardcoded.

---

## Control Rules
- **100 Hz loop — 10 ms hard limit.** Every function inside the loop needs a timing guard. Violations trigger Safe Freeze, not a warning.
- **No rogue variables.** If a value persists across loop iterations, it belongs in `Siclo1State`.
- **Module priority:** Safety → Balance (LIPM/Capture Point) → Load Stability → Gait/Swing → WBC.

---

## Code Standards
- Every constant needs a unit and physical meaning in its comment. `kp = 45.0` is rejected. `HIP_TORQUE_KP = 45.0  # N·m/rad` is accepted.
- No bare `except` on physics logic. Validate all sensor reads, IK solutions, and URDF queries before use.
- If something is physically wrong or will fail under realistic conditions — say so first, then fix it.

---

##Agent Constraints
Workflow Mode: Use "Linked Chat" by default. Do not spawn a sub-agent unless the task involves complex mathematical optimization (e.g., Gait Inverse Kinematics) or unless specify by the user. (maximum sub-agent 1)

---
## Response Format
End every code analysis or implementation with:
```
KEY POINT: [one sentence — the concept to retain]
KEY LINE:  [the single most important line, with inline comment]
```
## LLM Wiki (Obsidian Knowledge Base)

### Paths
- **Wiki root**: `C:\Siclo1_V1_Vault\Siclo1 Brain\`
- **WSL workspace**: `\\wsl.localhost\Ubuntu-22.04\home\notlord\ros2_ws\Siclo1_V1\`
- **Raw source mirror**: `C:\Siclo1_V1_Vault\Siclo1 Brain\raw\`
- **Sync script**: `C:\Siclo1_V1_Vault\Siclo1 Brain\scripts\sync-from-wsl.sh`

### Mandatory Session Protocol

**On every session start**, before writing any code or making any plan:
1. Read the wiki. Navigate `Siclo1 Brain/` and read all relevant notes.
2. Cross-reference the task against what the wiki says. If there is a conflict between the wiki and the user's description, surface it before proceeding.

**Access rules (strictly enforced):**
- Wiki is **read-only**. Claude must never edit, delete, or create files inside `Siclo1 Brain/`, with one exception below.
- Claude may **execute** `scripts/sync-from-wsl.sh` when a refresh is warranted.

### When to Trigger a Wiki Refresh

Run `bash "C:\Siclo1_V1_Vault\Siclo1 Brain\scripts\sync-from-wsl.sh"` when any of the following is true:
- A file or module is referenced in conversation that does not appear in `raw/`
- The user mentions recent changes (new files, refactors, renames) not reflected in the wiki
- The timestamps in `raw/` are older than the session warrants for an active dev task
- Claude is about to make architectural decisions and the wiki state feels uncertain

After running the sync, re-read any notes that reference the updated files before continuing.

### After a Sync

Claude does **not** write or update any `.md` notes in the wiki. Ingestion and wiki note generation is a separate, manual or scripted step owned by the user. Claude's job is only to:
1. Trigger the sync script when warranted
2. Read the freshly synced `raw/` files directly as ground truth