# POSITION_CONTROL Warmup — Design Spec

**Date:** 2026-05-03
**Status:** Approved, pending implementation

---

## Problem

The WBC (TORQUE_CONTROL PD) cannot safely initialize the robot from a cold spawn. On the first physics step after `resetJointState`, PyBullet's constraint solver resolves foot-ground contact by creating joint velocities that exceed `effort_limit / KD = 100 / 10 = 10 rad/s`. This drives the KD term into saturation, turning the controller effectively bang-bang (±100 N·m) from cycle 1 onward, regardless of KP/KD values. The robot is launched to altitude within 0.5s every run.

PyBullet's internal POSITION_CONTROL is unconditionally stable under contact impulses. Using it during warmup (50 cycles, 0.5s) allows the robot to settle at stance before the WBC takes over.

---

## Scope

POSITION_CONTROL is active **only during the 50-cycle warmup**. The real simulation loop uses TORQUE_CONTROL (WBC with KP=30, KD=10) throughout — including the IDLE mission state. No mode switching in the real loop.

---

## Design

### New methods on `PyBulletRobot` (HeartBeat.py)

**`enter_position_mode(target_angles: dict) -> None`**

Called once before the warmup loop. Sets all 8 controlled leg joints to PyBullet `POSITION_CONTROL`:

```
p.setJointMotorControl2(
    robot_id, jid,
    controlMode=p.POSITION_CONTROL,
    targetPosition=angle,
    targetVelocity=0.0,
    maxVelocity=1.0,      # rad/s — gentle settling, avoids overshoot
    force=50.0,           # N·m — sufficient to hold stance, half effort limit
    physicsClientId=pc,
)
```

`target_angles` comes from `gait_planner.get_idle_stance_angles()` — the same dict used by `set_stance_pose()`.

`maxVelocity=1.0 rad/s`: at maximum rate, a 1.22 rad cold-start error takes ~1.2s to resolve. Over 50 warmup cycles (0.5s), joints reach ~40% of their target. Remaining error (~0.73 rad) is well within KP=30 stable range (τ = 22 N·m, below 100 N·m effort limit) when the WBC takes over.

`force=50.0 N·m`: enough to support stance against gravity (estimated hip gravity torque < 10 N·m). Not so high it creates contact instability.

**`restore_torque_mode() -> None`**

Called once after the warmup loop. Re-sets all 8 controlled joints to `VELOCITY_CONTROL force=0` — identical to what `_build_joint_map()` does at URDF load. Hands joints back to the WBC.

```
p.setJointMotorControl2(
    robot_id, jid,
    controlMode=p.VELOCITY_CONTROL,
    force=0.0,
    physicsClientId=pc,
)
```

### Modified `_warmup()` in `Siclo1Controller`

**Before loop:** call `self.pybullet.enter_position_mode(get_idle_stance_angles())`

**Loop body (50 cycles):**
```
read_sensors()
update_link_positions()
perception.update_perception()
stability.update_stability(dt=TARGET_DT)
balance_controller.update_balance()
sim.interface.step_simulation(physics_client)
```

Removed from loop: `grf.update_grf()`, `gait_planner.update_gait_planner()`, `self._mission.update()`, `self._wbc_step()`, `recovery.update_recovery()`, `self.pybullet.apply_control()`. These are TORQUE_CONTROL-dependent and do not contribute to module state initialization.

**After loop:**
```
self.pybullet.restore_torque_mode()
gait_planner.reset_gait_planner()
balance_controller.reset_balance()
```

`restore_torque_mode()` must be called before `reset_gait_planner()` so joints are in the correct mode when the first real `apply_control()` fires.

**Removed:** both `set_stance_pose()` calls added in the previous session. POSITION_CONTROL renders them redundant.

### Spawn height

`URDF_SPAWN_Z = -0.14` (unchanged from last session). Feet at z≈0 (2.5mm contact penetration) — gentle for POSITION_CONTROL settling.

---

## Not Changed

- `WBC_KP = 30.0 N·m/rad`, `WBC_KD = 10.0 N·m·s/rad` — correct values, stay.
- `get_idle_stance_angles()` in `gait_planner.py` — used by `enter_position_mode()`.
- `set_stance_pose()` on `PyBulletRobot` — keep the method (may be useful for debugging/reset scenarios), but no longer called from `_warmup()`.
- The real simulation loop (`step()`) — unchanged.
- All existing tests — no breakage expected.

---

## Verification

Run `python3 main.py --duration 1000` and confirm:

1. **No explosion:** COM stays below 2m for the entire run
2. **Standing:** COM_z stabilizes between 0.65–0.90m within the first 1.0s
3. **No oscillation:** Hip torques don't alternate ±100 N·m every cycle
4. **Contact forces:** Both feet show > 5N within first 2.0s
5. **Torque telemetry:** Columns 45-50 show non-zero values (confirms WBC is active post-warmup)

Smoke test commands (from previous spec):
```bash
LATEST=$(ls -td sessions/2026-* | head -1)
awk -F',' 'NR>1{z=$6+0; if(z>2.0) print "FAIL: COM_z="z" at t="$1}' "$LATEST/telemetry.csv" | head -3
awk -F',' 'NR>1 && NR<12{printf "t=%.2f tau_L_hip=%.1f tau_R_hip=%.1f\n", $1, $46, $49}' "$LATEST/telemetry.csv"
awk -F',' 'NR>1 && $1+0>1.0 && $1+0<1.5{printf "t=%.2f L=%.1f R=%.1f\n", $1, $13, $14}' "$LATEST/telemetry.csv"
```

---

## Testing

Three new unit tests in `Test_Enviroment/test_heartbeat_strict.py` or a new `test_position_warmup.py`:

1. `test_enter_position_mode_exists` — `PyBulletRobot` has `enter_position_mode` method
2. `test_restore_torque_mode_exists` — `PyBulletRobot` has `restore_torque_mode` method
3. `test_warmup_does_not_call_wbc` — mock `_wbc_step` and verify it is not called during `_warmup()`

Integration verification is via the smoke test (simulation run).
