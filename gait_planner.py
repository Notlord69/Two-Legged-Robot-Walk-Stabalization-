"""
================================================================================
PROJECT SICLO1 — GAIT PLANNER  (gait_planner.py)
================================================================================

5-State Step-Phase FSM:
    DOUBLE_SUPPORT → COM_SHIFT → LIFT → SWING → PLACE → DOUBLE_SUPPORT

Phase advance per cycle:
    step_phase_timer += dt on every cycle.
    Each phase handler checks exit conditions and calls _transition_to().

Stance IK anchor (all phases, every cycle, stance leg only in LIFT/SWING/PLACE):
    stance_foot_rel = stance_foot_world_pos - stance_hip_pos
    ik_stance_angles = kinematics.solve_ik(stance_foot_rel, stance_side)

Swing arc (SWING and PLACE phases only):
    z_swing = SWING_HEIGHT * 4 * φ * (1 - φ)
    x_swing = x_stance + (x_target - x_stance) * φ

INPUTS (shared_state):
    step_phase, step_phase_timer, stance_side, stance_foot_world_pos,
    capture_point, swing_phase, swing_foot_x_stance,
    left/right_foot_{position,velocity,force,contact_state},
    stability_status, mission_state, ramp_gain, freeze_robot, link_positions,
    last_dt

OUTPUTS (shared_state):
    step_phase, step_phase_timer, swing_phase, swing_foot_x_stance,
    swing_foot_target, left/right_foot_target,
    ik_left_angles, ik_right_angles,
    step_count, active_swing_side, stance_side,
    stance_foot_world_pos, freeze_robot

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
import numpy as np

import kinematics
import recovery
from shared_state import (
    shared_state, MissionState, StepPhase, ContactState, StabilityStatus,
    ERR_PHASE_TIMEOUT,
)
print("GAIT_PLANNER LOADED — version with DS gate")

# ============================================================================
# CONSTANTS
# ============================================================================

STEP_LENGTH:       float = 0.12   # m, fixed sagittal advance per step (nominal)
STEP_TIMING_SCALE: float = 0.5    # dimensionless, blend factor for CP correction

SWING_HEIGHT:   float = 0.04   # m, peak foot clearance above ground at φ=0.5
SWING_DURATION: float = 0.40   # s, full swing phase (40 cycles at 100 Hz)

# Phase timeout constants
DS_MIN_TIME:           float = 0.10   # s, minimum double-support duration
DS_TIMEOUT:            float = 2.0    # s, DS timeout → freeze_robot
COM_SHIFT_TIMEOUT:     float = 1.0    # s, COM_SHIFT timeout → conditional
COM_SHIFT_THRESHOLD:   float = 0.03   # m, |CP_x − stance_foot_x| to exit COM_SHIFT
LIFT_TIMEOUT:          float = 0.15   # s, LIFT timeout → conditional
SWING_TIMEOUT_FACTOR:  float = 1.5    # ×SWING_DURATION before force-advance to PLACE
PLACE_TIMEOUT:         float = 0.5    # s, PLACE timeout → conditional

# Physical thresholds
SWING_UNLOAD_THRESHOLD: float = 5.0    # N, swing foot considered empty below this
STANCE_LOAD_THRESHOLD:  float = 60.0   # N, ~77% of 78 N body weight; stance must carry
                                        #    this before LIFT is permitted
FORCE_BALANCE_RATIO:    float = 2.0    # dimensionless, max allowed ratio max(F)/min(F) at DS→COM_SHIFT
FORCE_BALANCE_FLOOR:    float = 10.0   # N, minimum per-foot force before ratio check applies
SETTLE_VEL_THRESHOLD:   float = 0.05   # m/s, foot considered settled below this
PLACE_ENTRY_PHI:        float = 0.85   # dimensionless, phi at which SWING → PLACE

# Hip link names in shared_state.link_positions (verified HeartBeat.py 2026-04-05)
_LEFT_HIP_LINK:  str = "Left_Upper_Leg_1"
_RIGHT_HIP_LINK: str = "Right_Upper_Leg_1"


# ============================================================================
# HELPERS
# ============================================================================

def _swing_z(phi: float) -> float:
    """Cycloidal foot height during swing.

    phi: normalized swing phase ∈ [0, 1]
    Returns z_foot (m) above ground; 0 at start and end, SWING_HEIGHT at mid.

    Uses cycloidal profile: z = H*(1 - cos(2π·φ))/2
    Velocity dz/dt = 0 at both endpoints — zero impact velocity at touchdown and
    zero yank at liftoff.  The old parabolic profile (4·H·φ·(1-φ)) had
    ±0.4 m/s at the endpoints (±4·H/SWING_DURATION), which drove an impact
    spike on every foot contact.

    Boundary check (float-exact): z(0)=0, z(0.5)=H, z(1)=0.
    """
    return SWING_HEIGHT * (1.0 - math.cos(2.0 * math.pi * phi)) / 2.0


def _compute_x_target(capture_point_x: float, decel: bool = False) -> float:
    """Compute foot landing x-position from Capture Point.

    capture_point_x: x-component of CP in world frame (m), pre-clamped
    decel: True in DECEL state → halve STEP_LENGTH to absorb stopping impulse
    Returns target x (m, world frame).
    """
    step = STEP_LENGTH * 0.5 if decel else STEP_LENGTH
    return capture_point_x * STEP_TIMING_SCALE + step


def _clamped_cp_x() -> float:
    """Read capture_point[0] from shared_state, clamped to ±0.20 m.

    Clamped to prevent IK workspace violations during erratic motion.
    ±0.20 m covers the full physically valid recovery range for this robot.
    """
    cp_x = float(getattr(shared_state, 'capture_point', np.zeros(2))[0])
    return float(np.clip(cp_x, -0.20, 0.20))


# ============================================================================
# GAIT PLANNER CONTROLLER
# ============================================================================

class GaitPlannerController:
    """Per-cycle gait planner FSM.  All state lives in shared_state."""

    def __init__(self):
        self._ds_lock_pending: bool = True  # lock stance foot on first DS cycle

    # ── Public entry point ────────────────────────────────────────────────────

    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        if (shared_state.freeze_robot or
                shared_state.mission_state == MissionState.IDLE):
            return

        # Mid-cycle overrun guard: if the prior stages (sensors, stability) already
        # exceeded the 10 ms budget, the sensor data for this cycle is from an
        # unusually long frame.  Hold the current phase and IK angles rather than
        # advancing on potentially stale data.
        if shared_state.timing_violation_this_cycle:
            return

        dt = shared_state.last_dt
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01   # fallback to 100 Hz nominal

        # Advance phase timer every cycle before dispatching
        shared_state.step_phase_timer += dt

        phase = shared_state.step_phase
        if phase == StepPhase.DOUBLE_SUPPORT:
            self._handle_double_support(dt)
        elif phase == StepPhase.COM_SHIFT:
            self._handle_com_shift(dt)
        elif phase == StepPhase.LIFT:
            self._handle_lift(dt)
        elif phase == StepPhase.SWING:
            self._handle_swing(dt)
        elif phase == StepPhase.PLACE:
            self._handle_place(dt)

    # ── Transition helpers ────────────────────────────────────────────────────

    def _transition_to(self, phase: StepPhase) -> None:
        """Advance FSM to phase: reset timer, reset step watchdog."""
        if phase == StepPhase.DOUBLE_SUPPORT:
            self._ds_lock_pending = True
        shared_state.step_phase       = phase
        shared_state.step_phase_timer = 0.0
        recovery.reset_step()

    def _abort_to_double_support(self) -> None:
        """Conditional timeout abort: preserve stance/swing sides, retry same step."""
        shared_state.swing_phase = 0.0
        self._transition_to(StepPhase.DOUBLE_SUPPORT)

    # ── Stance IK anchor (used every cycle for stance leg) ───────────────────

    def _compute_stance_ik(self) -> None:
        """Recompute stance leg IK from locked world-frame anchor.

        Runs every cycle in every phase for the stance leg.
        Stance foot is never frozen — WBC always gets a valid target.
        """
        stance_side = shared_state.stance_side
        hip_key = (_LEFT_HIP_LINK if stance_side == "left"
                   else _RIGHT_HIP_LINK)
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        stance_foot_rel = shared_state.stance_foot_world_pos - hip_pos
        rel_z = float(stance_foot_rel[2])
        if not (-1.0 < rel_z < 0.0):
            return   # invalid geometry (robot falling); hold last angles
        foot_xyz_rel = (
            float(stance_foot_rel[0]),
            float(stance_foot_rel[1]),
            rel_z,
        )
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, stance_side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)
        if stance_side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

    def _lock_stance_foot(self) -> None:
        """Snapshot current stance foot world position into stance_foot_world_pos.

        Called exactly once per stance entry (DOUBLE_SUPPORT entry).
        """
        stance_side = shared_state.stance_side
        foot_pos = (shared_state.left_foot_position  if stance_side == "left"
                    else shared_state.right_foot_position)
        shared_state.stance_foot_world_pos = foot_pos.copy()

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _handle_double_support(self, dt: float) -> None:
        lf = shared_state.left_foot_force
        rf = shared_state.right_foot_force
        both_confirmed = shared_state.both_feet_in_contact()
        print(f"[GATE] timer={shared_state.step_phase_timer:.3f} "
          f"ramp={shared_state.ramp_gain:.2f} "
          f"both={both_confirmed} "
          f"F=[{lf:.0f},{rf:.0f}] "
          f"ratio={max(lf,rf)/min(lf,rf) if min(lf,rf)>0 else 999:.2f}")
          
        # Lock stance foot exactly once on DS entry
        if self._ds_lock_pending:
            self._lock_stance_foot()
            self._ds_lock_pending = False

        self._compute_stance_ik()

        timer = shared_state.step_phase_timer
        if timer >= DS_TIMEOUT:
            shared_state.freeze_robot = True
            return

        # Fix 4: soft ramp gate — DS timer counts during RAMP but no advance until full torque
        if shared_state.ramp_gain < 1.0:
            return

        both_confirmed = shared_state.both_feet_in_contact()
        if both_confirmed and timer >= DS_MIN_TIME:
            # Fix 3: force balance gate — neither foot carrying > FORCE_BALANCE_RATIO× the other
            lf = shared_state.left_foot_force
            rf = shared_state.right_foot_force
            if (lf >= FORCE_BALANCE_FLOOR and rf >= FORCE_BALANCE_FLOOR and
                    max(lf, rf) / min(lf, rf) <= FORCE_BALANCE_RATIO):
                self._transition_to(StepPhase.COM_SHIFT)

    def _handle_com_shift(self, dt: float) -> None:
        self._compute_stance_ik()

        timer    = shared_state.step_phase_timer
        stable   = (shared_state.stability_status != StabilityStatus.UNSTABLE)
        cp_close = (np.linalg.norm(
            shared_state.capture_point - shared_state.stance_foot_world_pos[:2]
        ) < COM_SHIFT_THRESHOLD)

        if stable and cp_close:
            self._transition_to(StepPhase.LIFT)
            return

        if timer >= COM_SHIFT_TIMEOUT:
            swing_force = self._swing_foot_force()
            if swing_force > SWING_UNLOAD_THRESHOLD:
                self._abort_to_double_support()
            else:
                self._transition_to(StepPhase.LIFT)

    def _handle_lift(self, dt: float) -> None:
        self._compute_stance_ik()

        timer        = shared_state.step_phase_timer
        swing_force  = self._swing_foot_force()
        stance_force = self._stance_foot_force()
        swing_vel_z  = abs(float(self._swing_foot_velocity()[2]))

        # Advance to SWING only when the SWING foot is unloaded AND the STANCE
        # foot is actively bearing weight.  A PyBullet GJK glitch can zero both
        # forces simultaneously; without the stance guard the robot would enter
        # SWING with the stance leg also unloaded, causing force collapse.
        if (swing_force  < SWING_UNLOAD_THRESHOLD and
                swing_vel_z < SETTLE_VEL_THRESHOLD and
                stance_force >= STANCE_LOAD_THRESHOLD):
            self._snapshot_swing_foot_x()
            self._transition_to(StepPhase.SWING)
            return

        if timer >= LIFT_TIMEOUT:
            # Abort if: swing foot still loaded, OR stance foot not bearing weight.
            if (swing_force  > SWING_UNLOAD_THRESHOLD or
                    stance_force < STANCE_LOAD_THRESHOLD):
                self._abort_to_double_support()
            else:
                self._snapshot_swing_foot_x()
                self._transition_to(StepPhase.SWING)

    def _handle_swing(self, dt: float) -> None:
        self._compute_stance_ik()

        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # Compute foot target — CP may update mid-swing
        x_target = _compute_x_target(
            _clamped_cp_x(),
            decel=(shared_state.mission_state == MissionState.DECEL),
        )
        shared_state.swing_foot_target = (x_target, 0.0, 0.0)
        if side == "left":
            shared_state.left_foot_target  = (x_target, 0.0, 0.0)
        else:
            shared_state.right_foot_target = (x_target, 0.0, 0.0)

        # Advance phi
        phi = shared_state.swing_phase + dt / SWING_DURATION
        shared_state.swing_phase = phi

        # Swing arc IK
        self._compute_swing_ik(side, hip_x, hip_z, phi, x_target)

        # Exit: phi reaches PLACE_ENTRY_PHI
        if phi >= PLACE_ENTRY_PHI:
            self._transition_to(StepPhase.PLACE)
            return

        # Timeout: force advance to PLACE, log error code
        if shared_state.step_phase_timer >= SWING_DURATION * SWING_TIMEOUT_FACTOR:
            shared_state.add_error_code(ERR_PHASE_TIMEOUT)
            self._transition_to(StepPhase.PLACE)

    def _handle_place(self, dt: float) -> None:
        self._compute_stance_ik()

        side    = shared_state.active_swing_side
        hip_key = _LEFT_HIP_LINK if side == "left" else _RIGHT_HIP_LINK
        hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
        hip_x   = float(hip_pos[0])
        hip_z   = float(hip_pos[2])

        # phi continues to 1.0 then clamps
        phi = min(shared_state.swing_phase + dt / SWING_DURATION, 1.0)
        shared_state.swing_phase = phi

        x_target = _compute_x_target(
            _clamped_cp_x(),
            decel=(shared_state.mission_state == MissionState.DECEL),
        )
        self._compute_swing_ik(side, hip_x, hip_z, phi, x_target)

        # Exit: contact confirmed + velocity settled
        swing_contact = self._swing_foot_contact()
        swing_vel_z   = abs(float(self._swing_foot_velocity()[2]))

        if (swing_contact == ContactState.CONTACT_CONFIRMED and
                swing_vel_z < SETTLE_VEL_THRESHOLD):
            self._complete_step()
            return

        # Timeout: route by force
        if shared_state.step_phase_timer >= PLACE_TIMEOUT:
            if self._swing_foot_force() > SWING_UNLOAD_THRESHOLD:
                # Sensor lagged — contact happened; complete step
                self._complete_step()
            else:
                # Foot missed ground — unsafe; freeze
                shared_state.freeze_robot = True

    # ── Swing IK helper ───────────────────────────────────────────────────────

    def _compute_swing_ik(
        self, side: str, hip_x: float, hip_z: float,
        phi: float, x_target: float,
    ) -> None:
        """Compute swing leg IK from parabolic arc and write to shared_state."""
        phi_c   = min(phi, 1.0)
        x_swing = shared_state.swing_foot_x_stance + (x_target - shared_state.swing_foot_x_stance) * phi_c
        z_swing = _swing_z(phi_c)
        rel_z   = z_swing - hip_z
        if not (-1.0 < rel_z < 0.0):
            return   # invalid geometry; hold last angles
        foot_xyz_rel = (x_swing - hip_x, 0.0, rel_z)
        try:
            angles = kinematics.solve_ik(foot_xyz_rel, side)
        except ValueError:
            angles = (0.0, 0.0, 0.0)
        if side == "left":
            shared_state.ik_left_angles  = angles
        else:
            shared_state.ik_right_angles = angles

    # ── Step completion ───────────────────────────────────────────────────────

    def _complete_step(self) -> None:
        """Clean PLACE → DOUBLE_SUPPORT: flip sides, increment step_count, lock anchor."""
        old_swing   = shared_state.active_swing_side
        new_stance  = old_swing
        new_swing   = "right" if old_swing == "left" else "left"

        shared_state.active_swing_side = new_swing
        shared_state.stance_side       = new_stance
        shared_state.step_count       += 1
        shared_state.swing_phase       = 0.0

        # Lock the newly landed foot as the stance anchor for the next step
        foot_pos = (shared_state.left_foot_position  if new_stance == "left"
                    else shared_state.right_foot_position)
        shared_state.stance_foot_world_pos = foot_pos.copy()

        self._transition_to(StepPhase.DOUBLE_SUPPORT)

    # ── Convenience accessors for swing-leg sensors ───────────────────────────

    def _swing_foot_force(self) -> float:
        side = shared_state.active_swing_side
        return float(shared_state.left_foot_force  if side == "left"
                     else shared_state.right_foot_force)

    def _stance_foot_force(self) -> float:
        """Normal force on the STANCE (weight-bearing) foot, N."""
        side = shared_state.stance_side
        return float(shared_state.left_foot_force  if side == "left"
                     else shared_state.right_foot_force)

    def _swing_foot_velocity(self) -> np.ndarray:
        side = shared_state.active_swing_side
        return (shared_state.left_foot_velocity  if side == "left"
                else shared_state.right_foot_velocity)

    def _swing_foot_contact(self) -> ContactState:
        side = shared_state.active_swing_side
        return (shared_state.left_foot_contact_state  if side == "left"
                else shared_state.right_foot_contact_state)

    def _snapshot_swing_foot_x(self) -> None:
        """Snapshot swing foot x at COM_SHIFT exit.  Written exactly once per step."""
        side = shared_state.active_swing_side
        foot_pos = (shared_state.left_foot_position  if side == "left"
                    else shared_state.right_foot_position)
        shared_state.swing_foot_x_stance = float(foot_pos[0])


_gait_planner = GaitPlannerController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_gait_planner() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _gait_planner.update()
