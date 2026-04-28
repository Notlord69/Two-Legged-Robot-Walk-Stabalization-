"""
================================================================================
PROJECT SICLO1 — OPTIMISED HEARTBEAT CONTROLLER  (TEMP_HeartBeat.py)
================================================================================

NON-DESTRUCTIVE rewrite of HeartBeat.py targeting strict 100 Hz (≤10 ms).

Changes from original HeartBeat.py
───────────────────────────────────
1. Physics runs on p.DIRECT (headless) — no GUI render stall.
   Optional p.GUI viewer connected as a second p.connect(p.GUI) client at decimated rate.
2. Deterministic hybrid spin-wait replaces time.sleep().
3. URDF link positions populated from PyBullet getLinkState() every cycle
   so stability.py avoids the expensive forward_kinematics_2d() fallback.
4. All hot-loop print() replaced with buffered deque logging.
5. 10 ms timing guard: logs overrun and increments violations.
6. Direct os.path.join() for URDF path — no glob.glob().
7. URDF mass cache pre-computed at init (already in shared_state
   DEFAULT_LINK_DATA — no per-cycle reparse).

Author : Siclo1 Project Team
Date   : March 2026
================================================================================
"""

import gc
import os
import sys
import time
import threading
import math
from dataclasses import dataclass
from sim.viz_bridge import VizBridge
from typing import Optional, Tuple, Dict, Any

import numpy as np
import pybullet as p
import pybullet_data

from shared_state import (
    shared_state,
    Siclo1State,
    TelemetryRingBuffer,
    SystemStatus,
    ContactState,
    MissionState,
    StepPhase,
    URDF_JOINT_NAMES,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
    ERR_TIMING_VIOLATION,
    ERR_MID_CYCLE_OVERRUN,
    STAGE_NAMES,
)
import sim.interface
import perception
import stability
import recovery
import balance_controller
import grf
import gait_planner
import mission
from mission import MissionController
from telemetry import TelemetryThread


# ============================================================================
# CONSTANTS
# ============================================================================

URDF_SPAWN_Z: float = 0.8806          # URDF-aligned spawn height
TARGET_FREQ:   float = 100.0          # Hz
TARGET_DT:     float = 1.0 / TARGET_FREQ   # 0.01 s
OVERRUN_LIMIT: float = TARGET_DT      # 10 ms hard ceiling

# Maximum log buffer size (printed at end of run, not during)
LOG_BUFFER_SIZE: int = 2000

# WBC joint-space PD gains — converts IK angle targets to additive torques.
# These are tuning parameters, not URDF-derived.
WBC_KP: float = 100.0   # N·m/rad, joint position proportional gain (halved to avoid saturation)
WBC_KD: float = 28.0    # N·m·s/rad, joint velocity derivative gain (ζ ≈ 0.9 near-critical damping)

# WBC joint mapping: (IK angle index, URDF joint name, sign)
# sign: +1 for right (axis +X), -1 for left (axis -X)
# Tuple order: (hip_roll, hip_pitch, knee, ankle) = indices 0, 1, 2, 3
_WBC_LEFT_JOINTS = [
    (0, 'Left_Hip_Inwards',  -1.0),   # hip_roll   (axis = -Y, adduction)
    (1, 'Left_Hip_Forwards', -1.0),   # hip_pitch  (axis = -X)
    (2, 'Left_Knee',         -1.0),   # knee       (axis = -X)
    (3, 'Left_Ankle',        -1.0),   # ankle      (axis ≈ -Z, neutral = 0)
]
_WBC_RIGHT_JOINTS = [
    (0, 'Right_Hip_Inwards', -1.0),   # hip_roll   (axis = -Y, adduction)
    (1, 'Right_Hip_Fowards', +1.0),   # hip_pitch  (axis = +X)
    (2, 'Right_Knee',        +1.0),   # knee       (axis = +X)
    (3, 'Right_Ankle',       +1.0),   # ankle      (axis ≈ -Z, neutral = 0)
]


# ============================================================================
# DETERMINISTIC TIMING CONTROLLER
# ============================================================================

class HeartbeatController:
    """Enforces strict 100 Hz with hybrid spin-wait.
    Uses scalar accumulators — zero dynamic allocation."""

    __slots__ = (
        'target_dt', 'strict', 'cycle_start', 'last_cycle_end',
        '_cycle_times', '_violations_count', '_cycle_count',
        '_max_compute', '_min_compute', '_sum_compute', '_sum_sq_compute',
    )

    def __init__(self, target_dt: float = TARGET_DT, strict: bool = True):
        self.target_dt = target_dt
        self.strict    = strict   # False → count internally but skip shared_state writes
        self.cycle_start: float = 0.0
        self.last_cycle_end: float = 0.0
        self._cycle_count: int = 0
        self._violations_count: int = 0
        # Running statistics — no arrays needed
        self._max_compute: float = 0.0
        self._min_compute: float = float('inf')
        self._sum_compute: float = 0.0
        self._sum_sq_compute: float = 0.0
        # Keep a single last_dt for diagnostics (no list)
        self._cycle_times: float = 0.0  # last measured dt

    # ------------------------------------------------------------------ #
    def start_cycle(self) -> float:
        now = time.perf_counter()
        self.cycle_start = now
        if self._cycle_count > 0:
            actual_dt = now - self.last_cycle_end
            self._cycle_times = actual_dt
            shared_state.last_dt = actual_dt
        return now

    # ------------------------------------------------------------------ #
    def end_cycle(self) -> Tuple[bool, float]:
        """
        Deterministic wait.
        1. If >2 ms remain, sleep the bulk (saves CPU).
        2. Spin-wait the final ≤1 ms for µs-level precision.
        Returns (violation, computation_time).
        """
        now = time.perf_counter()
        compute_time = now - self.cycle_start
        violation = compute_time > self.target_dt

        # Running stats — zero allocation
        if compute_time > self._max_compute:
            self._max_compute = compute_time
        if compute_time < self._min_compute:
            self._min_compute = compute_time
        self._sum_compute += compute_time
        self._sum_sq_compute += compute_time * compute_time

        if violation:
            self._violations_count += 1
            if self.strict:
                shared_state.increment_timing_violations()
                shared_state.add_error_code(ERR_TIMING_VIOLATION)
        else:
            # Hybrid sleep + spin-wait
            remaining = self.target_dt - compute_time
            if remaining > 0.002:
                time.sleep(remaining - 0.001)
            # Spin the last fraction
            deadline = self.cycle_start + self.target_dt
            while time.perf_counter() < deadline:
                pass

        self._cycle_count += 1
        self.last_cycle_end = time.perf_counter()
        return violation, compute_time

    # ------------------------------------------------------------------ #
    def get_statistics(self) -> dict:
        n = self._cycle_count
        if n == 0:
            return {}
        mean = self._sum_compute / n
        variance = (self._sum_sq_compute / n) - (mean * mean)
        std = variance ** 0.5 if variance > 0 else 0.0
        return {
            'mean_dt':        mean,
            'std_dt':         std,
            'min_dt':         self._min_compute,
            'max_dt':         self._max_compute,
            'jitter_ms':      std * 1000,
            'violations':     self._violations_count,
            'violation_rate': self._violations_count / max(1, n),
        }


# Foot-flat gate — single contact point pitch tolerance
FLAT_PITCH_THRESHOLD: float = math.radians(7.0)

# Contact force threshold for tick accumulation (matches read_sensors)
_CONTACT_FORCE_THRESHOLD: float = 5.0  # N, Fz below this is treated as no-contact


def _update_contact_ticks(current: int, force: float, threshold: float) -> int:
    """Decay-based contact tick counter.

    force > threshold  →  current + 1       (accumulate contact evidence)
    force ≤ threshold  →  max(0, current − 1) (decay: one bad frame costs one tick)

    A single PyBullet GJK glitch decrements by 1 rather than wiping the
    accumulator.  The foot needs 3 consecutive zero-force frames to fully
    lose contact confidence, matching the 3-tick confirmation gate in
    perception.FootContactFSM.
    """
    if force > threshold:
        return current + 1
    return max(0, current - 1)
# rad — foot-flat gate for single-contact confirmation.
# At q=0 the URDF foot sole is parallel to the floor (zero plantar-flexion offset).
# 7° accepts post-bounce settling (0–6°); rejects tiptoe (>20°) and
# transient heel-strike (8–15°, never accumulates 3 ticks anyway).


def _compute_foot_flat(pts_x: list, foot_pitch: float) -> bool:
    """Pure function: is the foot confirmed flat from contact geometry and pitch?

    pts_x       -- list of contact point X coordinates (world frame, metres).
    foot_pitch  -- foot link pitch in radians (rotation about world Y, from getLinkState).

    Multi-point path: requires spread > 1 cm (original behaviour).
    Single-point path: pitch fallback for the PyBullet degenerate case where a
        stable rectangular box on a flat plane returns only one contact point.
    """
    if len(pts_x) > 1:
        return (max(pts_x) - min(pts_x)) > 0.01
    if len(pts_x) == 1:
        return abs(foot_pitch) < FLAT_PITCH_THRESHOLD
    return False


# ============================================================================
# URDF SAFETY CLIPPER  (unchanged logic, inlined for speed)
# ============================================================================

def _clip_effort(joint_name: str, value: float) -> float:
    lim = URDF_JOINT_LIMITS.get(joint_name)
    if lim is None:
        return value
    e = lim['effort']
    if value > e:
        return e
    if value < -e:
        return -e
    return value


def _clip_position(joint_name: str, value: float) -> float:
    lim = URDF_JOINT_LIMITS.get(joint_name)
    if lim is None:
        return value
    lo, hi = lim['lower'], lim['upper']
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _saturate_hip_pitch(wbc_tau: float, grf_tau: float,
                        emergency_tau: float, effort_limit: float) -> float:
    """Scale WBC down if total exceeds limit. Preserve safety + feedforward."""
    total = wbc_tau + grf_tau + emergency_tau
    if abs(total) <= effort_limit:
        return total
    protected = grf_tau + emergency_tau
    remaining = effort_limit * np.sign(total) - protected
    if abs(wbc_tau) > 1e-6:
        scale = min(1.0, abs(remaining / wbc_tau))
        return protected + wbc_tau * scale
    return float(np.clip(protected, -effort_limit, effort_limit))


# ============================================================================
# PYBULLET INTERFACE  — OPTIMISED, URDF-AWARE
# ============================================================================

class PyBulletInterface:
    """
    Headless physics interface.
    Sensor reads and control writes; no rendering.
    """

    def __init__(self, physics_client: int, state: Siclo1State):
        self.pc = physics_client
        self.shared_state = state
        self.robot_id: Optional[int] = None

        self.joint_ids: Dict[str, int] = {}
        self.link_index_by_name: Dict[str, int] = {}  # for getLinkState cache

        self.left_foot_link_id:  Optional[int] = None
        self.right_foot_link_id: Optional[int] = None

        # Pre-extracted joint-id list for fast iteration
        self._joint_list: list = []  # [(name, pybullet_id), ...]

        p.setGravity(0, 0, -9.81, physicsClientId=self.pc)

    # ------------------------------------------------------------------ #
    # ROBOT LOADING
    # ------------------------------------------------------------------ #

    def load_robot(self, urdf_path: str) -> int:
        p.setAdditionalSearchPath(os.path.dirname(urdf_path))
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0.0, 0.0, URDF_SPAWN_Z],
            physicsClientId=self.pc,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )
        self._build_joint_map()
        return self.robot_id

    def _build_joint_map(self) -> None:
        n = p.getNumJoints(self.robot_id, physicsClientId=self.pc)
        for i in range(n):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.pc)
            jname = info[1].decode('utf-8')
            lname = info[12].decode('utf-8')

            self.joint_ids[jname] = i
            self.link_index_by_name[lname] = i

            if lname == 'Left_Foot_1':
                self.left_foot_link_id = i
            if lname == 'Right_Foot_1':
                self.right_foot_link_id = i

        # Freeze iteration order
        self._joint_list = list(self.joint_ids.items())

        # Joint map info is logged via TelemetryThread after init

    # ------------------------------------------------------------------ #
    # SENSOR READING  — fast path
    # ------------------------------------------------------------------ #

    def read_sensors(self) -> None:
        if self.robot_id is None:
            return

        rid = self.robot_id
        pc  = self.pc
        ss  = self.shared_state

        # Base pose + velocity (2 C calls)
        pos, orn = p.getBasePositionAndOrientation(rid, physicsClientId=pc)
        vel, ang = p.getBaseVelocity(rid, physicsClientId=pc)

        ss.base_position         = np.array(pos)
        ss.base_orientation      = np.array(orn)
        ss.base_velocity         = np.array(vel)
        ss.base_angular_velocity = np.array(ang)

        # Joint states — iterate pre-frozen list
        jp = ss.joint_positions
        jv = ss.joint_velocities
        jt = ss.joint_torques
        for jname, jid in self._joint_list:
            js = p.getJointState(rid, jid, physicsClientId=pc)
            jp[jname] = js[0]
            jv[jname] = js[1]
            jt[jname] = js[3]

        # Foot contact forces & Robust Validation (3-tick gate + flat-foot)
        # Decay-based tick counter: a single zero-force frame decrements by 1
        # (not a hard reset), preventing PyBullet GJK glitches from collapsing
        # the support polygon.  See _update_contact_ticks for the exact rule.
        threshold = _CONTACT_FORCE_THRESHOLD  # N

        for foot, link_id in [('left', self.left_foot_link_id), ('right', self.right_foot_link_id)]:
            if link_id is not None:
                contacts = p.getContactPoints(bodyA=rid, linkIndexA=link_id, physicsClientId=pc)
                force = sum(c[9] for c in contacts) if contacts else 0.0

                # Sync foot position and velocity from link state
                link_state = p.getLinkState(rid, link_id, computeLinkVelocity=1, physicsClientId=pc)
                pos = np.array(link_state[0])
                vel = np.array(link_state[6])

                if foot == 'left':
                    ss.left_foot_position = pos
                    ss.left_foot_velocity = vel
                    ss.left_foot_force = force
                    # Pitch is kinematic — computed unconditionally, not gated on force
                    qx, qy, qz, qw = link_state[1]
                    foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
                    ss.left_foot_pitch = foot_pitch
                    ss.left_contact_ticks = _update_contact_ticks(
                        ss.left_contact_ticks, force, threshold)
                    if force > threshold:
                        pts_x = [c[5][0] for c in contacts]
                        ss.left_foot_flat = _compute_foot_flat(pts_x, foot_pitch)
                        ss.left_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.left_foot_flat = False
                        ss.left_contact_points = []
                else:
                    ss.right_foot_position = pos
                    ss.right_foot_velocity = vel
                    ss.right_foot_force = force
                    # Pitch is kinematic — computed unconditionally, not gated on force
                    qx, qy, qz, qw = link_state[1]
                    foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
                    ss.right_foot_pitch = foot_pitch
                    ss.right_contact_ticks = _update_contact_ticks(
                        ss.right_contact_ticks, force, threshold)
                    if force > threshold:
                        pts_x = [c[5][0] for c in contacts]
                        ss.right_foot_flat = _compute_foot_flat(pts_x, foot_pitch)
                        ss.right_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.right_foot_flat = False
                        ss.right_contact_points = []

    # ------------------------------------------------------------------ #
    # POPULATE LINK POSITIONS (avoids FK fallback in stability.py)
    # ------------------------------------------------------------------ #

    def update_link_positions(self) -> None:
        """
        Fill shared_state.link_positions from PyBullet getLinkState.
        This prevents stability.py from falling back to the expensive
        forward_kinematics_2d() path.
        """
        if self.robot_id is None:
            return

        rid = self.robot_id
        pc  = self.pc
        lp  = self.shared_state.link_positions

        for lname, link_idx in self.link_index_by_name.items():
            state = p.getLinkState(rid, link_idx, physicsClientId=pc)
            lp[lname] = np.array(state[0])  # worldLinkFramePosition

    # ------------------------------------------------------------------ #
    # CONTROL OUTPUT  —  inlined clipper (no np.clip overhead)
    # ------------------------------------------------------------------ #

    def apply_control(self) -> None:
        if self.robot_id is None:
            return

        rid = self.robot_id
        pc  = self.pc
        torques  = getattr(self.shared_state, 'target_torques', {})
        grf_corr = getattr(self.shared_state, 'grf_torque_correction', {})

        # Hip roll joints use POSITION_CONTROL to override PyBullet's default motor.
        # Target angles from balance_controller via shared_state.
        hip_roll_joints = {
            'Left_Hip_Inwards':  self.shared_state.balance_hip_roll_left,
            'Right_Hip_Inwards': self.shared_state.balance_hip_roll_right,
        }
        for jname, target_pos in hip_roll_joints.items():
            jid = self.joint_ids.get(jname)
            if jid is None:
                continue
            clamped_pos = _clip_position(jname, target_pos)
            p.setJointMotorControl2(
                rid, jid,
                controlMode=p.POSITION_CONTROL,
                targetPosition=clamped_pos,
                force=100.0,  # max motor force (URDF effort limit)
                positionGain=1.0,  # P gain, dimensionless (stiff)
                velocityGain=0.5,  # D gain, dimensionless (damped)
                physicsClientId=pc,
            )

        emergency = getattr(self.shared_state, 'emergency_sagittal_torque', {})

        for jname, raw_torque in torques.items():
            jid = self.joint_ids.get(jname)
            if jid is None:
                continue
            if jname in hip_roll_joints:
                continue

            grf_val = grf_corr.get(jname, 0.0)
            emerg_val = emergency.get(jname, 0.0)

            # Saturation-aware merge for hip pitch joints
            if jname in ('Left_Hip_Forwards', 'Right_Hip_Fowards') and abs(emerg_val) > 1e-6:
                lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
                merged = _saturate_hip_pitch(raw_torque, grf_val, emerg_val, lim)
            else:
                merged = raw_torque + grf_val + emerg_val

            if not math.isfinite(merged):
                print(f"[SANITY] NaN/Inf torque on {jname}: raw={raw_torque} "
                      f"grf={grf_val} emerg={emerg_val} — zeroed to prevent crash")
                merged = 0.0
            clipped = _clip_effort(jname, merged)
            p.setJointMotorControl2(
                rid, jid,
                controlMode=p.TORQUE_CONTROL,
                force=clipped,
                physicsClientId=pc,
            )


# ============================================================================
# MAIN CONTROLLER — OPTIMISED
# ============================================================================

class Siclo1Controller:
    """100 Hz headless controller with optional decimated GUI."""

    def __init__(self, use_gui: bool = False, viz_decimation: int = 10,
                 walk_distance: float = None, argv_command: str = "",
                 quiet: bool = True):
        if viz_decimation < 1:
            raise ValueError(f"viz_decimation must be >= 1, got {viz_decimation}")
        self.use_gui = use_gui
        self.viz_decimation: int = viz_decimation  # cycles between GUI renders
        self._visualizer = None                    # set after warmup if GUI mode
        self._walk_active: bool = (walk_distance is not None)
        self._quiet: bool = quiet  # suppresses all terminal telemetry output

        # 1. Physics client — ALWAYS headless
        self.physics_client = p.connect(p.DIRECT)

        # 2. GUI subprocess placeholder (VizBridge started after warmup)
        self.gui_client: Optional[int] = None  # kept for API compatibility; always None
        self._viz_bridge: Optional[VizBridge] = None

        # 3. Heartbeat
        self.heartbeat = HeartbeatController(target_dt=TARGET_DT)

        # 4. World setup
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=self.physics_client)
        p.setTimeStep(TARGET_DT, physicsClientId=self.physics_client)
        p.loadURDF("plane.urdf", physicsClientId=self.physics_client)

        # 5. Shared state + interface
        self.shared_state = shared_state
        self.pybullet = PyBulletInterface(self.physics_client, self.shared_state)

        # 6. URDF — direct path, no glob
        current_folder = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(current_folder, "Siclo1.urdf")
        if not os.path.isfile(urdf_file):
            print(f"[CRITICAL] Siclo1.urdf not found in {current_folder}")
            sys.exit(1)
        print(f"[Siclo1] URDF path: {os.path.abspath(urdf_file)}")

        self.pybullet.load_robot(urdf_path=urdf_file)

        # Spawn guard: refuse to run a ghost simulation
        _body_count = p.getNumBodies(physicsClientId=self.physics_client)
        if _body_count == 0 or self.pybullet.robot_id is None:
            raise RuntimeError(
                f"[Siclo1] URDF spawn failed — body_count={_body_count}, "
                f"robot_id={self.pybullet.robot_id}. Cannot run ghost simulation."
            )

        # Hip-pitch child link names (child of Left_Hip_Forwards / Right_Hip_Fowards).
        # Used by DebugVisualizer to read world-space hip positions.
        self._left_hip_link  = "Left_Upper_Leg_1"   # URDF-verified 2026-04-05
        self._right_hip_link = "Right_Upper_Leg_1"  # URDF-verified 2026-04-05

        # 7. GUI robot loaded in subprocess — nothing to do in main process

        # 8. Pre-cache mass totals (avoid per-cycle reparse)
        self._total_link_mass = sum(
            d['mass'] for d in DEFAULT_LINK_DATA.values()
        )

        # 9. Reset
        self.shared_state.reset()

        # 10. Telemetry consumer thread
        self._telemetry_thread = TelemetryThread(
            self.shared_state,
            argv_command=argv_command,
            quiet=quiet,
        )
        self._telemetry_thread.start()

        self._telemetry_thread.log(f"[Siclo1] Initialised (OPTIMISED, URDF-synced)")
        self._telemetry_thread.log(f"  Target freq  : {TARGET_FREQ} Hz")
        self._telemetry_thread.log(f"  Target dt    : {TARGET_DT*1000:.2f} ms")
        self._telemetry_thread.log(f"  Spawn height : {URDF_SPAWN_Z} m")
        self._telemetry_thread.log(f"  Total mass   : {self._total_link_mass:.4f} kg")
        self._telemetry_thread.log(f"  Robot ID     : {self.pybullet.robot_id}")
        self._telemetry_thread.log(f"  Bodies       : {p.getNumBodies(physicsClientId=self.physics_client)}")
        self._telemetry_thread.log(f"  GUI viewer   : {'ON' if self.gui_client is not None else 'OFF'}")

        # Warmup: settle physics before real-time loop.
        # GUI mode gets fewer cycles (window already visible); Direct gets more.
        # Mission controller — manages gait state machine and ramp_gain.
        # Created before _warmup so _mission is available inside the warmup loop.
        self._mission = MissionController(walk_distance=walk_distance)

        warmup_cycles = 5 if self.use_gui else 50
        self._warmup(warmup_cycles)
        self._telemetry_thread.log(f"  Warmup cycles: {warmup_cycles}")

        # 11. VizBridge — subprocess GUI (separate OS process, zero GIL contention)
        if use_gui:
            self._viz_bridge = VizBridge(
                joint_names=list(URDF_JOINT_NAMES.values()),
                urdf_path=urdf_file,
                viz_fps=max(1, 100 // self.viz_decimation),
                spawn_z=URDF_SPAWN_Z,
            )
            self._viz_bridge.start()

        # 12. VideoRecorder — disabled temporarily
        VIDEO_LOGGING_ENABLED = False  # set True to re-enable MP4 recording
        self._recorder = None
        if VIDEO_LOGGING_ENABLED:
            from recorder import VideoRecorder
            self._recorder = VideoRecorder(
                physics_client=self.physics_client,
                session_path=self._telemetry_thread.session_path,
            )
            self._recorder.start()

    # ------------------------------------------------------------------ #
    def _wbc_step(self) -> None:
        """Joint-space PD controller: IK angles → additive torques in target_torques.

        Reads ik_left_angles and ik_right_angles (URDF-signed rad).
        Computes PD torque for each leg joint and adds to shared_state.target_torques.
        GRF corrections are NOT merged here — handled in apply_control().
        """
        shared_state.target_torques = {}
        torques = shared_state.target_torques

        jp = shared_state.joint_positions
        jv = shared_state.joint_velocities

        for idx, jname, _ in _WBC_LEFT_JOINTS:
            kp = WBC_KP
            kd = WBC_KD

            theta_target = shared_state.ik_left_angles[idx]
            if idx == 1:  # hip_pitch — add sagittal balance offset
                theta_target += shared_state.sagittal_pitch_offset
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            error = theta_target - theta_now
            tau = kp * error - kd * omega_now
            clipped_tau = _clip_effort(jname, tau)
            torques[jname] = clipped_tau
            # Tracking telemetry
            shared_state.wbc_tracking_error[jname] = error
            lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
            shared_state.wbc_torque_saturated[jname] = (abs(tau) >= lim - 0.1)

        for idx, jname, _ in _WBC_RIGHT_JOINTS:
            kp = WBC_KP
            kd = WBC_KD

            theta_target = shared_state.ik_right_angles[idx]
            if idx == 1:  # hip_pitch — add sagittal balance offset
                theta_target += shared_state.sagittal_pitch_offset
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            error = theta_target - theta_now
            tau = kp * error - kd * omega_now
            clipped_tau = _clip_effort(jname, tau)
            torques[jname] = clipped_tau
            # Tracking telemetry
            shared_state.wbc_tracking_error[jname] = error
            lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
            shared_state.wbc_torque_saturated[jname] = (abs(tau) >= lim - 0.1)

        # Periodic WBC telemetry logging (every 50 cycles = 0.5s)
        if shared_state.cycle_count % 50 == 0:
            max_err = max(abs(e) for e in shared_state.wbc_tracking_error.values()) if shared_state.wbc_tracking_error else 0.0
            sat_count = sum(1 for v in shared_state.wbc_torque_saturated.values() if v)
            print(f"[WBC] cycle={shared_state.cycle_count} max_err={max_err:.3f}rad sat_count={sat_count}")

    # ------------------------------------------------------------------ #
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
            balance_controller.update_balance()
            grf.update_grf()
            gait_planner.update_gait_planner()
            self._mission.update()
            self._wbc_step()
            recovery.update_recovery()
            self.pybullet.apply_control()
            sim.interface.step_simulation(self.physics_client)

        # Reset gait planner and balance controller after warmup
        gait_planner.reset_gait_planner()
        balance_controller.reset_balance()

    # ------------------------------------------------------------------ #
    def step(self) -> bool:
        """
        One 100 Hz control cycle.

        Order:
          1. Heartbeat start
          2. Read sensors
          3. Populate link positions (prevents FK fallback)
          4. Perception
          5. Stability
          6. Active balance
          7. Emergency gate
          8. Recovery
          9. Apply control (torques clipped to URDF limits)
         10. stepSimulation (DIRECT only — no render block)
         11. Produce telemetry row (zero-alloc)
         12. Timing guard & end cycle
        """
        # 1.
        self.heartbeat.start_cycle()
        shared_state.timing_violation_this_cycle = False  # reset per-cycle overrun flag

        # 2. Sensors
        self.pybullet.read_sensors()
        shared_state._stage_times[0] = time.perf_counter() - self.heartbeat.cycle_start

        if shared_state.freeze_robot:
            return False

        # 3. Link positions → avoids FK fallback
        self.pybullet.update_link_positions()
        shared_state._stage_times[1] = time.perf_counter() - self.heartbeat.cycle_start

        # 4. Perception
        perception.update_perception()
        shared_state._stage_times[2] = time.perf_counter() - self.heartbeat.cycle_start

        # 5. Stability
        stability.update_stability(dt=TARGET_DT)
        shared_state._stage_times[3] = time.perf_counter() - self.heartbeat.cycle_start

        # ── TIMING GUARD (mid-cycle) ────────────────────────────────────
        elapsed = time.perf_counter() - self.heartbeat.cycle_start
        if elapsed > OVERRUN_LIMIT:
            shared_state.add_error_code(ERR_MID_CYCLE_OVERRUN)
            shared_state.timing_violation_this_cycle = True

        # 6. Balance controller (unified lateral + sagittal)
        balance_controller.update_balance()
        shared_state._stage_times[4] = time.perf_counter() - self.heartbeat.cycle_start

        # 7. GRF — virtual spring-damper torque corrections
        grf.update_grf()
        shared_state._stage_times[5] = time.perf_counter() - self.heartbeat.cycle_start

        # 8. Gait Planner — swing arc + IK angle targets
        gait_planner.update_gait_planner()
        shared_state._stage_times[6] = time.perf_counter() - self.heartbeat.cycle_start

        # 9. Mission — state machine, ramp_gain, step counting
        self._mission.update()
        shared_state._stage_times[7] = time.perf_counter() - self.heartbeat.cycle_start

        # 10. WBC — IK angles → additive joint PD torques
        self._wbc_step()
        shared_state._stage_times[8] = time.perf_counter() - self.heartbeat.cycle_start

        # 11. Emergency gate
        if shared_state.emergency_stop_triggered:
            return False

        # 12. Recovery
        recovery.update_recovery()
        shared_state._stage_times[9] = time.perf_counter() - self.heartbeat.cycle_start

        # 13. Control
        self.pybullet.apply_control()
        shared_state._stage_times[10] = time.perf_counter() - self.heartbeat.cycle_start

        # 14. Physics step — DIRECT only (no render stall)
        sim.interface.step_simulation(self.physics_client)
        shared_state._stage_times[11] = time.perf_counter() - self.heartbeat.cycle_start

        # 15. Advance counters
        shared_state.cycle_count += 1
        shared_state.sim_time   += TARGET_DT

        # 16. End cycle (deterministic wait)
        violation, comp_time = self.heartbeat.end_cycle()

        # Per-stage worst logging (text buffer only — CSV schema unchanged)
        _prev = 0.0
        _worst_ms = 0.0
        _worst_stage = ''
        for _i in range(12):
            _t = shared_state._stage_times[_i]
            _delta = (_t - _prev) * 1000.0
            if _delta > _worst_ms:
                _worst_ms = _delta
                _worst_stage = STAGE_NAMES[_i]
            _prev = _t
        if _worst_ms > 3.0:
            self._telemetry_thread.log(
                f"[STAGE] worst={_worst_stage}  {_worst_ms:.2f} ms"
            )

        # 17. Write telemetry (zero-alloc: reuse scratch row, 72 columns)
        row = shared_state._telem_row
        # Core timing & state (0-15)
        row[0] = shared_state.sim_time
        row[1] = shared_state.cycle_count
        row[2] = 0  # error code (set below if violation)
        row[3] = shared_state.com_position[0]
        row[4] = shared_state.com_position[1]
        row[5] = shared_state.com_position[2]
        row[6] = shared_state.com_velocity[0]
        row[7] = shared_state.com_velocity[1]
        row[8] = shared_state.com_velocity[2]
        row[9] = shared_state.left_foot_contact_state.value
        row[10] = shared_state.right_foot_contact_state.value
        row[11] = shared_state.stability_status.value
        row[12] = shared_state.left_foot_force
        row[13] = shared_state.right_foot_force
        row[14] = shared_state.stability_margin
        row[15] = comp_time * 1e6  # microseconds

        # Base angular velocity (16-18)
        row[16] = shared_state.base_angular_velocity[0]
        row[17] = shared_state.base_angular_velocity[1]
        row[18] = shared_state.base_angular_velocity[2]

        # Joint angles - actual (19-24)
        jp = shared_state.joint_positions
        row[19] = jp.get('Left_Hip_Forwards', 0.0)
        row[20] = jp.get('Left_Knee', 0.0)
        row[21] = jp.get('Left_Ankle', 0.0)
        row[22] = jp.get('Right_Hip_Fowards', 0.0)
        row[23] = jp.get('Right_Knee', 0.0)
        row[24] = jp.get('Right_Ankle', 0.0)

        # IK commanded angles (25-32) - (hip_roll, hip_pitch, knee, ankle) per side
        ik_l = shared_state.ik_left_angles
        ik_r = shared_state.ik_right_angles
        row[25] = ik_l[0] if len(ik_l) > 0 else 0.0  # L hip_roll
        row[26] = ik_l[1] if len(ik_l) > 1 else 0.0  # L hip_pitch
        row[27] = ik_l[2] if len(ik_l) > 2 else 0.0  # L knee
        row[28] = ik_l[3] if len(ik_l) > 3 else 0.0  # L ankle
        row[29] = ik_r[0] if len(ik_r) > 0 else 0.0  # R hip_roll
        row[30] = ik_r[1] if len(ik_r) > 1 else 0.0  # R hip_pitch
        row[31] = ik_r[2] if len(ik_r) > 2 else 0.0  # R knee
        row[32] = ik_r[3] if len(ik_r) > 3 else 0.0  # R ankle

        # WBC tracking error (33-38)
        te = shared_state.wbc_tracking_error
        row[33] = te.get('Left_Hip_Forwards', 0.0)
        row[34] = te.get('Left_Knee', 0.0)
        row[35] = te.get('Left_Ankle', 0.0)
        row[36] = te.get('Right_Hip_Fowards', 0.0)
        row[37] = te.get('Right_Knee', 0.0)
        row[38] = te.get('Right_Ankle', 0.0)

        # Torque saturation flags (39-44)
        ts = shared_state.wbc_torque_saturated
        row[39] = 1.0 if ts.get('Left_Hip_Forwards', False) else 0.0
        row[40] = 1.0 if ts.get('Left_Knee', False) else 0.0
        row[41] = 1.0 if ts.get('Left_Ankle', False) else 0.0
        row[42] = 1.0 if ts.get('Right_Hip_Fowards', False) else 0.0
        row[43] = 1.0 if ts.get('Right_Knee', False) else 0.0
        row[44] = 1.0 if ts.get('Right_Ankle', False) else 0.0

        # Applied torques (45-50)
        jt = shared_state.joint_torques
        row[45] = jt.get('Left_Hip_Forwards', 0.0)
        row[46] = jt.get('Left_Knee', 0.0)
        row[47] = jt.get('Left_Ankle', 0.0)
        row[48] = jt.get('Right_Hip_Fowards', 0.0)
        row[49] = jt.get('Right_Knee', 0.0)
        row[50] = jt.get('Right_Ankle', 0.0)

        # Gait state (51-55)
        row[51] = shared_state.step_phase.value
        row[52] = shared_state.swing_phase
        row[53] = 0.0 if shared_state.active_swing_side == 'left' else 1.0
        row[54] = shared_state.mission_state.value
        row[55] = shared_state.ramp_gain

        # Foot positions - actual (56-61)
        row[56] = shared_state.left_foot_position[0]
        row[57] = shared_state.left_foot_position[1]
        row[58] = shared_state.left_foot_position[2]
        row[59] = shared_state.right_foot_position[0]
        row[60] = shared_state.right_foot_position[1]
        row[61] = shared_state.right_foot_position[2]

        # Foot targets (62-67)
        lft = shared_state.left_foot_target
        rft = shared_state.right_foot_target
        row[62] = lft[0]
        row[63] = lft[1]
        row[64] = lft[2]
        row[65] = rft[0]
        row[66] = rft[1]
        row[67] = rft[2]

        # Capture point & slip (68-71)
        row[68] = shared_state.capture_point[0]
        row[69] = shared_state.capture_point[1]
        row[70] = 1.0 if shared_state.left_foot_slip_detected else 0.0
        row[71] = 1.0 if shared_state.right_foot_slip_detected else 0.0

        if violation:
            row[2] = ERR_TIMING_VIOLATION
        shared_state.telemetry.write(row)

        # 18. Optional GUI sync (decimated — every viz_decimation cycles)
        if (self._viz_bridge is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._viz_bridge.push_pose(
                shared_state.base_position,
                shared_state.base_orientation,
                shared_state.joint_positions,
            )

        return True

    # ------------------------------------------------------------------ #
    def run(self, max_cycles: int = 1000, print_interval: float = 1.0):
        print(f"\n{'='*70}")
        print(f"STARTING SIMULATION \u2014 {max_cycles} cycles @ {TARGET_FREQ} Hz  (OPTIMISED)")
        print(f"{'='*70}\n")

        gc.disable()  # prevent GC pauses inside 100 Hz hot loop
        try:
            for i in range(max_cycles):
                success = self.step()
                if not success:
                    self._telemetry_thread.log(
                        f"[STOPPED] t={shared_state.sim_time:.2f}s")
                    break
        finally:
            gc.enable()
            gc.collect()  # collect deferred garbage after loop

        if not self._quiet:
            self._print_final_summary()

    # ------------------------------------------------------------------ #
    def finalize_telemetry(self) -> None:
        """Stop telemetry thread and flush/close the CSV.

        Call BEFORE any hold loop and BEFORE shutdown().
        shutdown() checks is_alive() so calling both is safe.
        """
        self._telemetry_thread.stop()
        self._telemetry_thread.join(timeout=2.0)

    # ------------------------------------------------------------------ #
    def _print_final_summary(self) -> None:
        print(f"\n{'='*70}")
        print("SIMULATION COMPLETE \u2014 FINAL SUMMARY (OPTIMISED)")
        print(f"{'='*70}")

        ts = self._telemetry_thread.get_summary_stats()
        if ts:
            mean_ms   = ts['mean_dt'] * 1000
            jitter_ms = ts['jitter_ms']
            viol      = ts['violations']
            viol_pct  = ts['violation_rate'] * 100

            status = "\u2705 PASS" if mean_ms < 10.5 and jitter_ms < 1.0 else "\u274c FAIL"

            print(f"\n[TIMING]  {status}")
            print(f"  Mean dt   : {mean_ms:.3f} ms  (target: \u226410.00 ms)")
            print(f"  Jitter    : {jitter_ms:.3f} ms  (target: <1.00 ms)")
            print(f"  Min dt    : {ts['min_dt']*1000:.3f} ms")
            print(f"  Max dt    : {ts['max_dt']*1000:.3f} ms")
            print(f"  Violations: {viol} ({viol_pct:.1f}%)")

        rs = recovery.get_recovery_statistics()
        print(f"\n[RECOVERY]  total={rs['total_events']}  "
              f"abort={rs['abort_hold_count']}  "
              f"repos={rs['reposition_count']}  "
              f"emergency={rs['emergency_stop_count']}")

        print(f"\n[FINAL STATE]")
        print(f"  Cycles: {shared_state.cycle_count}  |  "
              f"Load: {shared_state.current_load_mass:.1f} kg  |  "
              f"Errors: {len(shared_state.error_messages)} "
              f"+ {shared_state._error_write_idx} coded")
        print(f"  COM: {np.round(shared_state.com_position, 3)}")

        # Flush telemetry thread log
        sep = '\u2500' * 70
        print(f"\n{sep}")
        print("TELEMETRY LOG:")
        print(sep)
        self._telemetry_thread.flush_to_stdout()

        print(f"\n{'='*70}\n")

    # ------------------------------------------------------------------ #
    def shutdown(self) -> None:
        # Step 1: stop VideoRecorder (disabled — self._recorder is None when off)
        if self._recorder is not None:
            video_path = self._recorder.stop()
            self._recorder.join(timeout=5.0)
            if self._recorder.is_alive():
                print("[WARN] VideoRecorder did not exit within 5 s — "
                      "walk.mp4 may be truncated")
            elif not os.path.isfile(video_path):
                print(f"[WARN] walk.mp4 expected at {video_path} but not found after flush")
            else:
                print(f"[INFO] walk.mp4 saved: {video_path} "
                      f"({self._recorder.frame_count} frames)")

        # Step 2: stop GUI subprocess.
        if self._viz_bridge is not None:
            self._viz_bridge.stop()

        # Step 3: stop telemetry thread.
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.stop()
            self._telemetry_thread.join(timeout=2.0)

        # Step 4: disconnect physics clients — safe now that threads are done.
        try:
            p.disconnect(physicsClientId=self.physics_client)
        except Exception:
            pass
        self._telemetry_thread.flush_to_stdout()


