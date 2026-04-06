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

import os
import sys
import time
import threading
import numpy as np
import pybullet as p
import pybullet_data
from typing import Optional, Tuple, Dict

from shared_state import (
    shared_state,
    Siclo1State,
    TelemetryRingBuffer,
    SystemStatus,
    ContactState,
    URDF_JOINT_NAMES,
    URDF_JOINT_LIMITS,
    DEFAULT_LINK_DATA,
    ERR_TIMING_VIOLATION,
    ERR_MID_CYCLE_OVERRUN,
)
import sim.interface
import perception
import stability
import recovery
import active_balance
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


# ============================================================================
# DETERMINISTIC TIMING CONTROLLER
# ============================================================================

class HeartbeatController:
    """Enforces strict 100 Hz with hybrid spin-wait.
    Uses scalar accumulators — zero dynamic allocation."""

    __slots__ = (
        'target_dt', 'cycle_start', 'last_cycle_end',
        '_cycle_times', '_violations_count', '_cycle_count',
        '_max_compute', '_min_compute', '_sum_compute', '_sum_sq_compute',
    )

    def __init__(self, target_dt: float = TARGET_DT):
        self.target_dt = target_dt
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
        threshold = 5.0  # Fz noise threshold
        
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
                    if force > threshold:
                        ss.left_contact_ticks += 1
                        # Flat Foot: X-spread of contact points > 1cm (more realistic for Siclo1)
                        pts_x = [c[5][0] for c in contacts]
                        ss.left_foot_flat = (max(pts_x) - min(pts_x)) > 0.01 if len(pts_x) > 1 else False
                        # Store all contact point positions
                        ss.left_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.left_contact_ticks = 0
                        ss.left_foot_flat = False
                        ss.left_contact_points = []
                else:
                    ss.right_foot_position = pos
                    ss.right_foot_velocity = vel
                    ss.right_foot_force = force
                    if force > threshold:
                        ss.right_contact_ticks += 1
                        pts_x = [c[5][0] for c in contacts]
                        # Reduced threshold to 1cm spread
                        ss.right_foot_flat = (max(pts_x) - min(pts_x)) > 0.01 if len(pts_x) > 1 else False
                        # Store all contact point positions
                        ss.right_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.right_contact_ticks = 0
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
        torques = getattr(self.shared_state, 'target_torques', {})

        for jname, raw_torque in torques.items():
            jid = self.joint_ids.get(jname)
            if jid is None:
                continue
            clipped = _clip_effort(jname, raw_torque)
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

    def __init__(self, use_gui: bool = False, viz_decimation: int = 10):
        if viz_decimation < 1:
            raise ValueError(f"viz_decimation must be >= 1, got {viz_decimation}")
        self.use_gui = use_gui
        self.viz_decimation: int = viz_decimation  # cycles between GUI renders
        self._visualizer = None                    # set after warmup if GUI mode

        # 1. Physics client — ALWAYS headless
        self.physics_client = p.connect(p.DIRECT)

        # 2. Optional GUI viewer
        self.gui_client: Optional[int] = None
        if use_gui:
            try:
                self.gui_client = p.connect(p.GUI)
                time.sleep(2.0)  # X-server buffer — wait for window to appear (WSL)
            except Exception:
                self.gui_client = None

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

        self.pybullet.load_robot(urdf_path=urdf_file)

        # Hip-pitch child link names (child of Left_Hip_Forwards / Right_Hip_Fowards).
        # Used by DebugVisualizer to read world-space hip positions.
        self._left_hip_link  = "Left_Upper_Leg_1"   # URDF-verified 2026-04-05
        self._right_hip_link = "Right_Upper_Leg_1"  # URDF-verified 2026-04-05

        # 7. If GUI, mirror the scene
        if self.gui_client is not None:
            try:
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0,
                                           physicsClientId=self.gui_client)  # Hide sidebars
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                                           physicsClientId=self.gui_client)  # Batch renders
                p.setGravity(0, 0, -9.81, physicsClientId=self.gui_client)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                p.loadURDF("plane.urdf", physicsClientId=self.gui_client)
                p.setAdditionalSearchPath(os.path.dirname(urdf_file))
                self._gui_robot_id: int = p.loadURDF(
                    urdf_file,
                    basePosition=[0.0, 0.0, URDF_SPAWN_Z],
                    physicsClientId=self.gui_client,
                    flags=p.URDF_USE_INERTIA_FROM_FILE,
                )
            except Exception:
                pass

        # 8. Pre-cache mass totals (avoid per-cycle reparse)
        self._total_link_mass = sum(
            d['mass'] for d in DEFAULT_LINK_DATA.values()
        )

        # 9. Reset
        self.shared_state.reset()

        # 10. Telemetry consumer thread
        self._telemetry_thread = TelemetryThread(self.shared_state)
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
        warmup_cycles = 5 if self.use_gui else 50
        self._warmup(warmup_cycles)
        self._telemetry_thread.log(f"  Warmup cycles: {warmup_cycles}")

        # Debug visualiser — GUI mode only
        if self.gui_client is not None:
            from viz.debug_markers import DebugVisualizer
            self._visualizer = DebugVisualizer(self.gui_client)

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
            active_balance.update_active_balance()
            recovery.update_recovery()
            self.pybullet.apply_control()
            sim.interface.step_simulation(self.physics_client)

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

        # 2. Sensors
        self.pybullet.read_sensors()

        if shared_state.freeze_robot:
            return False

        # 3. Link positions → avoids FK fallback
        self.pybullet.update_link_positions()

        # 4. Perception
        perception.update_perception()

        # 5. Stability
        stability.update_stability(dt=TARGET_DT)

        # ── TIMING GUARD (mid-cycle) ────────────────────────────────────
        elapsed = time.perf_counter() - self.heartbeat.cycle_start
        if elapsed > OVERRUN_LIMIT:
            shared_state.add_error_code(ERR_MID_CYCLE_OVERRUN)

        # 6. Active balance
        active_balance.update_active_balance()

        # 7. Emergency gate
        if shared_state.emergency_stop_triggered:
            return False

        # 8. Recovery
        recovery.update_recovery()

        # 9. Control
        self.pybullet.apply_control()

        # 10. Physics step — DIRECT only (no render stall)
        sim.interface.step_simulation(self.physics_client)
        
        # 11. Advance counters
        shared_state.cycle_count += 1
        shared_state.sim_time   += TARGET_DT

        # 13. End cycle (deterministic wait)
        violation, comp_time = self.heartbeat.end_cycle()

        # 14. Write telemetry (zero-alloc: reuse scratch row)
        row = shared_state._telem_row
        row[0] = shared_state.sim_time
        row[1] = shared_state.cycle_count
        row[2] = 0  # error code (set below if violation)
        row[3] = shared_state.com_position[0]
        row[4] = shared_state.com_position[1]
        row[5] = shared_state.com_position[2]
        row[6] = shared_state.left_foot_contact_state.value
        row[7] = shared_state.right_foot_contact_state.value
        row[8] = shared_state.stability_status.value
        row[9] = shared_state.left_foot_force
        row[10] = shared_state.right_foot_force
        row[11] = shared_state.stability_margin
        row[12] = comp_time * 1e6  # microseconds
        if violation:
            row[2] = ERR_TIMING_VIOLATION
        shared_state.telemetry.write(row)

        # 15. Optional GUI sync (decimated — every viz_decimation cycles)
        if (self.gui_client is not None and
                shared_state.cycle_count % self.viz_decimation == 0):
            self._sync_gui()

        return True

    # ------------------------------------------------------------------ #
    def _sync_gui(self) -> None:
        """Mirror joint states to the GUI client at decimated rate."""
        try:
            rid_phys = self.pybullet.robot_id
            pc_phys  = self.physics_client
            pc_gui   = self.gui_client

            # Mirror base pose
            pos, orn = p.getBasePositionAndOrientation(
                rid_phys, physicsClientId=pc_phys)
            p.resetBasePositionAndOrientation(
                self._gui_robot_id, pos, orn, physicsClientId=pc_gui)

            # Mirror joint positions
            for jname, jid in self.pybullet._joint_list:
                js = p.getJointState(rid_phys, jid, physicsClientId=pc_phys)
                p.resetJointState(self._gui_robot_id, jid, js[0], js[1],
                                  physicsClientId=pc_gui)
            # Update debug visualisation (annulus arcs + hip→foot vectors)
            if self._visualizer is not None:
                lp = self.shared_state.link_positions
                left_hip  = tuple(lp.get(self._left_hip_link,  [0.0, 0.0, 0.0]))
                right_hip = tuple(lp.get(self._right_hip_link, [0.0, 0.0, 0.0]))
                self._visualizer.update(self.shared_state, left_hip, right_hip)
        except Exception:
            pass  # GUI sync is non-critical

    # ------------------------------------------------------------------ #
    def run(self, max_cycles: int = 1000, print_interval: float = 1.0):
        print(f"\n{'='*70}")
        print(f"STARTING SIMULATION \u2014 {max_cycles} cycles @ {TARGET_FREQ} Hz  (OPTIMISED)")
        print(f"{'='*70}\n")

        for i in range(max_cycles):
            success = self.step()
            if not success:
                self._telemetry_thread.log(
                    f"[STOPPED] t={shared_state.sim_time:.2f}s")
                break

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
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.stop()
            self._telemetry_thread.join(timeout=2.0)
        try:
            p.disconnect(physicsClientId=self.physics_client)
        except Exception:
            pass
        if self.gui_client is not None:
            try:
                p.disconnect(physicsClientId=self.gui_client)
            except Exception:
                pass
        self._telemetry_thread.flush_to_stdout()


