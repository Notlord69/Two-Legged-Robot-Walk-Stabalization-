"""GUI subprocess entry point — never imported by the physics process.

Runs p.GUI in a completely separate OS process, reading pose data from a
shared_memory buffer written by VizBridge.push_pose() at 100 Hz.
The physics process's GIL is never shared; render stalls cannot cause violations.

Public API:
    main(...)          — entry point called by VizBridge.start()
    _init_pybullet(...)— init phase (separated for testability)
    _render_loop(...)  — infinite mirror loop (separated for testability)
"""
import os
import sys
import time
import numpy as np
from multiprocessing.shared_memory import SharedMemory

import pybullet as p
import pybullet_data


def _init_pybullet(shm_name: str, n_joints: int, joint_names: list,
                   urdf_path: str, viz_fps: int,
                   spawn_z: float) -> tuple:
    """Connect p.GUI, load world and robot, build joint map, signal ready.

    Returns: (client_id, robot_id, joint_ids, period)
        client_id  — PyBullet GUI client integer
        robot_id   — URDF body ID in the GUI client
        joint_ids  — {joint_name: pybullet_joint_index}
        period     — seconds per render frame (1 / viz_fps)
    """
    shm = SharedMemory(name=shm_name)
    arr = np.ndarray((1 + 3 + 4 + n_joints,), dtype=np.float64, buffer=shm.buf)

    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf", physicsClientId=client)
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(urdf_path)))
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, spawn_z],
        physicsClientId=client,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    p.setRealTimeSimulation(0, physicsClientId=client)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5, cameraYaw=90, cameraPitch=-20,
        cameraTargetPosition=[0.0, 0.0, 0.5],
        physicsClientId=client,
    )

    # Build joint name → index map
    joint_ids = {}
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        joint_ids[info[1].decode('utf-8')] = i

    # Signal ready to VizBridge.start() spin-wait: arr[0] was -1, now 0
    arr[0] = 0.0

    return client, robot_id, joint_ids, 1.0 / viz_fps


def _render_loop(client: int, robot_id: int, joint_ids: dict,
                 arr: np.ndarray, period: float,
                 joint_names: list, n_joints: int) -> None:
    """Infinite render loop. Runs until killed by parent (daemon process).

    Reads seq_num each frame; skips p.reset* calls if seq_num is unchanged
    (no-op frame). Camera interaction is handled natively by p.GUI event loop.
    """
    last_seq = -1.0

    while True:
        t0 = time.perf_counter()
        seq = arr[0]

        if seq != last_seq and seq >= 0.0:
            base_pos = [arr[1], arr[2], arr[3]]
            base_orn = [arr[4], arr[5], arr[6], arr[7]]
            p.resetBasePositionAndOrientation(
                robot_id, base_pos, base_orn, physicsClientId=client
            )
            for i, jname in enumerate(joint_names):
                jid = joint_ids.get(jname)
                if jid is not None:
                    p.resetJointState(
                        robot_id, jid, arr[8 + i], 0.0, physicsClientId=client
                    )
            p.stepSimulation(physicsClientId=client)
            last_seq = seq

        elapsed = time.perf_counter() - t0
        rem = period - elapsed
        if rem > 0.0:
            time.sleep(rem)


def main(shm_name: str, n_joints: int, joint_names: list,
         urdf_path: str, viz_fps: int, spawn_z: float) -> None:
    """Subprocess entry point called by VizBridge.start().

    Runs forever until killed by parent (daemon=True in VizBridge).
    """
    shm = SharedMemory(name=shm_name)
    arr = np.ndarray((1 + 3 + 4 + n_joints,), dtype=np.float64, buffer=shm.buf)

    client, robot_id, joint_ids, period = _init_pybullet(
        shm_name, n_joints, joint_names, urdf_path, viz_fps, spawn_z
    )
    _render_loop(client, robot_id, joint_ids, arr, period, joint_names, n_joints)
