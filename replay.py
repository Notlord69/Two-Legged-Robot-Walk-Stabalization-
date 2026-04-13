"""
================================================================================
PROJECT SICLO1 — POST-RUN 3D REPLAY
================================================================================

Reads poses.npy from a session folder and plays back the robot in 3D.
Optionally records walk.mp4 via VideoRecorder (TinyRenderer, no GPU needed).

No imports from HeartBeat.py or any control module (stability, gait, WBC).
Only uses: recorder.py, pybullet, numpy.

Usage:
    python3 replay.py sessions/2026-04-13_14-30-00/
    python3 replay.py sessions/2026-04-13_14-30-00/ --speed 0.5
    python3 replay.py sessions/2026-04-13_14-30-00/ --record
    python3 replay.py sessions/2026-04-13_14-30-00/ --headless --record

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import argparse
import os
import time
import numpy as np
import pybullet as p
import pybullet_data

# ── Constants ─────────────────────────────────────────────────────────────────

TARGET_DT: float = 0.01       # s — matches HeartBeat.py physics step

URDF_SPAWN_Z: float = 0.8806  # m — URDF-aligned spawn height

# Joint names in the order they appear in poses.npy columns 8–13.
# Must match _JOINT_COLS in pose_logger.py.
_REPLAY_JOINT_NAMES = [
    'Left_Hip_Forwards',   # col 8
    'Left_Knee',           # col 9
    'Left_Ankle',          # col 10
    'Right_Hip_Fowards',   # col 11 — URDF typo preserved
    'Right_Knee',          # col 12
    'Right_Ankle',         # col 13
]

# poses.npy column slices
_COL_POS     = slice(1, 4)
_COL_ORN     = slice(4, 8)
_COL_J_START = 8
_COL_J_END   = 14


# ── Public helpers ─────────────────────────────────────────────────────────────

def _load_poses(session_path: str) -> np.ndarray:
    """Load poses.npy from session_path. Raises FileNotFoundError if absent."""
    path = os.path.join(session_path, 'poses.npy')
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"poses.npy not found in {session_path}. "
            "Run with --gui to generate pose logs."
        )
    return np.load(path)


def _build_joint_map(robot_id: int, client: int) -> dict:
    """Return {joint_name: joint_index} for the 6 active replay joints."""
    joint_map = {}
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        name = info[1].decode('utf-8')
        if name in _REPLAY_JOINT_NAMES:
            joint_map[name] = i
    return joint_map


# ── Main replay function ───────────────────────────────────────────────────────

def replay(
    session_path: str,
    speed:        float = 1.0,
    record:       bool  = False,
    headless:     bool  = False,
) -> None:
    """Play back a session in PyBullet.

    session_path : folder containing poses.npy
    speed        : playback speed multiplier (1.0 = real-time, 0.5 = half speed)
    record       : write walk.mp4 to session_path via VideoRecorder
    headless     : use p.DIRECT instead of p.GUI (combine with record for MP4-only)
    """
    poses = _load_poses(session_path)
    n_frames = len(poses)
    print(f"[replay] {n_frames} frames loaded from {session_path}")

    client = p.connect(p.DIRECT if headless else p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.loadURDF('plane.urdf', physicsClientId=client)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path   = os.path.join(current_dir, 'Siclo1.urdf')
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"Siclo1.urdf not found at {urdf_path}")

        p.setAdditionalSearchPath(os.path.dirname(urdf_path))
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0.0, 0.0, URDF_SPAWN_Z],
            physicsClientId=client,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        joint_map = _build_joint_map(robot_id, client)

        if not headless:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5, cameraYaw=90, cameraPitch=-20,
                cameraTargetPosition=[0.0, 0.0, 0.5],
                physicsClientId=client,
            )

        # Optional video recording — TinyRenderer, software-only, no GPU needed
        recorder = None
        if record:
            from recorder import VideoRecorder
            recorder = VideoRecorder(
                physics_client=client,
                session_path=session_path,
            )
            recorder.start()
            print(f"[replay] Recording to {recorder.video_path}")

        frame_dt = TARGET_DT / max(speed, 1e-6)  # s per frame at chosen speed

        for row in poses:
            t_frame = time.perf_counter()

            base_pos = tuple(float(v) for v in row[_COL_POS])
            base_orn = tuple(float(v) for v in row[_COL_ORN])

            p.resetBasePositionAndOrientation(
                robot_id, base_pos, base_orn, physicsClientId=client)

            angles = row[_COL_J_START:_COL_J_END]
            for j, jname in enumerate(_REPLAY_JOINT_NAMES):
                jid = joint_map.get(jname)
                if jid is not None:
                    p.resetJointState(
                        robot_id, jid, float(angles[j]), 0.0,
                        physicsClientId=client,
                    )

            p.stepSimulation(physicsClientId=client)

            elapsed   = time.perf_counter() - t_frame
            remaining = frame_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

        if recorder is not None:
            video_path = recorder.stop()
            recorder.join(timeout=5.0)
            if os.path.isfile(video_path):
                print(f"[replay] Video saved: {video_path}")
            else:
                print(f"[replay][WARN] walk.mp4 not found at {video_path}")

        if not headless:
            print("[replay] Playback complete. Press Ctrl-C to close.")
            try:
                while p.isConnected(physicsClientId=client):
                    time.sleep(0.05)
            except KeyboardInterrupt:
                pass

    finally:
        if p.isConnected(physicsClientId=client):
            p.disconnect(physicsClientId=client)


# ── CLI entry point ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Siclo1 post-run 3D session replay'
    )
    parser.add_argument('session', help='Path to session folder (contains poses.npy)')
    parser.add_argument('--speed', type=float, default=1.0, metavar='X',
                        help='Playback speed multiplier (default: 1.0 = real-time)')
    parser.add_argument('--record', action='store_true',
                        help='Record walk.mp4 to the session folder')
    parser.add_argument('--headless', action='store_true',
                        help='Use p.DIRECT (no window); use with --record for MP4-only')
    args = parser.parse_args()
    replay(
        session_path=args.session,
        speed=args.speed,
        record=args.record,
        headless=args.headless,
    )


if __name__ == '__main__':
    main()
