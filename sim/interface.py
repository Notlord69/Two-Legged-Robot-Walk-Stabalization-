"""PyBullet / URDF abstraction layer for Siclo1.

ALL PyBullet calls go through this module. Direct p.* calls in control logic
are prohibited. This isolation enables future Gazebo swap by changing only
this file.

URDF parsers (get_joint_limits, get_segment_lengths) are pure stdlib —
no PyBullet import required to call them.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

_URDF_PATH = Path(__file__).parent.parent / "Siclo1.urdf"


def get_joint_limits() -> dict[str, dict[str, float]]:
    """Parse Siclo1.urdf, return joint limits by exact URDF joint name.

    Returns: {joint_name: {'lower': float, 'upper': float}}  angles in rad.
    Includes only joints with a <limit> element (revolute joints).
    Skips <joint> children inside <transmission> blocks (no 'type' attribute).
    """
    tree = ET.parse(_URDF_PATH)
    limits: dict[str, dict[str, float]] = {}
    for joint in tree.getroot().iter("joint"):
        if joint.get("type") is None:          # transmission child — skip
            continue
        limit_el = joint.find("limit")
        if limit_el is not None:
            limits[joint.attrib["name"]] = {
                "lower": float(limit_el.attrib["lower"]),
                "upper": float(limit_el.attrib["upper"]),
            }
    return limits


def get_segment_lengths() -> dict[str, dict[str, float]]:
    """Parse Siclo1.urdf joint origins, return IK segment lengths (m).

    Segment length = Euclidean norm of joint <origin xyz=...>.
    Left leg is the canonical reference (verified and patched 2026-04-04).

    Returns:
        {'left':  {'thigh': m, 'shank': m},
         'right': {'thigh': m, 'shank': m}}
    """
    tree = ET.parse(_URDF_PATH)
    joints = {
        j.attrib["name"]: j
        for j in tree.getroot().iter("joint")
        if j.get("type") is not None          # skip transmission children
    }

    def _norm(name: str) -> float:
        origin = joints[name].find("origin")
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        return math.sqrt(sum(v * v for v in xyz))

    return {
        "left":  {"thigh": _norm("Left_Knee"),  "shank": _norm("Left_Ankle")},
        "right": {"thigh": _norm("Right_Knee"), "shank": _norm("Right_Ankle")},
    }


# ── PyBullet wrappers ────────────────────────────────────────────────────────
# Never call p.* directly in control logic. Use these functions instead.

def get_joint_state(body_id: int, joint_index: int) -> tuple[float, float]:
    """Return (position_rad, velocity_rad_s) for one joint.

    Wraps p.getJointState. All p.* calls are confined to this file.
    """
    import pybullet as p                       # deferred: not needed for URDF-only callers
    state = p.getJointState(body_id, joint_index)
    return float(state[0]), float(state[1])


def set_joint_position_target(body_id: int, joint_index: int,
                               target_rad: float, kp: float,
                               kd: float, max_torque: float) -> None:
    """Apply PD position control to one joint.

    Wraps p.setJointMotorControl2. All p.* calls are confined to this file.
    kp: position gain (N·m/rad)
    kd: velocity gain (N·m·s/rad)
    max_torque: effort limit (N·m)
    """
    import pybullet as p                       # deferred: not needed for URDF-only callers
    p.setJointMotorControl2(
        body_id,
        joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_rad,
        positionGain=kp,
        velocityGain=kd,
        force=max_torque,
    )
