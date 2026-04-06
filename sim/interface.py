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


def add_debug_line(from_xyz, to_xyz, color_rgb,
                   width: float = 1.0,
                   replace_id: int = -1,
                   physics_client: int = 0) -> int:
    """Add or replace a PyBullet debug line. Returns item ID.

    from_xyz:      (x, y, z) start point (m, world frame)
    to_xyz:        (x, y, z) end point (m, world frame)
    color_rgb:     [r, g, b] each in [0, 1]
    width:         line width in pixels
    replace_id:    item ID to update in-place; pass -1 to create a new line
    physics_client: PyBullet client ID (must be a GUI client to be visible)

    All p.* calls are confined to this file (CLAUDE.md).
    """
    import pybullet as p                       # deferred: not needed for URDF-only callers
    kwargs = dict(lineColorRGB=color_rgb, lineWidth=width,
                  physicsClientId=physics_client)
    if replace_id >= 0:
        kwargs['replaceItemUniqueId'] = replace_id
    return p.addUserDebugLine(list(from_xyz), list(to_xyz), **kwargs)


def remove_debug_line(item_id: int, physics_client: int = 0) -> None:
    """Remove a PyBullet debug line by item ID. No-op if item_id < 0.

    All p.* calls are confined to this file (CLAUDE.md).
    """
    import pybullet as p                       # deferred: not needed for URDF-only callers
    if item_id >= 0:
        p.removeUserDebugItem(item_id, physicsClientId=physics_client)


def step_simulation(physics_client: int) -> None:
    """Advance physics by one timestep.

    physics_client: PyBullet client ID returned by p.connect()
    All p.* calls are confined to this file (CLAUDE.md).
    """
    import pybullet as p                       # deferred: not needed for URDF-only callers
    p.stepSimulation(physicsClientId=physics_client)
