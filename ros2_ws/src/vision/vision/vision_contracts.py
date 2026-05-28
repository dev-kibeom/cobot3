from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


OBB_PAYLOAD_SIZE = 16
GOAL_PAYLOAD_SIZE = 15

EE_NONE = 0.0
EE_SUCTION = 1.0
EE_GRIPPER = 2.0


@dataclass(frozen=True)
class ObbDetection:
    valid: bool
    class_id: int
    confidence: float
    center_px: tuple[float, float]
    corners_px: tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]
    width_px: float
    height_px: float
    angle_rad: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class PickPlaceGoal:
    valid: bool
    robot_index: int
    confidence: float
    pick: Pose2D
    place: Pose2D
    dx: float
    dy: float
    dyaw: float
    ee_type: float = EE_SUCTION


def empty_obb() -> ObbDetection:
    return ObbDetection(
        valid=False,
        class_id=-1,
        confidence=0.0,
        center_px=(0.0, 0.0),
        corners_px=((0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
        width_px=0.0,
        height_px=0.0,
        angle_rad=0.0,
    )


def encode_obb(det: ObbDetection) -> list[float]:
    values = [
        1.0 if det.valid else 0.0,
        float(det.class_id),
        float(det.confidence),
        float(det.center_px[0]),
        float(det.center_px[1]),
    ]
    for x, y in det.corners_px:
        values.extend([float(x), float(y)])
    values.extend([float(det.width_px), float(det.height_px), float(det.angle_rad)])
    if len(values) != OBB_PAYLOAD_SIZE:
        raise ValueError(f"OBB payload size mismatch: {len(values)}")
    return values


def decode_obb(data: Iterable[float]) -> ObbDetection:
    values = list(data)
    if len(values) < OBB_PAYLOAD_SIZE:
        values.extend([0.0] * (OBB_PAYLOAD_SIZE - len(values)))
    corners = (
        (float(values[5]), float(values[6])),
        (float(values[7]), float(values[8])),
        (float(values[9]), float(values[10])),
        (float(values[11]), float(values[12])),
    )
    return ObbDetection(
        valid=bool(values[0] >= 0.5),
        class_id=int(values[1]),
        confidence=float(values[2]),
        center_px=(float(values[3]), float(values[4])),
        corners_px=corners,
        width_px=float(values[13]),
        height_px=float(values[14]),
        angle_rad=float(values[15]),
    )


def encode_goal(goal: PickPlaceGoal) -> list[float]:
    values = [
        1.0 if goal.valid else 0.0,
        float(goal.robot_index),
        float(goal.confidence),
        goal.pick.x,
        goal.pick.y,
        goal.pick.z,
        goal.pick.yaw,
        goal.place.x,
        goal.place.y,
        goal.place.z,
        goal.place.yaw,
        goal.dx,
        goal.dy,
        goal.dyaw,
        goal.ee_type,
    ]
    if len(values) != GOAL_PAYLOAD_SIZE:
        raise ValueError(f"Goal payload size mismatch: {len(values)}")
    return [float(v) for v in values]


def decode_goal(data: Iterable[float]) -> PickPlaceGoal:
    values = list(data)
    if len(values) < GOAL_PAYLOAD_SIZE:
        values.extend([0.0] * (GOAL_PAYLOAD_SIZE - len(values)))
    return PickPlaceGoal(
        valid=bool(values[0] >= 0.5),
        robot_index=int(values[1]),
        confidence=float(values[2]),
        pick=Pose2D(float(values[3]), float(values[4]), float(values[5]), float(values[6])),
        place=Pose2D(float(values[7]), float(values[8]), float(values[9]), float(values[10])),
        dx=float(values[11]),
        dy=float(values[12]),
        dyaw=float(values[13]),
        ee_type=float(values[14]),
    )


def cell_to_robot_index(cell_name: str) -> int:
    tail = cell_name.rsplit("_", 1)[-1]
    return int(tail) if tail.isdigit() else 0
