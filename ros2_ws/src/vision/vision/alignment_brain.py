from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

try:
    from .vision_contracts import EE_SUCTION, ObbDetection, PickPlaceGoal, Pose2D, cell_to_robot_index
except ImportError:  # Allows direct execution/import from the legacy visions directory.
    from vision_contracts import EE_SUCTION, ObbDetection, PickPlaceGoal, Pose2D, cell_to_robot_index


T_CV_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])
CLASS_STEEL_PLATE = 0
CLASS_STEEL_CUBE = 1


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    t_world_camera: np.ndarray


@dataclass(frozen=True)
class TargetFrame:
    center: Pose2D
    width: float = 0.0
    height: float = 0.0


@dataclass(frozen=True)
class BrainConfig:
    plate_top_z: float = 0.03
    pick_z: float = 0.03
    place_z: float = 0.03
    min_confidence: float = 0.25
    axis_sample_px: float = 80.0


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_error_180(target: float, current: float) -> float:
    return (target - current + math.pi / 2.0) % math.pi - math.pi / 2.0


def pixel_to_world_on_plane(camera: CameraModel, u: float, v: float, plane_z: float) -> np.ndarray:
    ray_cv = np.array([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0, 0.0])
    ray_gl = T_CV_TO_GL @ ray_cv
    origin = camera.t_world_camera[:3, 3]
    direction = camera.t_world_camera[:3, :3] @ ray_gl[:3]
    direction = direction / np.linalg.norm(direction)
    if abs(direction[2]) < 1e-9:
        raise ValueError("Camera ray is parallel to the target plane.")
    distance = (plane_z - origin[2]) / direction[2]
    return origin + direction * distance


def longest_obb_axis(corners_px: tuple[tuple[float, float], ...]) -> np.ndarray:
    pts = np.array(corners_px, dtype=np.float64)
    edges = [pts[(idx + 1) % 4] - pts[idx] for idx in range(4)]
    edge = edges[int(np.argmax([np.linalg.norm(e) for e in edges]))]
    norm = np.linalg.norm(edge)
    if norm < 1e-9:
        return np.array([1.0, 0.0])
    return edge / norm


def obb_to_plate_pose(det: ObbDetection, camera: CameraModel, config: BrainConfig) -> Pose2D:
    center = np.array(det.center_px, dtype=np.float64)
    center_world = pixel_to_world_on_plane(camera, center[0], center[1], config.plate_top_z)

    if det.class_id == CLASS_STEEL_CUBE:
        return Pose2D(
            x=float(center_world[0]),
            y=float(center_world[1]),
            z=float(config.pick_z),
            yaw=0.0,
        )

    axis = longest_obb_axis(det.corners_px)
    end_px = center + axis * config.axis_sample_px
    end_world = pixel_to_world_on_plane(camera, end_px[0], end_px[1], config.plate_top_z)

    delta = end_world - center_world
    yaw = math.atan2(float(delta[1]), float(delta[0]))
    return Pose2D(
        x=float(center_world[0]),
        y=float(center_world[1]),
        z=float(config.pick_z),
        yaw=normalize_angle(yaw),
    )


def make_pick_place_goal(
    cell_name: str,
    det: ObbDetection,
    target: TargetFrame,
    camera: CameraModel,
    config: BrainConfig | None = None,
) -> PickPlaceGoal:
    config = config or BrainConfig()
    robot_index = cell_to_robot_index(cell_name)

    if not det.valid or det.confidence < config.min_confidence:
        empty = Pose2D(0.0, 0.0, 0.0, 0.0)
        return PickPlaceGoal(False, robot_index, det.confidence, empty, empty, 0.0, 0.0, 0.0, EE_SUCTION)

    pick = obb_to_plate_pose(det, camera, config)
    place = Pose2D(target.center.x, target.center.y, config.place_z, target.center.yaw)
    dx = place.x - pick.x
    dy = place.y - pick.y
    dyaw = 0.0 if det.class_id == CLASS_STEEL_CUBE else angle_error_180(place.yaw, pick.yaw)

    return PickPlaceGoal(
        valid=True,
        robot_index=robot_index,
        confidence=det.confidence,
        pick=pick,
        place=place,
        dx=dx,
        dy=dy,
        dyaw=dyaw,
        ee_type=EE_SUCTION,
    )


def default_six_cell_targets() -> dict[str, TargetFrame]:
    return {
        "cell_01": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
        "cell_02": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
        "cell_03": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
        "cell_04": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
        "cell_05": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
        "cell_06": TargetFrame(Pose2D(0.45, -0.22, 0.03, 0.0)),
    }
