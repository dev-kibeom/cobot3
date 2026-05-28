from __future__ import annotations

# Run with Isaac Sim Python, for example:
# ~/.local/share/ov/pkg/isaac-sim-*/python.sh \
#   tools/isaac/isaac_vision_prototype.py
#
# Optional YOLO path:
# ~/.local/share/ov/pkg/isaac-sim-*/python.sh \
#   tools/isaac/isaac_vision_prototype.py \
#   --detector yolo --model vision/models/yolo11s_obb_metal_hard-v2_refinetune_best.pt

import argparse
from dataclasses import dataclass
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Isaac Sim top-camera vision prototype for plate pose testing."
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim without a GUI window.",
    )
    parser.add_argument(
        "--detector",
        choices=("auto", "color", "yolo"),
        default="auto",
        help="Detection backend. auto uses YOLO only when --model is provided.",
    )
    parser.add_argument(
        "--model",
        default="",
        help="Ultralytics YOLO OBB model path, e.g. vision/models/yolo11s_obb_metal_hard-v2_refinetune_best.pt or .engine.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=80,
        help="Warm-up/render frames before grabbing the camera image.",
    )
    parser.add_argument(
        "--plate-yaw-deg",
        type=float,
        default=37.0,
        help="Ground-truth yaw of the synthetic plate.",
    )
    parser.add_argument(
        "--save-dir",
        default=str(Path(__file__).resolve().parents[2] / "work"),
        help="Directory for captured and annotated images.",
    )
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": ARGS.headless})

import math

import cv2
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdLux

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils


WIDTH = 960
HEIGHT = 720
FX = 850.0
FY = 850.0
CX = WIDTH / 2.0
CY = HEIGHT / 2.0
DIST_COEFFS = [0.0] * 12

CAMERA_PRIM_PATH = "/World/top_camera"
PLATE_PRIM_PATH = "/World/test_plate"
PLATE_TOP_Z = 0.031

# USD/Isaac camera uses an OpenGL-like camera frame. OpenCV uses +Z forward
# and +Y down, so this bridges the two conventions.
T_CV_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])


@dataclass
class PlateDetection:
    source: str
    center_px: np.ndarray
    axis_angle_image_rad: float
    confidence: float
    box_px: np.ndarray | None = None


@dataclass
class PlatePose:
    position_world: np.ndarray
    yaw_rad: float
    confidence: float


def get_world_transform(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    return np.array(matrix, dtype=np.float64).T


def normalize_angle_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_error_mod_180_deg(estimate: float, truth: float) -> float:
    diff = (estimate - truth + math.pi / 2.0) % math.pi - math.pi / 2.0
    return abs(math.degrees(diff))


def yaw_from_transform(transform: np.ndarray) -> float:
    rot = transform[:3, :3].copy()
    for idx in range(3):
        norm = np.linalg.norm(rot[:, idx])
        if norm > 1e-9:
            rot[:, idx] /= norm
    return math.atan2(rot[1, 0], rot[0, 0])


def rgba_to_bgr(rgba: np.ndarray) -> np.ndarray:
    arr = np.asarray(rgba)
    if arr.size == 0:
        raise RuntimeError("Camera returned an empty RGBA buffer.")
    if arr.dtype != np.uint8:
        if float(np.nanmax(arr)) <= 1.0:
            arr = arr * 255.0
        arr = np.clip(arr, 0.0, 255.0).astype(np.uint8)
    rgb = arr[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def setup_scene(plate_yaw_deg: float) -> tuple[World, Camera]:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    stage = world.stage
    light = UsdLux.DistantLight.Define(stage, "/World/key_light")
    light.CreateIntensityAttr(700.0)

    plate_orientation = rot_utils.euler_angles_to_quats(
        np.array([0.0, 0.0, plate_yaw_deg]), degrees=True
    )
    world.scene.add(
        VisualCuboid(
            prim_path=PLATE_PRIM_PATH,
            name="test_plate",
            position=np.array([0.42, 0.08, PLATE_TOP_Z / 2.0]),
            orientation=plate_orientation,
            scale=np.array([0.34, 0.14, PLATE_TOP_Z]),
            color=np.array([0.08, 0.32, 0.95]),
        )
    )

    world.scene.add(
        VisualCuboid(
            prim_path="/World/goal_marker",
            name="goal_marker",
            position=np.array([0.62, -0.22, 0.002]),
            scale=np.array([0.24, 0.12, 0.004]),
            color=np.array([0.0, 0.9, 0.25]),
        )
    )

    camera = Camera(
        prim_path=CAMERA_PRIM_PATH,
        position=np.array([0.42, 0.02, 1.28]),
        frequency=20,
        resolution=(WIDTH, HEIGHT),
        orientation=rot_utils.euler_angles_to_quats(
            np.array([0.0, 90.0, 180.0]), degrees=True
        ),
    )

    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(
        cx=CX,
        cy=CY,
        fx=FX,
        fy=FY,
        pinhole=DIST_COEFFS,
    )
    return world, camera


def detect_plate_by_color(bgr: np.ndarray) -> PlateDetection | None:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # The synthetic plate is blue. This fallback keeps the prototype usable
    # before a trained YOLO11s-OBB model exists.
    lower = np.array([90, 45, 35], dtype=np.uint8)
    upper = np.array([132, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < 300.0:
        return None

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(np.float32)
    center = np.array(rect[0], dtype=np.float64)

    edges = [box[(idx + 1) % 4] - box[idx] for idx in range(4)]
    lengths = [float(np.linalg.norm(edge)) for edge in edges]
    long_edge = edges[int(np.argmax(lengths))]
    axis_angle = math.atan2(float(long_edge[1]), float(long_edge[0]))

    confidence = min(1.0, area / (WIDTH * HEIGHT * 0.02))
    return PlateDetection(
        source="color",
        center_px=center,
        axis_angle_image_rad=axis_angle,
        confidence=confidence,
        box_px=box,
    )


def to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def detect_plate_by_yolo(bgr: np.ndarray, model_path: str) -> PlateDetection | None:
    from ultralytics import YOLO

    model = YOLO(model_path)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    results = model.predict(source=rgb, imgsz=960, conf=0.25, device=0, verbose=False)
    if not results:
        return None

    result = results[0]
    obb = getattr(result, "obb", None)
    if obb is not None and getattr(obb, "xywhr", None) is not None:
        xywhr = to_numpy(obb.xywhr)
        if xywhr.size:
            conf = to_numpy(obb.conf) if getattr(obb, "conf", None) is not None else np.ones(len(xywhr))
            best = int(np.argmax(conf))
            cx, cy, width, height, theta = xywhr[best].astype(float)
            axis_angle = float(theta)
            if height > width:
                axis_angle += math.pi / 2.0

            box = None
            if getattr(obb, "xyxyxyxy", None) is not None:
                box = to_numpy(obb.xyxyxyxy[best]).reshape(4, 2).astype(np.float32)

            return PlateDetection(
                source="yolo-obb",
                center_px=np.array([cx, cy], dtype=np.float64),
                axis_angle_image_rad=axis_angle,
                confidence=float(conf[best]),
                box_px=box,
            )

    boxes = getattr(result, "boxes", None)
    if boxes is not None and getattr(boxes, "xyxy", None) is not None:
        xyxy = to_numpy(boxes.xyxy)
        if xyxy.size:
            conf = to_numpy(boxes.conf) if getattr(boxes, "conf", None) is not None else np.ones(len(xyxy))
            best = int(np.argmax(conf))
            x1, y1, x2, y2 = xyxy[best].astype(float)
            center = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=np.float64)
            box = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
            return PlateDetection(
                source="yolo-box",
                center_px=center,
                axis_angle_image_rad=0.0,
                confidence=float(conf[best]),
                box_px=box,
            )

    return None


def detect_plate(bgr: np.ndarray, detector: str, model_path: str) -> PlateDetection | None:
    if detector == "yolo" or (detector == "auto" and model_path):
        if not model_path:
            raise ValueError("--detector yolo requires --model.")
        try:
            return detect_plate_by_yolo(bgr, model_path)
        except Exception as exc:
            if detector == "yolo":
                raise
            print(f"[WARN] YOLO unavailable, falling back to color detector: {exc}")

    return detect_plate_by_color(bgr)


def pixel_ray_world(u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
    x = (u - CX) / FX
    y = (v - CY) / FY
    ray_cv = np.array([x, y, 1.0, 0.0], dtype=np.float64)
    ray_gl = T_CV_TO_GL @ ray_cv

    t_world_camera = get_world_transform(CAMERA_PRIM_PATH)
    origin = t_world_camera[:3, 3]
    direction = t_world_camera[:3, :3] @ ray_gl[:3]
    direction = direction / np.linalg.norm(direction)
    return origin, direction


def pixel_to_world_on_plane(u: float, v: float, plane_z: float) -> np.ndarray:
    origin, direction = pixel_ray_world(u, v)
    if abs(direction[2]) < 1e-8:
        raise RuntimeError("Camera ray is parallel to the target plane.")
    distance = (plane_z - origin[2]) / direction[2]
    if distance < 0.0:
        raise RuntimeError("Target plane is behind the camera.")
    return origin + direction * distance


def detection_to_world_pose(detection: PlateDetection) -> PlatePose:
    center = detection.center_px
    center_world = pixel_to_world_on_plane(center[0], center[1], PLATE_TOP_Z)

    axis_len_px = 80.0
    axis_px = np.array(
        [
            math.cos(detection.axis_angle_image_rad),
            math.sin(detection.axis_angle_image_rad),
        ],
        dtype=np.float64,
    )
    end_px = center + axis_px * axis_len_px
    end_world = pixel_to_world_on_plane(end_px[0], end_px[1], PLATE_TOP_Z)
    delta = end_world - center_world
    yaw = math.atan2(float(delta[1]), float(delta[0]))

    return PlatePose(
        position_world=center_world,
        yaw_rad=normalize_angle_pi(yaw),
        confidence=detection.confidence,
    )


def annotate_image(bgr: np.ndarray, detection: PlateDetection, pose: PlatePose) -> np.ndarray:
    annotated = bgr.copy()
    if detection.box_px is not None:
        box = detection.box_px.astype(np.int32)
        cv2.polylines(annotated, [box], isClosed=True, color=(0, 255, 255), thickness=2)

    center = tuple(detection.center_px.astype(int))
    cv2.circle(annotated, center, 5, (0, 0, 255), -1)

    axis_len = 70
    end = (
        int(center[0] + math.cos(detection.axis_angle_image_rad) * axis_len),
        int(center[1] + math.sin(detection.axis_angle_image_rad) * axis_len),
    )
    cv2.line(annotated, center, end, (0, 0, 255), 2)

    text = (
        f"{detection.source} conf={pose.confidence:.2f} "
        f"x={pose.position_world[0]:+.3f} y={pose.position_world[1]:+.3f} "
        f"yaw={math.degrees(pose.yaw_rad):+.1f}deg"
    )
    cv2.putText(
        annotated,
        text,
        (20, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


def main() -> None:
    save_dir = Path(ARGS.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    world, camera = setup_scene(ARGS.plate_yaw_deg)

    print("[INFO] Warming up camera...")
    for idx in range(max(1, ARGS.frames)):
        world.step(render=True)
        if idx in (0, ARGS.frames // 2, ARGS.frames - 1):
            print(f"[INFO] rendered frame {idx + 1}/{ARGS.frames}")

    bgr = rgba_to_bgr(camera.get_rgba())
    raw_path = save_dir / "prototype_camera_raw.png"
    cv2.imwrite(str(raw_path), bgr)

    detection = detect_plate(bgr, ARGS.detector, ARGS.model)
    if detection is None:
        print("[FAIL] No plate detection.")
        print(f"[INFO] Raw image saved to: {raw_path}")
        world.stop()
        simulation_app.close()
        return

    pose = detection_to_world_pose(detection)

    plate_tf = get_world_transform(PLATE_PRIM_PATH)
    truth_position = plate_tf[:3, 3]
    truth_yaw = normalize_angle_pi(yaw_from_transform(plate_tf))

    xy_error_mm = float(np.linalg.norm(pose.position_world[:2] - truth_position[:2]) * 1000.0)
    yaw_error_deg = yaw_error_mod_180_deg(pose.yaw_rad, truth_yaw)

    annotated = annotate_image(bgr, detection, pose)
    annotated_path = save_dir / "prototype_detection.png"
    cv2.imwrite(str(annotated_path), annotated)

    print("\n=== Isaac Vision Prototype Result ===")
    print(f"detector:      {detection.source}")
    print(f"pixel center:  u={detection.center_px[0]:.1f}, v={detection.center_px[1]:.1f}")
    print(
        "world pose:    "
        f"x={pose.position_world[0]:+.4f}, "
        f"y={pose.position_world[1]:+.4f}, "
        f"z={pose.position_world[2]:+.4f}, "
        f"yaw={math.degrees(pose.yaw_rad):+.2f} deg"
    )
    print(
        "ground truth:  "
        f"x={truth_position[0]:+.4f}, "
        f"y={truth_position[1]:+.4f}, "
        f"z={truth_position[2]:+.4f}, "
        f"yaw={math.degrees(truth_yaw):+.2f} deg"
    )
    print(f"xy error:      {xy_error_mm:.2f} mm")
    print(f"yaw error:     {yaw_error_deg:.2f} deg (180-deg symmetry)")
    print(f"raw image:     {raw_path}")
    print(f"annotated:     {annotated_path}")
    print("=====================================\n")

    world.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
