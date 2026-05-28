from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from .vision_contracts import ObbDetection, empty_obb
except ImportError:  # Allows direct execution/import from the legacy visions directory.
    from vision_contracts import ObbDetection, empty_obb


CLASS_STEEL_PLATE = 0
CLASS_STEEL_CUBE = 1


@dataclass(frozen=True)
class ColorShapeConfig:
    min_area_px: float = 180.0
    max_area_fraction: float = 0.18
    min_rectangularity: float = 0.22
    plate_aspect_min: float = 1.55
    lab_delta_threshold: float = 16.0
    low_saturation_margin: float = 0.65
    morph_kernel: int = 5


def _largest_surface_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, w = bgr.shape[:2]

    # Work surfaces in our Isaac prototypes are colored and non-black.
    mask = cv2.inRange(hsv, np.array([35, 20, 35], np.uint8), np.array([125, 255, 255], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.full((h, w), 255, np.uint8)

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 0.12 * h * w:
        return np.full((h, w), 255, np.uint8)

    surface = np.zeros((h, w), np.uint8)
    cv2.drawContours(surface, [contour], -1, 255, -1)
    return cv2.erode(surface, np.ones((9, 9), np.uint8), iterations=1)


def _estimate_surface_stats(bgr: np.ndarray, surface_mask: np.ndarray) -> tuple[np.ndarray, float, float]:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    valid = surface_mask > 0
    if int(valid.sum()) < 100:
        valid = np.ones(surface_mask.shape, dtype=bool)
    return (
        np.median(lab[valid].reshape(-1, 3), axis=0).astype(np.float32),
        float(np.median(hsv[:, :, 1][valid])),
        float(np.median(hsv[:, :, 2][valid])),
    )


def _candidate_mask(bgr: np.ndarray, config: ColorShapeConfig, surface: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    bg_lab, bg_sat, bg_val = _estimate_surface_stats(bgr, surface)
    delta = np.linalg.norm(lab - bg_lab.reshape(1, 1, 3), axis=2)

    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    low_sat = sat < max(38.0, bg_sat * config.low_saturation_margin)
    different = delta > config.lab_delta_threshold
    dark_edge = val < (bg_val - 18.0)
    bright_face = val > (bg_val + 28.0)

    edges = cv2.Canny(gray, 40, 120)
    edge_dilate = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1) > 0

    mask = (surface > 0) & ((low_sat & different) | (dark_edge & different) | (bright_face & low_sat) | (edge_dilate & different))
    mask = mask.astype(np.uint8) * 255

    k = max(3, int(config.morph_kernel) | 1)
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def _rect_angle_and_size(box: np.ndarray) -> tuple[float, float, float]:
    edges = [box[(idx + 1) % 4] - box[idx] for idx in range(4)]
    lengths = [float(np.linalg.norm(edge)) for edge in edges]
    major_idx = int(np.argmax(lengths))
    major = edges[major_idx]
    width = lengths[major_idx]
    height = lengths[(major_idx + 1) % 4]
    angle = math.atan2(float(major[1]), float(major[0]))
    return width, height, angle


def detect_color_shape_obb(bgr: np.ndarray, config: ColorShapeConfig | None = None) -> ObbDetection:
    config = config or ColorShapeConfig()
    h, w = bgr.shape[:2]
    max_area = config.max_area_fraction * h * w
    surface = _largest_surface_mask(bgr)
    mask = _candidate_mask(bgr, config, surface)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: tuple[float, ObbDetection] | None = None
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area_px or area > max_area:
            continue

        rect = cv2.minAreaRect(contour)
        (_, _), (rw, rh), _ = rect
        rect_area = float(rw * rh)
        if rect_area <= 1.0:
            continue

        rectangularity = area / rect_area
        if rectangularity < config.min_rectangularity:
            continue

        box = cv2.boxPoints(rect).astype(np.float32)
        width, height, angle = _rect_angle_and_size(box)
        if min(width, height) < 8.0:
            continue

        aspect = max(width, height) / max(1.0, min(width, height))
        class_id = CLASS_STEEL_PLATE if aspect >= config.plate_aspect_min else CLASS_STEEL_CUBE

        contour_mask = np.zeros((h, w), np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        obj = gray[contour_mask > 0]
        bg = gray[(mask == 0) & (surface > 0)]
        contrast = abs(float(obj.mean()) - float(bg.mean())) if len(obj) and len(bg) else 0.0
        confidence = min(0.99, max(0.05, 0.25 + 0.45 * rectangularity + 0.30 * min(1.0, contrast / 80.0)))
        score = confidence * area

        center = tuple(float(v) for v in np.mean(box, axis=0))
        det = ObbDetection(
            valid=True,
            class_id=class_id,
            confidence=float(confidence),
            center_px=center,
            corners_px=tuple((float(x), float(y)) for x, y in box),
            width_px=float(width),
            height_px=float(height),
            angle_rad=float(angle),
        )
        if best is None or score > best[0]:
            best = (score, det)

    return best[1] if best else empty_obb()


def _normalized_yaw(angle_rad: float) -> float:
    yaw = float(angle_rad)
    while yaw > math.pi / 2.0:
        yaw -= math.pi
    while yaw < -math.pi / 2.0:
        yaw += math.pi
    return yaw


def _draw_center_cross(out: np.ndarray, center_px: tuple[float, float], size: int = 18) -> None:
    cx, cy = np.round(center_px).astype(int)
    cv2.line(out, (cx - size, cy), (cx + size, cy), (0, 0, 255), 2)
    cv2.line(out, (cx, cy - size), (cx, cy + size), (0, 0, 255), 2)
    cv2.circle(out, (cx, cy), 3, (255, 255, 255), -1)


def _draw_alignment_axes(out: np.ndarray, det: ObbDetection) -> float:
    cx, cy = np.array(det.center_px, dtype=np.float64)
    yaw = _normalized_yaw(det.angle_rad)
    major_len = max(28.0, min(90.0, det.width_px * 0.55))
    minor_len = max(18.0, min(65.0, det.height_px * 0.55))
    major = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
    minor = np.array([-major[1], major[0]], dtype=np.float64)
    cv2.arrowedLine(
        out,
        tuple(np.round([cx, cy] - major * major_len).astype(int)),
        tuple(np.round([cx, cy] + major * major_len).astype(int)),
        (255, 0, 255),
        2,
        tipLength=0.18,
    )
    cv2.line(
        out,
        tuple(np.round([cx, cy] - minor * minor_len).astype(int)),
        tuple(np.round([cx, cy] + minor * minor_len).astype(int)),
        (255, 255, 0),
        2,
    )
    return yaw


def draw_detection(bgr: np.ndarray, det: ObbDetection) -> np.ndarray:
    out = bgr.copy()
    if not det.valid:
        cv2.putText(out, "no object", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out
    pts = np.array(det.corners_px, np.int32)
    label = "steel_plate" if det.class_id == CLASS_STEEL_PLATE else "steel_cube"
    cv2.polylines(out, [pts], True, (0, 255, 255), 2)
    _draw_center_cross(out, det.center_px)
    yaw = _draw_alignment_axes(out, det)
    cv2.putText(
        out,
        f"color_obb {label} conf={det.confidence:.2f}",
        tuple(np.round(pts[0]).astype(int)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )
    cx, cy = np.round(det.center_px).astype(int)
    cv2.putText(
        out,
        f"center=({cx},{cy}) align yaw={math.degrees(yaw):+.1f}deg",
        (max(8, cx - 145), min(out.shape[0] - 12, cy + 34)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
    )
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--debug-out", default="")
    parser.add_argument("--min-area-px", type=float, default=180.0)
    parser.add_argument("--plate-aspect-min", type=float, default=1.55)
    parser.add_argument("--lab-delta-threshold", type=float, default=16.0)
    args = parser.parse_args()

    image_path = Path(args.image)
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    config = ColorShapeConfig(
        min_area_px=args.min_area_px,
        plate_aspect_min=args.plate_aspect_min,
        lab_delta_threshold=args.lab_delta_threshold,
    )
    det = detect_color_shape_obb(bgr, config)
    print(det)
    if args.debug_out:
        debug_path = Path(args.debug_out)
        debug_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(debug_path), draw_detection(bgr, det))
        print(f"debug: {debug_path}")


if __name__ == "__main__":
    main()
