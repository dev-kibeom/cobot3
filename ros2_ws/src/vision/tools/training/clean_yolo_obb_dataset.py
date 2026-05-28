from __future__ import annotations

# Build a non-destructive clean copy of a YOLO OBB dataset.
#
# The current synthetic dataset can contain labels whose projected object is
# not actually visible in the rendered RGB image. Those samples train YOLO to
# detect invisible objects, so this script filters them into a new dataset.
#
# Example:
# python3 tools/training/clean_yolo_obb_dataset.py
#
# Then train with:
# python3 tools/training/train_yolo11s_obb.py \
#   --data work/datasets/metal_objects_obb_visible/metal_objects_obb.yaml \
#   --epochs 50 --name yolo11s_obb_visible_e50

import argparse
from dataclasses import dataclass
from pathlib import Path
import shutil

import cv2
import numpy as np


WIDTH_FALLBACK = 640
HEIGHT_FALLBACK = 480
NAMES = {0: "steel_plate", 1: "steel_cube"}


@dataclass(frozen=True)
class LabelCheck:
    ok: bool
    reason: str
    class_id: int = -1
    contrast: float = 0.0
    edge_density: float = 0.0
    area: int = 0


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        default=str(root / "work" / "datasets" / "metal_objects_obb"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(root / "work" / "datasets" / "metal_objects_obb_visible"),
    )
    parser.add_argument("--min-visible-contrast", type=float, default=8.0)
    parser.add_argument("--min-visible-edge-density", type=float, default=0.012)
    parser.add_argument("--min-polygon-area", type=int, default=20)
    parser.add_argument("--clean-output", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def parse_obb_line(line: str, width: int, height: int) -> tuple[int, np.ndarray]:
    values = line.strip().split()
    if len(values) != 9:
        raise ValueError(f"expected 9 values, got {len(values)}")
    class_id = int(float(values[0]))
    coords = np.array([float(v) for v in values[1:]], dtype=np.float64).reshape(4, 2)
    coords *= np.array([width, height], dtype=np.float64)
    return class_id, coords.astype(np.int32)


def check_polygon_visible(
    bgr: np.ndarray,
    polygon: np.ndarray,
    class_id: int,
    min_contrast: float,
    min_edge_density: float,
    min_area: int,
) -> LabelCheck:
    height, width = bgr.shape[:2]
    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, polygon, 255)
    area = int(np.count_nonzero(mask))
    if area < min_area:
        return LabelCheck(False, "tiny_polygon", class_id=class_id, area=area)

    x, y, w, h = cv2.boundingRect(polygon)
    pad = 28
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(width, x + w + pad)
    y1 = min(height, y + h + pad)
    nearby = np.zeros_like(mask)
    nearby[y0:y1, x0:x1] = 255
    bg_mask = (nearby > 0) & (mask == 0)
    if int(bg_mask.sum()) < 50:
        bg_mask = mask == 0

    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    obj_lab = np.median(lab[mask > 0].reshape(-1, 3), axis=0)
    bg_lab = np.median(lab[bg_mask].reshape(-1, 3), axis=0)
    contrast = float(np.linalg.norm(obj_lab - bg_lab))

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 35, 115)
    edge_density = float(np.count_nonzero(edges[mask > 0]) / max(1, area))
    ok = contrast >= min_contrast or edge_density >= min_edge_density
    return LabelCheck(
        ok=ok,
        reason="ok" if ok else "low_visibility",
        class_id=class_id,
        contrast=contrast,
        edge_density=edge_density,
        area=area,
    )


def check_sample(image_path: Path, label_path: Path, args) -> list[LabelCheck]:
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        return [LabelCheck(False, "image_read_failed")]
    height, width = bgr.shape[:2]
    lines = [line for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        return [LabelCheck(False, "empty_label")]

    checks = []
    for line in lines:
        try:
            class_id, polygon = parse_obb_line(line, width, height)
        except ValueError as exc:
            checks.append(LabelCheck(False, f"bad_label:{exc}"))
            continue
        checks.append(
            check_polygon_visible(
                bgr,
                polygon,
                class_id,
                args.min_visible_contrast,
                args.min_visible_edge_density,
                args.min_polygon_area,
            )
        )
    return checks


def prepare_output(root: Path, clean: bool, dry_run: bool):
    if dry_run:
        return
    if clean and root.exists():
        shutil.rmtree(root)
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)


def write_yaml(root: Path):
    yaml_path = root / "metal_objects_obb.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {root}",
                "train: images/train",
                "val: images/val",
                "names:",
                "  0: steel_plate",
                "  1: steel_cube",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path


def main():
    args = parse_args()
    src = Path(args.dataset_dir)
    dst = Path(args.output_dir)
    prepare_output(dst, args.clean_output, args.dry_run)

    totals = {"seen": 0, "kept": 0, "rejected": 0}
    kept_by_class = {0: 0, 1: 0}
    rejected_reasons: dict[str, int] = {}

    for split in ("train", "val"):
        image_root = src / "images" / split
        label_root = src / "labels" / split
        for image_path in sorted(image_root.glob("*.png")):
            label_path = label_root / f"{image_path.stem}.txt"
            totals["seen"] += 1
            if not label_path.exists():
                totals["rejected"] += 1
                rejected_reasons["missing_label"] = rejected_reasons.get("missing_label", 0) + 1
                continue

            checks = check_sample(image_path, label_path, args)
            ok = bool(checks) and all(check.ok for check in checks)
            if ok:
                totals["kept"] += 1
                for check in checks:
                    kept_by_class[check.class_id] = kept_by_class.get(check.class_id, 0) + 1
                if not args.dry_run:
                    shutil.copy2(image_path, dst / "images" / split / image_path.name)
                    shutil.copy2(label_path, dst / "labels" / split / label_path.name)
            else:
                totals["rejected"] += 1
                reason = next((check.reason for check in checks if not check.ok), "unknown")
                rejected_reasons[reason] = rejected_reasons.get(reason, 0) + 1

    yaml_path = None if args.dry_run else write_yaml(dst)
    print("\n=== OBB dataset visibility filter ===")
    print(f"source:       {src}")
    print(f"output:       {dst}")
    print(f"dry_run:      {args.dry_run}")
    print(f"seen:         {totals['seen']}")
    print(f"kept:         {totals['kept']}")
    print(f"rejected:     {totals['rejected']}")
    print(f"steel_plate:  {kept_by_class.get(0, 0)}")
    print(f"steel_cube:   {kept_by_class.get(1, 0)}")
    print(f"reasons:      {rejected_reasons}")
    if yaml_path:
        print(f"yaml:         {yaml_path}")
    print("=====================================\n")


if __name__ == "__main__":
    main()
