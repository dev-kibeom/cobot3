from __future__ import annotations

# Convert randomized evidence samples into a YOLO OBB fine-tuning dataset.
#
# Input:
#   work/confusion_matrix_hard_v2/samples_evaluated.csv
#
# Output:
#   work/datasets/metal_objects_hard_v2/
#     images/train/*.png
#     images/val/*.png
#     labels/train/*.txt
#     labels/val/*.txt
#     metal_objects_finetune.yaml

import argparse
import csv
from pathlib import Path
import random
import shutil

import cv2


CLASS_NAMES = {0: "steel_plate", 1: "steel_cube"}


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--samples-csv",
        default=str(root / "work" / "confusion_matrix_hard_v2" / "samples_evaluated.csv"),
        help="Use samples_evaluated.csv when available so failure oversampling can use correct/pred_class.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(root / "work" / "datasets" / "metal_objects_hard_v2"),
    )
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--clean", action="store_true")
    parser.add_argument(
        "--cube-repeat",
        type=int,
        default=2,
        help="Repeat steel_cube training samples this many times. Validation samples are never repeated.",
    )
    parser.add_argument(
        "--failure-repeat",
        type=int,
        default=2,
        help="Repeat failed training samples this many times. Validation samples are never repeated.",
    )
    return parser.parse_args()


def read_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def resolve_path(value: str, base_dir: Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else base_dir / path


def clean_output(root: Path):
    if root.exists():
        shutil.rmtree(root)


def prepare_dirs(root: Path):
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)


def gt_points(row):
    if row["true_class"] == "background":
        return []
    return [
        (float(row["gt_x0"]), float(row["gt_y0"])),
        (float(row["gt_x1"]), float(row["gt_y1"])),
        (float(row["gt_x2"]), float(row["gt_y2"])),
        (float(row["gt_x3"]), float(row["gt_y3"])),
    ]


def yolo_obb_label(row, width: int, height: int) -> str:
    if row["true_class"] == "background":
        return ""
    class_id = int(float(row["true_class_id"]))
    values = [str(class_id)]
    for x, y in gt_points(row):
        nx = min(1.0, max(0.0, x / width))
        ny = min(1.0, max(0.0, y / height))
        values.extend([f"{nx:.6f}", f"{ny:.6f}"])
    return " ".join(values)


def stratified_split(rows, val_ratio: float, rng: random.Random):
    by_class: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_class.setdefault(row["true_class"], []).append(row)

    train = []
    val = []
    for class_rows in by_class.values():
        rng.shuffle(class_rows)
        val_count = int(round(len(class_rows) * val_ratio))
        val.extend(class_rows[:val_count])
        train.extend(class_rows[val_count:])
    rng.shuffle(train)
    rng.shuffle(val)
    return train, val


def is_failed(row) -> bool:
    value = row.get("correct", "")
    if value == "":
        return False
    return value in ("0", "False", "false")


def repeat_count(row, split: str, args) -> int:
    if split != "train":
        return 1
    repeats = 1
    if row["true_class"] == "steel_cube":
        repeats = max(repeats, args.cube_repeat)
    if is_failed(row):
        repeats = max(repeats, args.failure_repeat)
    return max(1, repeats)


def write_sample(row, source_image: Path, root: Path, split: str, sample_index: int, repeat_index: int):
    bgr = cv2.imread(str(source_image))
    if bgr is None:
        raise RuntimeError(f"Failed to read image: {source_image}")
    height, width = bgr.shape[:2]

    class_name = row["true_class"]
    stem = f"{class_name}_{sample_index:05d}"
    if repeat_index:
        stem += f"_r{repeat_index}"

    image_path = root / "images" / split / f"{stem}.png"
    label_path = root / "labels" / split / f"{stem}.txt"
    shutil.copy2(source_image, image_path)
    label = yolo_obb_label(row, width, height)
    label_path.write_text((label + "\n") if label else "", encoding="utf-8")
    return image_path, label_path


def write_yaml(root: Path):
    yaml_path = root / "metal_objects_hard_v2.yaml"
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


def write_summary(root: Path, yaml_path: Path, counts: dict[str, dict[str, int]], args):
    lines = [
        "# YOLO OBB Fine-Tuning Dataset",
        "",
        f"- source: `{args.samples_csv}`",
        f"- yaml: `{yaml_path}`",
        f"- val_ratio: `{args.val_ratio}`",
        f"- cube_repeat: `{args.cube_repeat}`",
        f"- failure_repeat: `{args.failure_repeat}`",
        "",
        "| split | steel_plate | steel_cube | background | total |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for split in ("train", "val"):
        plate = counts[split].get("steel_plate", 0)
        cube = counts[split].get("steel_cube", 0)
        background = counts[split].get("background", 0)
        lines.append(f"| `{split}` | {plate} | {cube} | {background} | {plate + cube + background} |")
    path = root / "README.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main():
    args = parse_args()
    source_csv = Path(args.samples_csv).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    rows = read_rows(source_csv)
    if not rows:
        raise RuntimeError(f"No rows found: {source_csv}")
    if args.clean:
        clean_output(output_dir)
    prepare_dirs(output_dir)

    rng = random.Random(args.seed)
    train_rows, val_rows = stratified_split(rows, args.val_ratio, rng)
    split_rows = {"train": train_rows, "val": val_rows}
    counts = {"train": {}, "val": {}}
    written = 0

    for split, current_rows in split_rows.items():
        for row in current_rows:
            source_image = resolve_path(row["raw_image"], source_csv.parent)
            repeats = repeat_count(row, split, args)
            for repeat_index in range(repeats):
                write_sample(row, source_image, output_dir, split, written, repeat_index)
                counts[split][row["true_class"]] = counts[split].get(row["true_class"], 0) + 1
                written += 1

    yaml_path = write_yaml(output_dir)
    summary_path = write_summary(output_dir, yaml_path, counts, args)

    print("\n=== YOLO OBB fine-tuning dataset ready ===")
    print(f"source:       {source_csv}")
    print(f"output:       {output_dir}")
    print(f"yaml:         {yaml_path}")
    print(f"summary:      {summary_path}")
    print(f"train_plate:  {counts['train'].get('steel_plate', 0)}")
    print(f"train_cube:   {counts['train'].get('steel_cube', 0)}")
    print(f"val_plate:    {counts['val'].get('steel_plate', 0)}")
    print(f"val_cube:     {counts['val'].get('steel_cube', 0)}")
    print("==========================================\n")


if __name__ == "__main__":
    main()
