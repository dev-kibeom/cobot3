from __future__ import annotations

# Train YOLO11s-OBB on the synthetic metal object dataset.
#
# Run after generating the hard-v2 dataset:
# python3 tools/training/train_yolo11s_obb.py

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=str(root / "work" / "datasets" / "metal_objects_hard_v2" / "metal_objects_hard_v2.yaml"),
    )
    parser.add_argument("--model", default=str(root / "vision" / "models" / "yolo11s_obb_metal_hard-v2_refinetune_best.pt"))
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0", help="Use 0 for CUDA, cpu for CPU, or auto to let Ultralytics choose.")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--aug-preset",
        choices=("stable", "default"),
        default="stable",
        help="stable disables heavy detector augmentation that can damage small OBB labels.",
    )
    parser.add_argument("--lr0", type=float, default=0.00015)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--freeze", type=int, default=10, help="Freeze first N layers for fine-tuning. 0 disables it.")
    parser.add_argument("--project", default=str(root / "work" / "runs" / "metal_objects_obb"))
    parser.add_argument("--name", default="yolo11s_obb_metal_hard_v2")
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument("--export-engine", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.model)
    device = None if args.device == "auto" else args.device
    train_kwargs = dict(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        project=args.project,
        name=args.name,
        task="obb",
        exist_ok=True,
        patience=args.patience,
        lr0=args.lr0,
        cos_lr=True,
    )
    if device is not None:
        train_kwargs["device"] = device
    if args.freeze > 0:
        train_kwargs["freeze"] = args.freeze

    if args.aug_preset == "stable":
        train_kwargs.update(
            optimizer="AdamW",
            mosaic=0.0,
            close_mosaic=0,
            mixup=0.0,
            cutmix=0.0,
            copy_paste=0.0,
            degrees=0.0,
            translate=0.02,
            scale=0.15,
            shear=0.0,
            perspective=0.0,
            flipud=0.0,
            fliplr=0.0,
            hsv_h=0.0,
            hsv_s=0.15,
            hsv_v=0.15,
        )

    results = model.train(**train_kwargs)

    best = Path(args.project) / args.name / "weights" / "best.pt"
    print(f"\n[train] best model: {best}\n")

    trained = YOLO(str(best)) if best.exists() else model
    val_kwargs = dict(data=args.data, imgsz=args.imgsz, project=args.project, name=f"{args.name}_val", exist_ok=True)
    if device is not None:
        val_kwargs["device"] = device
    metrics = trained.val(**val_kwargs)
    print(f"[val] mAP50-95: {metrics.box.map:.4f}")
    print(f"[val] mAP50:    {metrics.box.map50:.4f}")

    if args.export_onnx:
        trained.export(format="onnx", imgsz=args.imgsz, dynamic=True, simplify=True)
    if args.export_engine:
        trained.export(format="engine", imgsz=args.imgsz, half=True)

    return results


if __name__ == "__main__":
    main()
