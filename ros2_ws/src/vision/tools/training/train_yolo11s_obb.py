from __future__ import annotations

# Train YOLO11s-OBB on the synthetic metal object dataset.
#
# Run after generating the normal dataset:
# python3 tools/training/train_yolo11s_obb.py

import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=str(root / "work" / "datasets" / "metal_objects_obb" / "metal_objects_obb.yaml"),
    )
    parser.add_argument("--model", default="yolo11s-obb.pt")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=2)
    parser.add_argument("--device", default="0", help="Use 0 for CUDA, cpu for CPU, or auto to let Ultralytics choose.")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument(
        "--amp",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable mixed precision. Default false is slower but avoids CUDA launch failures on some GPUs/drivers.",
    )
    parser.add_argument(
        "--disable-cudnn",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Disable cuDNN kernels. Slower, but avoids CUDNN_STATUS_EXECUTION_FAILED on some GPU/driver setups.",
    )
    parser.add_argument(
        "--aug-preset",
        choices=("stable", "balanced", "default"),
        default="stable",
        help=(
            "stable disables heavy detector augmentation that can damage small OBB labels. "
            "balanced enables moderate mosaic/flip/HSV/rotation suitable for medium-large OBBs "
            "(e.g. top-down cube/panel). default uses Ultralytics built-in OBB augmentation."
        ),
    )
    parser.add_argument("--lr0", type=float, default=0.001)
    parser.add_argument("--patience", type=int, default=0, help="0 disables early stopping and runs all epochs.")
    parser.add_argument("--freeze", type=int, default=0, help="Freeze first N layers for fine-tuning. 0 disables it.")
    parser.add_argument("--multi-scale", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--scale",
        type=float,
        default=0.10,
        help="Training scale augmentation. Larger values improve distance/projected-size robustness.",
    )
    parser.add_argument("--translate", type=float, default=0.02)
    parser.add_argument("--project", default=str(root / "work" / "runs" / "metal_objects_obb"))
    parser.add_argument("--name", default="yolo11s_obb_metal_wrist_finetune")
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument("--export-engine", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.disable_cudnn:
        torch.backends.cudnn.enabled = False
        torch.backends.cudnn.benchmark = False
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
        multi_scale=args.multi_scale,
        amp=args.amp,
        deterministic=False,
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
            translate=args.translate,
            scale=args.scale,
            shear=0.0,
            perspective=0.0,
            flipud=0.0,
            fliplr=0.0,
            hsv_h=0.0,
            hsv_s=0.15,
            hsv_v=0.15,
            auto_augment=None,
            erasing=0.0,
            bgr=0.0,
        )
    elif args.aug_preset == "balanced":
        # Moderate augmentation tuned for medium-to-large OBBs (top-down cube/panel).
        # Mosaic is on early then turned off via close_mosaic for clean final epochs.
        train_kwargs.update(
            optimizer="AdamW",
            mosaic=0.5,
            close_mosaic=10,
            mixup=0.0,
            cutmix=0.0,
            copy_paste=0.1,
            degrees=10.0,
            translate=max(args.translate, 0.05),
            scale=max(args.scale, 0.25),
            shear=2.0,
            perspective=0.0,
            flipud=0.5,
            fliplr=0.5,
            hsv_h=0.015,
            hsv_s=0.4,
            hsv_v=0.4,
            auto_augment=None,
            erasing=0.2,
            bgr=0.0,
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
