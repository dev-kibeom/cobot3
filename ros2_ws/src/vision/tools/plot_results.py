#!/usr/bin/env python3
"""Plot training results from Ultralytics `results.csv`.

Usage:
  python3 tools/plot_results.py work/runs/metal_objects_obb/<run_name>/results.csv
  python3 tools/plot_results.py <results.csv> --out <out_dir>
"""
from pathlib import Path
import argparse
import pandas as pd
import matplotlib.pyplot as plt


def plot_results(csv_path: Path, out_dir: Path):
    df = pd.read_csv(csv_path)
    epochs = df['epoch'].values

    out_dir.mkdir(parents=True, exist_ok=True)

    # Loss curves
    plt.figure(figsize=(10, 5))
    for col in ['train/box_loss', 'train/cls_loss', 'train/dfl_loss', 'train/angle_loss']:
        if col in df.columns:
            plt.plot(epochs, df[col], label=col)
    for col in ['val/box_loss', 'val/cls_loss', 'val/dfl_loss', 'val/angle_loss']:
        if col in df.columns:
            plt.plot(epochs, df[col], '--', label=col)
    plt.xlabel('epoch')
    plt.ylabel('loss')
    plt.title('Loss curves')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / 'loss_curves.png')
    plt.close()

    # mAP / PR metrics
    metrics = [
        ('metrics/mAP50(B)', 'mAP50'),
        ('metrics/mAP50-95(B)', 'mAP50-95'),
        ('metrics/precision(B)', 'precision'),
        ('metrics/recall(B)', 'recall'),
    ]
    for col, name in metrics:
        if col in df.columns:
            plt.figure(figsize=(6, 4))
            plt.plot(epochs, df[col], marker='o')
            plt.xlabel('epoch')
            plt.ylabel(name)
            plt.title(name + ' per epoch')
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(out_dir / f'{name}.png')
            plt.close()

    # combined mAP curves
    plt.figure(figsize=(8, 4))
    if 'metrics/mAP50(B)' in df.columns:
        plt.plot(epochs, df['metrics/mAP50(B)'], label='mAP50')
    if 'metrics/mAP50-95(B)' in df.columns:
        plt.plot(epochs, df['metrics/mAP50-95(B)'], label='mAP50-95')
    plt.xlabel('epoch')
    plt.ylabel('mAP')
    plt.title('mAP curves')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / 'map_curves.png')
    plt.close()

    print('Saved plots to', out_dir)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('results_csv', help='Path to results.csv produced by training')
    p.add_argument('--out', help='Output directory for plots (default: results.csv parent/plots)')
    args = p.parse_args()

    csv_path = Path(args.results_csv)
    if not csv_path.exists():
        raise SystemExit('results.csv not found: ' + str(csv_path))
    out_dir = Path(args.out) if args.out else csv_path.parent / 'plots'
    plot_results(csv_path, out_dir)


if __name__ == '__main__':
    main()
