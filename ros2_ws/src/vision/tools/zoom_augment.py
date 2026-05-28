#!/usr/bin/env python3
"""Create zoom-in augmented images for YOLO OBB dataset.

Usage example:
  python3 tools/zoom_augment.py \
    --src work/datasets/metal_objects_normal_usd \
    --dst work/datasets/metal_objects_normal_usd_zoom \
    --num 300 --seed 42
"""
from pathlib import Path
import argparse
import random
import shutil
import cv2
import os


def load_labels(label_path):
    with open(label_path, 'r') as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]
    objs = []
    for l in lines:
        toks = l.split()
        cls = int(float(toks[0]))
        coords = list(map(float, toks[1:1+8]))
        objs.append((cls, coords))
    return objs


def save_labels(label_path, objs):
    with open(label_path, 'w') as f:
        for cls, coords in objs:
            coords_s = ' '.join(f"{x:.6f}" for x in coords)
            f.write(f"{cls} {coords_s}\n")


def augment_image(img_path, label_path, out_img_path, out_label_path, num_aug=1, seed=None, name_suffix=None):
    img = cv2.imread(str(img_path))
    if img is None:
        print('failed to read', img_path)
        return 0
    H, W = img.shape[:2]
    objs = load_labels(label_path)
    if not objs:
        return 0
    created = 0
    rng = random.Random(seed)
    for i in range(num_aug):
        # Unique tag per call so multiple zooms on the same source image don't collide.
        # name_suffix is set by the caller; falls back to a random hex if missing.
        tag = name_suffix if name_suffix is not None else f"{rng.randrange(2**31):08x}"
        # pick random object to zoom on
        cls, coords = rng.choice(objs)
        xs = coords[0::2]
        ys = coords[1::2]
        minx = min(xs); maxx = max(xs); miny = min(ys); maxy = max(ys)
        # convert to pixels
        bx0 = int(minx * W); by0 = int(miny * H); bx1 = int(maxx * W); by1 = int(maxy * H)
        bw = max(1, bx1 - bx0); bh = max(1, by1 - by0)

        # choose crop window slightly larger than bbox to keep context
        pad_frac = rng.uniform(0.2, 0.6)  # smaller pad -> tighter crop -> larger apparent object
        cx = (bx0 + bx1) // 2
        cy = (by0 + by1) // 2
        crop_w = int(min(W, bw * (1.0 + pad_frac)))
        crop_h = int(min(H, bh * (1.0 + pad_frac)))

        # jitter center a little
        jitter_x = int(rng.uniform(-0.05, 0.05) * W)
        jitter_y = int(rng.uniform(-0.05, 0.05) * H)
        cx = min(max(cx + jitter_x, 0), W)
        cy = min(max(cy + jitter_y, 0), H)

        x0 = max(0, cx - crop_w // 2)
        y0 = max(0, cy - crop_h // 2)
        x1 = min(W, x0 + crop_w)
        y1 = min(H, y0 + crop_h)

        # fix in case crop hits border
        crop_w = x1 - x0; crop_h = y1 - y0
        if crop_w <= 0 or crop_h <= 0:
            continue

        crop = img[y0:y1, x0:x1]
        resized = cv2.resize(crop, (W, H), interpolation=cv2.INTER_LINEAR)

        # transform all labels
        new_objs = []
        for cls2, coords2 in objs:
            px = []
            py = []
            for xi, yi in zip(coords2[0::2], coords2[1::2]):
                xpix = xi * W
                ypix = yi * H
                xnew = (xpix - x0) * (W / crop_w)
                ynew = (ypix - y0) * (H / crop_h)
                # clamp
                xnew = min(max(xnew, 0.0), W - 1.0)
                ynew = min(max(ynew, 0.0), H - 1.0)
                px.append(xnew / W)
                py.append(ynew / H)
            coords_new = []
            for a, b in zip(px, py):
                coords_new.append(a)
                coords_new.append(b)
            new_objs.append((cls2, coords_new))

        # save
        out_img_path.parent.mkdir(parents=True, exist_ok=True)
        out_label_path.parent.mkdir(parents=True, exist_ok=True)
        base, ext = os.path.splitext(out_img_path.name)
        new_img_name = f"{base}_aug{tag}{ext}"
        new_label_name = f"{base}_aug{tag}.txt"
        new_img_path = out_img_path.parent / new_img_name
        new_label_path = out_label_path.parent / new_label_name
        cv2.imwrite(str(new_img_path), resized)
        save_labels(new_label_path, new_objs)
        created += 1
    return created


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--dst', required=True)
    ap.add_argument('--num', type=int, default=300, help='total augmented images to create')
    ap.add_argument('--per', type=int, default=1, help='attempts per source image')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    if not src.exists():
        raise SystemExit('src does not exist: ' + str(src))

    src_images_train = src / 'images' / 'train'
    src_labels_train = src / 'labels' / 'train'
    src_images_val = src / 'images' / 'val'
    src_labels_val = src / 'labels' / 'val'

    dst_images_train = dst / 'images' / 'train'
    dst_labels_train = dst / 'labels' / 'train'
    dst_images_val = dst / 'images' / 'val'
    dst_labels_val = dst / 'labels' / 'val'

    # copy original train images/labels as base
    if dst.exists():
        print('Removing existing dst', dst)
        shutil.rmtree(dst)
    dst.mkdir(parents=True, exist_ok=True)

    # Copy original train set first
    print('Copying original train/val to dst')
    shutil.copytree(src_images_train, dst_images_train)
    shutil.copytree(src_labels_train, dst_labels_train)
    shutil.copytree(src_images_val, dst_images_val)
    shutil.copytree(src_labels_val, dst_labels_val)

    # list source label files
    label_files = sorted(src_labels_train.glob('*.txt'))
    if not label_files:
        print('No source label files found in', src_labels_train)
        return

    total_to_create = args.num
    created = 0
    rng = random.Random(args.seed)
    print(f'Will create {total_to_create} augmented images')
    idxs = list(range(len(label_files)))

    while created < total_to_create:
        rng.shuffle(idxs)
        for idx in idxs:
            if created >= total_to_create:
                break
            label_path = label_files[idx]
            img_name = label_path.stem + '.png'
            img_path = src_images_train / img_name
            out_img_path = dst_images_train / img_name
            out_label_path = dst_labels_train / (label_path.stem + '.txt')
            # attempt per image
            c = augment_image(
                img_path, label_path, out_img_path, out_label_path,
                num_aug=args.per, seed=rng.randint(0, 2**31-1),
                name_suffix=f"{created:04d}",
            )
            created += c
            if created % 50 == 0:
                print('created', created)
            if created >= total_to_create:
                break

    print('Finished augmentation, created', created, 'images in', dst_images_train)


if __name__ == '__main__':
    main()
