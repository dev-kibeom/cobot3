from __future__ import annotations

# Isaac Sim top-camera calibration sanity check for the M0609 vision flow.
#
# This script does not use YOLO. Calibration should first verify camera
# intrinsics/extrinsics by projecting known world points to pixels and
# back-projecting those pixels onto the table plane.
#
# Run with Isaac Sim Python:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/isaac_m0609_calibration_check.py

import argparse
from pathlib import Path
import sys

from isaacsim import SimulationApp


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--camera-path", default="/World/top_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=500.0)
    parser.add_argument("--fy", type=float, default=500.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--plane-z", type=float, default=0.03)
    parser.add_argument("--camera-z", type=float, default=1.35)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--out", default=str(root / "work" / "calibration_check.png"))
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import cv2
import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, UsdLux

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

sys.path.append(str(Path(__file__).resolve().parents[2]))
from vision.alignment_brain import CameraModel, T_CV_TO_GL, pixel_to_world_on_plane  # noqa: E402


CALIBRATION_POINTS = [
    (0.20, -0.20, "p0"),
    (0.50, -0.20, "p1"),
    (0.50, 0.20, "p2"),
    (0.20, 0.20, "p3"),
    (0.35, 0.00, "center"),
]


def get_tf(path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {path}")
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64).T


def project_world_to_pixel(point_world, t_world_camera):
    p_world = np.array([point_world[0], point_world[1], point_world[2], 1.0], dtype=np.float64)
    p_gl = np.linalg.inv(t_world_camera) @ p_world
    p_cv = T_CV_TO_GL @ p_gl
    if p_cv[2] <= 1e-9:
        return None
    u = ARGS.fx * (p_cv[0] / p_cv[2]) + ARGS.cx
    v = ARGS.fy * (p_cv[1] / p_cv[2]) + ARGS.cy
    return np.array([u, v], dtype=np.float64)


def rgba_to_bgr(rgba):
    img = np.asarray(rgba)
    if img.size == 0:
        raise RuntimeError("Camera returned an empty RGBA buffer.")
    if img.dtype != np.uint8:
        if float(np.nanmax(img)) <= 1.0:
            img = img * 255.0
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage

    light = UsdLux.DistantLight.Define(stage, "/World/calibration_light")
    light.CreateIntensityAttr(900.0)
    light_xform = UsdGeom.Xformable(light.GetPrim())
    light_xform.ClearXformOpOrder()
    light_xform.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 20.0))

    world.scene.add(
        VisualCuboid(
            prim_path="/World/calibration_table",
            name="calibration_table",
            position=np.array([0.35, 0.0, ARGS.plane_z - 0.004]),
            scale=np.array([0.75, 0.55, 0.008]),
            color=np.array([0.35, 0.55, 0.62]),
        )
    )

    for x, y, name in CALIBRATION_POINTS:
        world.scene.add(
            VisualCuboid(
                prim_path=f"/World/calibration_{name}",
                name=f"calibration_{name}",
                position=np.array([x, y, ARGS.plane_z + 0.003]),
                scale=np.array([0.025, 0.025, 0.006]),
                color=np.array([1.0, 0.1, 0.05]),
            )
        )

    camera = Camera(
        prim_path=ARGS.camera_path,
        position=np.array([0.35, 0.0, ARGS.camera_z]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 90.0, 180.0]), degrees=True),
        frequency=10,
        resolution=(ARGS.width, ARGS.height),
    )
    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(cx=ARGS.cx, cy=ARGS.cy, fx=ARGS.fx, fy=ARGS.fy, pinhole=[0.0] * 12)
    return world, camera


def main():
    world = None
    try:
        world, camera = setup_scene()
        for _ in range(max(1, ARGS.warmup_frames)):
            world.step(render=True)

        t_world_camera = get_tf(ARGS.camera_path)
        camera_model = CameraModel(ARGS.fx, ARGS.fy, ARGS.cx, ARGS.cy, t_world_camera)
        image = rgba_to_bgr(camera.get_rgba())

        print("\n=== Isaac top-camera calibration check ===")
        print(f"camera_path: {ARGS.camera_path}")
        print(f"image_size:  {ARGS.width}x{ARGS.height}")
        print(f"intrinsic:   fx={ARGS.fx:.3f}, fy={ARGS.fy:.3f}, cx={ARGS.cx:.3f}, cy={ARGS.cy:.3f}")
        print(f"plane_z:     {ARGS.plane_z:.4f} m")
        print("t_world_camera:")
        print(np.array2string(t_world_camera, precision=5, suppress_small=True))
        print("\npoint      pixel(u,v)          back_project(x,y,z)       xy_error_mm")

        errors_mm = []
        for x, y, name in CALIBRATION_POINTS:
            world_point = np.array([x, y, ARGS.plane_z], dtype=np.float64)
            pixel = project_world_to_pixel(world_point, t_world_camera)
            if pixel is None:
                print(f"{name:<9} behind camera")
                continue
            back = pixel_to_world_on_plane(camera_model, float(pixel[0]), float(pixel[1]), ARGS.plane_z)
            err_mm = float(np.linalg.norm(back[:2] - world_point[:2]) * 1000.0)
            errors_mm.append(err_mm)
            cv2.circle(image, tuple(np.round(pixel).astype(int)), 5, (0, 255, 255), -1)
            cv2.putText(image, name, tuple(np.round(pixel + np.array([8.0, -8.0])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            print(
                f"{name:<9} ({pixel[0]:8.2f},{pixel[1]:8.2f}) "
                f"({back[0]:+.4f},{back[1]:+.4f},{back[2]:+.4f}) "
                f"{err_mm:9.3f}"
            )

        if errors_mm:
            print(f"\nmax_xy_error_mm: {max(errors_mm):.3f}")
            print(f"mean_xy_error_mm: {float(np.mean(errors_mm)):.3f}")

        out_path = Path(ARGS.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_path), image)
        print(f"debug_image: {out_path}")
        print("=========================================\n")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
