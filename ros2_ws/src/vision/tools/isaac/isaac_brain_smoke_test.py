from __future__ import annotations

# Isaac-only smoke test for alignment_brain.py.
# This does not run YOLO or ROS2. It builds a tiny top-camera scene,
# detects the synthetic plate by color, wraps that result as an OBB,
# then asks alignment_brain to create a pick/place goal.
#
# Run:
# ~/.local/share/ov/pkg/isaac-sim-*/python.sh \
#   tools/isaac/isaac_brain_smoke_test.py

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--save-dir", default=str(Path(__file__).resolve().parents[2] / "work"))
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import math
import sys

import cv2
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdLux

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.append(str(PACKAGE_ROOT))

from vision.alignment_brain import BrainConfig, CameraModel, TargetFrame, make_pick_place_goal
from vision.vision_contracts import ObbDetection, Pose2D, encode_goal


WIDTH, HEIGHT = 640, 480
FX, FY = 500.0, 500.0
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
CAMERA_PATH = "/World/top_camera"
PLATE_Z = 0.03


def get_tf(path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64).T


def rgba_to_bgr(rgba):
    img = np.asarray(rgba)
    if img.dtype != np.uint8:
        if float(np.nanmax(img)) <= 1.0:
            img = img * 255.0
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    light = UsdLux.DistantLight.Define(world.stage, "/World/key_light")
    light.CreateIntensityAttr(600.0)

    world.scene.add(
        VisualCuboid(
            prim_path="/World/test_plate",
            name="test_plate",
            position=np.array([0.25, 0.14, PLATE_Z / 2.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 35.0]), degrees=True),
            scale=np.array([0.32, 0.14, PLATE_Z]),
            color=np.array([0.95, 0.18, 0.05]),
        )
    )
    world.scene.add(
        VisualCuboid(
            prim_path="/World/target_marker",
            name="target_marker",
            position=np.array([0.45, -0.22, 0.002]),
            scale=np.array([0.36, 0.18, 0.004]),
            color=np.array([0.0, 0.9, 0.25]),
        )
    )

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.40, -0.02, 1.35]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 90.0, 180.0]), degrees=True),
        frequency=10,
        resolution=(WIDTH, HEIGHT),
    )
    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12)
    return world, camera


def color_plate_to_obb(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([22, 255, 255]))
    mask |= cv2.inRange(hsv, np.array([170, 80, 60]), np.array([179, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("Plate was not detected.")

    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
    box = cv2.boxPoints(rect).astype(np.float32)
    center = box.mean(axis=0)
    width, height = rect[1]

    edges = [box[(idx + 1) % 4] - box[idx] for idx in range(4)]
    edge = edges[int(np.argmax([np.linalg.norm(e) for e in edges]))]
    angle = math.atan2(float(edge[1]), float(edge[0]))

    return ObbDetection(
        valid=True,
        class_id=0,
        confidence=1.0,
        center_px=(float(center[0]), float(center[1])),
        corners_px=tuple((float(x), float(y)) for x, y in box),
        width_px=float(width),
        height_px=float(height),
        angle_rad=float(angle),
    )


def draw_debug(bgr, obb, goal):
    img = bgr.copy()
    cv2.polylines(img, [np.array(obb.corners_px, dtype=np.int32)], True, (255, 0, 255), 2)
    cv2.putText(
        img,
        f"pick=({goal.pick.x:+.3f},{goal.pick.y:+.3f},{math.degrees(goal.pick.yaw):+.1f})",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        img,
        f"place=({goal.place.x:+.3f},{goal.place.y:+.3f},{math.degrees(goal.place.yaw):+.1f})",
        (15, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        2,
    )
    return img


def main():
    out_dir = Path(ARGS.save_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    world = None
    try:
        world, camera = setup_scene()
        for _ in range(max(1, ARGS.frames)):
            world.step(render=True)

        bgr = rgba_to_bgr(camera.get_rgba())
        obb = color_plate_to_obb(bgr)

        camera_model = CameraModel(
            fx=FX,
            fy=FY,
            cx=CX,
            cy=CY,
            t_world_camera=get_tf(CAMERA_PATH),
        )
        target = TargetFrame(center=Pose2D(0.45, -0.22, PLATE_Z, 0.0))
        goal = make_pick_place_goal(
            "cell_01",
            obb,
            target,
            camera_model,
            BrainConfig(plate_top_z=PLATE_Z, pick_z=PLATE_Z, place_z=PLATE_Z),
        )

        cv2.imwrite(str(out_dir / "brain_smoke_debug.png"), draw_debug(bgr, obb, goal))

        print("\n=== Isaac Brain Smoke Test ===")
        print(f"valid:  {goal.valid}")
        print(f"robot:  {goal.robot_index}")
        print(f"pick:   x={goal.pick.x:+.3f}, y={goal.pick.y:+.3f}, z={goal.pick.z:+.3f}, yaw={math.degrees(goal.pick.yaw):+.1f} deg")
        print(f"place:  x={goal.place.x:+.3f}, y={goal.place.y:+.3f}, z={goal.place.z:+.3f}, yaw={math.degrees(goal.place.yaw):+.1f} deg")
        print(f"move:   dx={goal.dx:+.3f}, dy={goal.dy:+.3f}, dyaw={math.degrees(goal.dyaw):+.1f} deg")
        print(f"topic payload: {encode_goal(goal)}")
        print(f"debug:  {out_dir / 'brain_smoke_debug.png'}")
        print("==============================\n")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
