from __future__ import annotations

# Lightweight Isaac Sim prototype.
# Default is headless and low resolution to avoid stressing the machine.
# Run:
# ~/.local/share/ov/pkg/isaac-sim-*/python.sh \
#   tools/isaac/aruco_alignment_prototype.py
#
# GUI debug:
# ~/.local/share/ov/pkg/isaac-sim-*/python.sh \
#   tools/isaac/aruco_alignment_prototype.py --gui

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Show Isaac Sim GUI.")
    parser.add_argument("--frames", type=int, default=25)
    parser.add_argument("--plate-yaw-deg", type=float, default=35.0)
    parser.add_argument("--target-yaw-deg", type=float, default=0.0)
    parser.add_argument(
        "--save-dir",
        default=str(Path(__file__).resolve().parents[2] / "work"),
    )
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import math
import os

import cv2
import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils


TEXTURE_DIR = Path(__file__).resolve().parent / "assets" / "aruco_marker_6x6"
OUT_DIR = Path(ARGS.save_dir)

WIDTH, HEIGHT = 640, 480
FX, FY = 500.0, 500.0
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
DIST = [0.0] * 12

CAMERA_PATH = "/World/top_camera"
PLATE_PATH = "/World/misaligned_plate"

MARKER_Z = 0.012
MARKER_SIZE = 0.10
PLATE_TOP_Z = 0.030

TARGET_CENTER = np.array([0.45, -0.22, 0.0], dtype=np.float64)
TARGET_SIZE = np.array([0.42, 0.24], dtype=np.float64)

T_CV_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])


def norm_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff_deg(target, current):
    diff = (target - current + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.degrees(diff)


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


def add_textured_marker(stage, marker_id, pos, yaw_deg):
    texture = TEXTURE_DIR / f"aruco_id{marker_id}.png"
    if not texture.exists():
        raise FileNotFoundError(texture)

    path = f"/World/aruco_{marker_id}"
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, 0.5, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    ).Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddRotateZOp().Set(float(yaw_deg))
    xf.AddScaleOp().Set(Gf.Vec3f(MARKER_SIZE, MARKER_SIZE, MARKER_SIZE))

    mat = UsdShade.Material.Define(stage, path + "_mat")
    shader = UsdShade.Shader.Define(stage, path + "_mat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    uv = UsdShade.Shader.Define(stage, path + "_mat/UV")
    uv.CreateIdAttr("UsdPrimvarReader_float2")
    uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, path + "_mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(os.fspath(texture))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)


def local_to_world(center, yaw, local_xy, z):
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = center[:2] + rot @ local_xy
    return np.array([xy[0], xy[1], z], dtype=np.float64)


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = world.stage

    light = UsdLux.DistantLight.Define(stage, "/World/key_light")
    light.CreateIntensityAttr(600.0)

    target_yaw = math.radians(ARGS.target_yaw_deg)
    marker_offsets = {
        0: np.array([-TARGET_SIZE[0] / 2, TARGET_SIZE[1] / 2]),
        1: np.array([TARGET_SIZE[0] / 2, TARGET_SIZE[1] / 2]),
        2: np.array([TARGET_SIZE[0] / 2, -TARGET_SIZE[1] / 2]),
        3: np.array([-TARGET_SIZE[0] / 2, -TARGET_SIZE[1] / 2]),
    }
    for marker_id, offset in marker_offsets.items():
        add_textured_marker(stage, marker_id, local_to_world(TARGET_CENTER, target_yaw, offset, MARKER_Z), ARGS.target_yaw_deg)

    world.scene.add(
        VisualCuboid(
            prim_path=PLATE_PATH,
            name="misaligned_plate",
            position=np.array([0.25, 0.14, PLATE_TOP_Z / 2.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, ARGS.plate_yaw_deg]), degrees=True),
            scale=np.array([0.32, 0.14, PLATE_TOP_Z]),
            color=np.array([0.95, 0.18, 0.05]),
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
    camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY, pinhole=DIST)
    return world, camera


def pixel_to_world(u, v, z):
    ray_cv = np.array([(u - CX) / FX, (v - CY) / FY, 1.0, 0.0], dtype=np.float64)
    ray_gl = T_CV_TO_GL @ ray_cv
    tf = get_tf(CAMERA_PATH)
    origin = tf[:3, 3]
    direction = tf[:3, :3] @ ray_gl[:3]
    direction /= np.linalg.norm(direction)
    t = (z - origin[2]) / direction[2]
    return origin + direction * t


def detect_marker_centers(bgr):
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return {}, []
    centers = {}
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        if marker_id in (0, 1, 2, 3):
            pts = marker_corners[0]
            centers[marker_id] = pts.mean(axis=0)
    return centers, corners


def target_frame_from_markers(centers):
    missing = [i for i in (0, 1, 2, 3) if i not in centers]
    if missing:
        raise RuntimeError(f"Missing ArUco markers: {missing}")
    world_corners = np.array([pixel_to_world(*centers[i], MARKER_Z) for i in (0, 1, 2, 3)])
    center = world_corners.mean(axis=0)
    left = 0.5 * (world_corners[0] + world_corners[3])
    right = 0.5 * (world_corners[1] + world_corners[2])
    yaw = math.atan2(float((right - left)[1]), float((right - left)[0]))
    return center, norm_angle(yaw), world_corners


def detect_plate(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([22, 255, 255]))
    mask |= cv2.inRange(hsv, np.array([170, 80, 60]), np.array([179, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("Plate was not detected.")
    box = cv2.boxPoints(cv2.minAreaRect(max(contours, key=cv2.contourArea))).astype(np.float32)
    edges = [box[(i + 1) % 4] - box[i] for i in range(4)]
    edge = edges[int(np.argmax([np.linalg.norm(e) for e in edges]))]
    center_px = box.mean(axis=0)
    angle_img = math.atan2(float(edge[1]), float(edge[0]))
    center_world = pixel_to_world(float(center_px[0]), float(center_px[1]), PLATE_TOP_Z)
    end_px = center_px + np.array([math.cos(angle_img), math.sin(angle_img)]) * 80.0
    end_world = pixel_to_world(float(end_px[0]), float(end_px[1]), PLATE_TOP_Z)
    yaw = math.atan2(float((end_world - center_world)[1]), float((end_world - center_world)[0]))
    return center_world, norm_angle(yaw), box


def draw_debug(bgr, marker_centers, target_center, target_yaw, plate_center, plate_yaw, plate_box):
    img = bgr.copy()
    for marker_id, c in marker_centers.items():
        cv2.circle(img, tuple(c.astype(int)), 5, (0, 255, 0), -1)
        cv2.putText(img, f"id{marker_id}", tuple((c + [6, -6]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.polylines(img, [plate_box.astype(np.int32)], True, (255, 0, 255), 2)
    cv2.putText(
        img,
        f"target yaw={math.degrees(target_yaw):+.1f} plate yaw={math.degrees(plate_yaw):+.1f}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        img,
        f"dx={target_center[0]-plate_center[0]:+.3f} dy={target_center[1]-plate_center[1]:+.3f} dyaw={angle_diff_deg(target_yaw, plate_yaw):+.1f}",
        (15, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2,
    )
    return img


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    world = None
    try:
        world, camera = setup_scene()
        for _ in range(max(1, ARGS.frames)):
            world.step(render=True)

        bgr = rgba_to_bgr(camera.get_rgba())
        marker_centers, _ = detect_marker_centers(bgr)
        target_center, target_yaw, _ = target_frame_from_markers(marker_centers)
        plate_center, plate_yaw, plate_box = detect_plate(bgr)

        debug = draw_debug(bgr, marker_centers, target_center, target_yaw, plate_center, plate_yaw, plate_box)
        raw_path = OUT_DIR / "aruco_alignment_raw.png"
        debug_path = OUT_DIR / "aruco_alignment_debug.png"
        cv2.imwrite(str(raw_path), bgr)
        cv2.imwrite(str(debug_path), debug)

        print("\n=== Lightweight ArUco Alignment ===")
        print(f"target: x={target_center[0]:+.3f}, y={target_center[1]:+.3f}, yaw={math.degrees(target_yaw):+.1f} deg")
        print(f"plate:  x={plate_center[0]:+.3f}, y={plate_center[1]:+.3f}, yaw={math.degrees(plate_yaw):+.1f} deg")
        print(f"move:   dx={target_center[0]-plate_center[0]:+.3f} m, dy={target_center[1]-plate_center[1]:+.3f} m, dyaw={angle_diff_deg(target_yaw, plate_yaw):+.1f} deg")
        print(f"debug:  {debug_path}")
        print("===================================\n")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
