from __future__ import annotations

# Generate a YOLO OBB dataset from Isaac Sim.
#
# This version intentionally keeps the generator small:
# - Isaac Sim still renders every image.
# - Labels come only from a second mask render of the same scene.
# - The default camera is an oblique wrist-D455 style view, not top-down.
# - Object-to-camera distance is clamped to <= 1.0 m.
#
# Example:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/generate_steel_plate_obb_dataset_fixed.py \
#   --count 1000 --guide-count 10 --clean

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import traceback

from isaacsim import SimulationApp


def parse_args():
    root = Path(__file__).resolve().parents[2]
    datasets_root = root / "work" / "datasets"
    assets_dir = Path(__file__).resolve().parent / "assets"

    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--guide-count", type=int, default=10)
    parser.add_argument("--val-ratio", type=float, default=0.20)
    parser.add_argument("--cube-ratio", type=float, default=0.45)
    parser.add_argument(
        "--background-ratio",
        type=float,
        default=0.0,
        help="Empty-label hard-negative samples. These help reduce false positives.",
    )
    parser.add_argument(
        "--both-ratio",
        type=float,
        default=0.0,
        help="Probability that one image contains both steel_plate and steel_cube.",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--dataset-name", default="metal_objects_obb")
    parser.add_argument(
        "--dataset-dir",
        default="",
        help="Dataset output directory. Relative values are placed under work/datasets.",
    )

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=500.0)
    parser.add_argument("--fy", type=float, default=500.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--frames-per-sample", type=int, default=4)
    parser.add_argument("--mask-render-frames", type=int, default=12)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--camera-retry-frames", type=int, default=60)
    parser.add_argument("--max-attempts-multiplier", type=int, default=100)
    parser.add_argument(
        "--max-new-samples",
        type=int,
        default=0,
        help="Accepted for old commands, but ignored. This script runs until --count is reached.",
    )
    parser.add_argument(
        "--camera-mode",
        choices=("wrist-d455", "lookat"),
        default="wrist-d455",
        help="Default is the robot wrist-camera style view. lookat is only for wider debugging.",
    )
    parser.add_argument("--camera-distance-min", type=float, default=0.10)
    parser.add_argument("--camera-distance-max", type=float, default=1.00)
    parser.add_argument("--wrist-camera-distance-min", type=float, default=0.35)
    parser.add_argument("--wrist-camera-distance-max", type=float, default=0.85)
    parser.add_argument("--close-sample-ratio", type=float, default=0.35)
    parser.add_argument("--wrist-close-distance-min", type=float, default=0.35)
    parser.add_argument("--wrist-close-distance-max", type=float, default=0.50)
    parser.add_argument("--wrist-elevation-min-deg", type=float, default=30.0)
    parser.add_argument("--wrist-elevation-max-deg", type=float, default=58.0)
    parser.add_argument("--wrist-azimuth-min-deg", type=float, default=145.0)
    parser.add_argument("--wrist-azimuth-max-deg", type=float, default=215.0)
    parser.add_argument("--wrist-target-jitter-xy", type=float, default=0.035)
    parser.add_argument("--lookat-elevation-min-deg", type=float, default=35.0)
    parser.add_argument("--lookat-elevation-max-deg", type=float, default=70.0)
    parser.add_argument("--lookat-azimuth-min-deg", type=float, default=-180.0)
    parser.add_argument("--lookat-azimuth-max-deg", type=float, default=180.0)

    parser.add_argument("--axis-aligned-ratio", type=float, default=0.50)
    parser.add_argument("--axis-yaw-jitter-deg", type=float, default=6.0)
    parser.add_argument("--cube-yaw-random", action="store_true")
    parser.add_argument("--fixed-cube-size", type=float, default=0.05)
    parser.add_argument(
        "--distractor-ratio",
        type=float,
        default=1.0,
        help="Probability of adding unlabeled distractors to background samples.",
    )
    parser.add_argument(
        "--object-distractor-ratio",
        type=float,
        default=0.0,
        help="Optional distractors in labeled object images. Default off to avoid occlusion-label mismatch.",
    )
    parser.add_argument("--max-distractors", type=int, default=3)

    parser.add_argument("--min-label-side-px", type=float, default=2.0)
    parser.add_argument("--min-label-area-px", type=float, default=10.0)
    parser.add_argument("--no-clip-labels", dest="clip_labels", action="store_false")
    parser.add_argument(
        "--label-source",
        choices=("mask-render", "geometry"),
        default="mask-render",
        help="Kept for command compatibility. This generator always uses mask-render labels.",
    )
    parser.add_argument("--debug-first-frame", default="")
    parser.add_argument("--debug-reject-dir", default="")
    parser.add_argument("--max-debug-rejects", type=int, default=20)

    parser.add_argument("--cube-usd", default=str(assets_dir / "iron_cube.usd"))
    parser.add_argument("--plate-usd", default=str(assets_dir / "iron_panel.usd"))
    parser.add_argument("--use-cube-usd", action="store_true")
    parser.add_argument("--use-plate-usd", action="store_true")
    parser.add_argument("--no-cube-usd", action="store_true")
    parser.add_argument("--no-plate-usd", action="store_true")
    parser.add_argument("--cube-usd-scale", type=float, default=1.0)
    parser.add_argument("--plate-usd-scale", type=float, default=1.0)
    parser.add_argument(
        "--auto-scale-cube-usd",
        action="store_true",
        help="Scale cube asset to --fixed-cube-size. Default keeps the USD asset size.",
    )
    parser.add_argument(
        "--auto-scale-plate-usd",
        action="store_true",
        help="Scale panel asset to the built-in 0.30 x 0.15 x 0.03 m plate size. Default keeps the USD asset size.",
    )
    parser.add_argument("--no-auto-scale-cube-usd", action="store_true")
    parser.add_argument("--no-auto-scale-plate-usd", action="store_true")
    parser.add_argument("--phys-iron-usd", default=str(assets_dir / "phys_iron.usd"))

    parser.set_defaults(clip_labels=True)
    args = parser.parse_args()

    if args.dataset_dir:
        dataset_dir = Path(args.dataset_dir).expanduser()
        if not dataset_dir.is_absolute():
            dataset_dir = datasets_root / dataset_dir
    else:
        dataset_dir = datasets_root / args.dataset_name
    args.dataset_dir = str(dataset_dir)
    args.label_source = "mask-render"
    return args


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import cv2
import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.rotations as gf_rot_utils
import isaacsim.core.utils.numpy.rotations as rot_utils


WIDTH = int(ARGS.width)
HEIGHT = int(ARGS.height)
FX = float(ARGS.fx)
FY = float(ARGS.fy)
CX = float(ARGS.cx)
CY = float(ARGS.cy)
MAX_CAMERA_OBJECT_DISTANCE_M = 1.0

CLASS_NAMES = {0: "steel_plate", 1: "steel_cube"}
CLASS_IDS = {"steel_plate": 0, "steel_cube": 1}

CAMERA_PATH = "/World/wrist_d455_camera"
PLATE_PATH = "/World/steel_plate"
CUBE_PATH = "/World/steel_cube"
PLATE_ASSET_PATH = "/World/steel_plate/asset"
CUBE_ASSET_PATH = "/World/steel_cube/asset"
PLATE_MASK_PATH = "/World/mask_steel_plate"
CUBE_MASK_PATH = "/World/mask_steel_cube"
TABLE_PATH = "/World/work_table"
KEY_LIGHT_PATH = "/World/key_light"
FILL_LIGHT_PATH = "/World/fill_light"
DISTRACTOR_PATHS = [f"/World/distractor_{index}" for index in range(4)]

PLATE_SIZE = np.array([0.30, 0.15, 0.03], dtype=np.float64)
CUBE_SIZE = np.full(3, float(ARGS.fixed_cube_size), dtype=np.float64)
OBJECT_SIZES = {"steel_plate": PLATE_SIZE, "steel_cube": CUBE_SIZE}
OBJECT_PATHS = {"steel_plate": PLATE_PATH, "steel_cube": CUBE_PATH}
MASK_OBJECT_PATHS = {"steel_plate": PLATE_MASK_PATH, "steel_cube": CUBE_MASK_PATH}

NORMAL_METAL_COLOR = np.array([0.78, 0.78, 0.76], dtype=np.float64)
TABLE_COLOR = np.array([0.54, 0.64, 0.64], dtype=np.float64)
MASK_TABLE_COLOR = np.array([0.015, 0.015, 0.015], dtype=np.float64)
MASK_CLASS_COLORS_RGB = {
    0: np.array([0.02, 0.95, 0.10], dtype=np.float64),
    1: np.array([1.00, 0.48, 0.02], dtype=np.float64),
}
GUIDE_COLORS = {0: (80, 220, 90), 1: (255, 185, 65)}
DISTRACTOR_COLORS = [
    np.array([0.10, 0.12, 0.14], dtype=np.float64),
    np.array([0.18, 0.30, 0.44], dtype=np.float64),
    np.array([0.43, 0.36, 0.25], dtype=np.float64),
    np.array([0.16, 0.34, 0.22], dtype=np.float64),
]


@dataclass
class ObjectState:
    center_xy: np.ndarray
    yaw_deg: float
    size: np.ndarray


@dataclass
class Scene:
    world: World
    camera: Camera
    table: object
    object_paths: dict
    mask_objects: dict
    distractors: list
    key_light: UsdLux.DistantLight
    fill_light: UsdLux.DomeLight
    table_shader: UsdShade.Shader
    metal_shader: UsdShade.Shader


def log(message):
    print(message, flush=True)


def step_world(world, render=True):
    world.step(render=render)
    simulation_app.update()


def clamp01(value):
    return max(0.0, min(1.0, float(value)))


def sample_range(rng, lo, hi):
    lo = float(lo)
    hi = float(hi)
    if lo > hi:
        lo, hi = hi, lo
    if math.isclose(lo, hi):
        return lo
    return float(rng.uniform(lo, hi))


def make_preview_surface(stage, material_path):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.55, 0.55))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material, shader


def set_material(shader, color, metallic, roughness, emissive=None):
    color = np.asarray(color, dtype=np.float64)
    shader.GetInput("diffuseColor").Set(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    shader.GetInput("metallic").Set(float(metallic))
    shader.GetInput("roughness").Set(float(roughness))
    emissive_color = np.zeros(3, dtype=np.float64) if emissive is None else np.asarray(emissive, dtype=np.float64)
    shader.GetInput("emissiveColor").Set(
        Gf.Vec3f(float(emissive_color[0]), float(emissive_color[1]), float(emissive_color[2]))
    )


def bind_material_tree(stage, root_path, material):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() in ("Cube", "Mesh", "Xform"):
            UsdShade.MaterialBindingAPI(prim).Bind(material)


def set_object_scale(prim_path, scale):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))
            return
    xform.AddScaleOp().Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))


def set_prim_xform(prim_path, position, yaw_deg=0.0, scale=None):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {prim_path}")
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))


def set_asset_local_xform(asset_path, offset, scale=None):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(asset_path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid asset prim: {asset_path}")
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(offset[0]), float(offset[1]), float(offset[2])))
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))


ASSET_HELPER_NAMES = ("Environment", "Render", "cameraSettings", "OmniverseKit", "Perspective", "Front", "Right")
ASSET_HELPER_TYPES = {
    "Camera",
    "DistantLight",
    "DomeLight",
    "RectLight",
    "SphereLight",
    "DiskLight",
    "CylinderLight",
    "RenderSettings",
    "RenderProduct",
    "RenderVar",
}


def is_asset_helper_prim(prim):
    path_text = str(prim.GetPath())
    name = prim.GetName()
    return prim.GetTypeName() in ASSET_HELPER_TYPES or any(
        part in path_text or part == name for part in ASSET_HELPER_NAMES
    )


def hide_asset_stage_helpers(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if is_asset_helper_prim(prim):
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()


def mesh_bounds(stage, root_path):
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    mins = []
    maxs = []
    mesh_count = 0
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh" or is_asset_helper_prim(prim):
            continue
        mesh_count += 1
        aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mins.append(np.array(aligned.GetMin(), dtype=np.float64))
        maxs.append(np.array(aligned.GetMax(), dtype=np.float64))
    if mesh_count == 0:
        return None, None, 0
    lo = np.min(np.vstack(mins), axis=0)
    hi = np.max(np.vstack(maxs), axis=0)
    return lo, hi, mesh_count


def add_usd_asset(stage, name, root_path, asset_path, usd_path, manual_scale, target_size, auto_scale):
    usd_path = Path(usd_path).expanduser()
    if not usd_path.is_file():
        raise FileNotFoundError(f"Missing USD asset for {name}: {usd_path}")

    stage.DefinePrim(root_path, "Xform")
    asset_prim = stage.DefinePrim(asset_path, "Xform")
    asset_prim.GetReferences().AddReference(str(usd_path))
    set_prim_xform(root_path, np.array([0.0, 0.0, 0.0], dtype=np.float64), 0.0)
    set_asset_local_xform(asset_path, np.zeros(3, dtype=np.float64))
    hide_asset_stage_helpers(stage, asset_path)

    lo, hi, mesh_count = mesh_bounds(stage, asset_path)
    if mesh_count == 0 or lo is None or hi is None:
        raise RuntimeError(f"No mesh prims found in {usd_path}")

    raw_size = hi - lo
    if np.any(raw_size <= 1e-6):
        raise RuntimeError(f"Invalid mesh bounds for {name}: size={raw_size}")

    if auto_scale:
        scale = np.asarray(target_size, dtype=np.float64) / raw_size
    else:
        scale = np.full(3, float(manual_scale), dtype=np.float64)

    center = lo + raw_size * 0.5
    set_asset_local_xform(asset_path, -center * scale, scale)
    set_prim_xform(root_path, np.array([0.0, 0.0, -5.0], dtype=np.float64), 0.0)
    measured_size = raw_size * scale
    log(f"[dataset] using {name} asset: {usd_path}")
    log(f"[dataset] {name} mesh_count={mesh_count} raw_size_m={raw_size} rendered_size_m={measured_size}")
    return measured_size


def setup_scene():
    global OBJECT_SIZES
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    stage.DefinePrim("/World/Materials", "Scope")

    key_light = UsdLux.DistantLight.Define(stage, KEY_LIGHT_PATH)
    key_light.CreateIntensityAttr(820.0)
    key_light_xform = UsdGeom.Xformable(key_light.GetPrim())
    key_light_xform.ClearXformOpOrder()
    key_light_xform.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 20.0))

    fill_light = UsdLux.DomeLight.Define(stage, FILL_LIGHT_PATH)
    fill_light.CreateIntensityAttr(420.0)
    fill_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    table = world.scene.add(
        VisualCuboid(
            prim_path=TABLE_PATH,
            name="work_table",
            position=np.array([0.35, 0.0, -0.004], dtype=np.float64),
            scale=np.array([1.10, 0.85, 0.008], dtype=np.float64),
            color=TABLE_COLOR,
        )
    )
    OBJECT_SIZES["steel_plate"] = add_usd_asset(
        stage,
        "steel_plate",
        PLATE_PATH,
        PLATE_ASSET_PATH,
        ARGS.plate_usd,
        ARGS.plate_usd_scale,
        PLATE_SIZE,
        bool(ARGS.auto_scale_plate_usd and not ARGS.no_auto_scale_plate_usd),
    )
    OBJECT_SIZES["steel_cube"] = add_usd_asset(
        stage,
        "steel_cube",
        CUBE_PATH,
        CUBE_ASSET_PATH,
        ARGS.cube_usd,
        ARGS.cube_usd_scale,
        CUBE_SIZE,
        bool(ARGS.auto_scale_cube_usd and not ARGS.no_auto_scale_cube_usd),
    )
    mask_plate = world.scene.add(
        VisualCuboid(
            prim_path=PLATE_MASK_PATH,
            name="mask_steel_plate",
            position=np.array([0.30, 0.0, -5.0], dtype=np.float64),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
            scale=OBJECT_SIZES["steel_plate"],
            color=MASK_CLASS_COLORS_RGB[0],
        )
    )
    mask_cube = world.scene.add(
        VisualCuboid(
            prim_path=CUBE_MASK_PATH,
            name="mask_steel_cube",
            position=np.array([0.30, 0.0, -5.0], dtype=np.float64),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
            scale=OBJECT_SIZES["steel_cube"],
            color=MASK_CLASS_COLORS_RGB[1],
        )
    )

    distractors = []
    for index, prim_path in enumerate(DISTRACTOR_PATHS):
        distractors.append(
            world.scene.add(
                VisualCuboid(
                    prim_path=prim_path,
                    name=f"distractor_{index}",
                    position=np.array([0.0, 0.0, -5.0], dtype=np.float64),
                    orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
                    scale=np.array([0.08, 0.04, 0.02], dtype=np.float64),
                    color=DISTRACTOR_COLORS[index % len(DISTRACTOR_COLORS)],
                )
            )
        )

    table_mat, table_shader = make_preview_surface(stage, "/World/Materials/table")
    metal_mat, metal_shader = make_preview_surface(stage, "/World/Materials/metal")
    mask_plate_mat, mask_plate_shader = make_preview_surface(stage, "/World/Materials/mask_plate")
    mask_cube_mat, mask_cube_shader = make_preview_surface(stage, "/World/Materials/mask_cube")
    bind_material_tree(stage, TABLE_PATH, table_mat)
    bind_material_tree(stage, PLATE_MASK_PATH, mask_plate_mat)
    bind_material_tree(stage, CUBE_MASK_PATH, mask_cube_mat)
    set_material(table_shader, TABLE_COLOR, 0.0, 0.90)
    set_material(metal_shader, NORMAL_METAL_COLOR, 1.0, 0.86)
    set_material(mask_plate_shader, MASK_CLASS_COLORS_RGB[0], 0.0, 1.0, emissive=MASK_CLASS_COLORS_RGB[0])
    set_material(mask_cube_shader, MASK_CLASS_COLORS_RGB[1], 0.0, 1.0, emissive=MASK_CLASS_COLORS_RGB[1])

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.0, 0.0, 0.60], dtype=np.float64),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 90.0, 180.0]), degrees=True),
        frequency=60,
        resolution=(WIDTH, HEIGHT),
    )

    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12)

    return Scene(
        world=world,
        camera=camera,
        table=table,
        object_paths={"steel_plate": PLATE_PATH, "steel_cube": CUBE_PATH},
        mask_objects={"steel_plate": mask_plate, "steel_cube": mask_cube},
        distractors=distractors,
        key_light=key_light,
        fill_light=fill_light,
        table_shader=table_shader,
        metal_shader=metal_shader,
    )


def object_top_center(state):
    return np.array([state.center_xy[0], state.center_xy[1], float(state.size[2])], dtype=np.float64)


def object_box_corners(state):
    size = np.asarray(state.size, dtype=np.float64)
    half = size * 0.5
    local = np.array(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    yaw = math.radians(float(state.yaw_deg))
    rot = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    center = np.array([state.center_xy[0], state.center_xy[1], half[2]], dtype=np.float64)
    return local @ rot.T + center


def object_radius_xy(name):
    size = OBJECT_SIZES[name]
    return float(np.linalg.norm(size[:2]) * 0.5)


def sample_yaw_deg(rng, name):
    if name == "steel_cube" and not ARGS.cube_yaw_random:
        return 0.0
    if name == "steel_plate" and rng.random() < clamp01(ARGS.axis_aligned_ratio):
        base_yaw = float(rng.choice(np.array([-90.0, 0.0, 90.0], dtype=np.float64)))
        return base_yaw + sample_range(rng, -ARGS.axis_yaw_jitter_deg, ARGS.axis_yaw_jitter_deg)
    return sample_range(rng, -85.0, 85.0)


def choose_scene_objects(rng, allow_background=True):
    if allow_background and rng.random() < clamp01(ARGS.background_ratio):
        return []
    if rng.random() < clamp01(ARGS.both_ratio):
        return ["steel_plate", "steel_cube"]
    return ["steel_cube" if rng.random() < clamp01(ARGS.cube_ratio) else "steel_plate"]


def sample_single_center(rng):
    return np.array([sample_range(rng, 0.16, 0.54), sample_range(rng, -0.24, 0.24)], dtype=np.float64)


def sample_object_states(rng, active_names):
    if not active_names:
        return {}
    if len(active_names) == 1:
        name = active_names[0]
        return {
            name: ObjectState(
                center_xy=sample_single_center(rng),
                yaw_deg=sample_yaw_deg(rng, name),
                size=np.array(OBJECT_SIZES[name], dtype=np.float64),
            )
        }

    for _ in range(100):
        center = np.array([sample_range(rng, 0.24, 0.46), sample_range(rng, -0.12, 0.12)], dtype=np.float64)
        angle = sample_range(rng, -math.pi, math.pi)
        gap = object_radius_xy("steel_plate") + object_radius_xy("steel_cube") + sample_range(rng, 0.04, 0.12)
        offset = np.array([math.cos(angle), math.sin(angle)], dtype=np.float64) * (gap * 0.5)
        centers = {"steel_plate": center - offset, "steel_cube": center + offset}
        if all(0.10 <= c[0] <= 0.60 and -0.28 <= c[1] <= 0.28 for c in centers.values()):
            return {
                name: ObjectState(
                    center_xy=centers[name],
                    yaw_deg=sample_yaw_deg(rng, name),
                    size=np.array(OBJECT_SIZES[name], dtype=np.float64),
                )
                for name in active_names
            }
    raise RuntimeError("Could not sample non-overlapping object positions.")


def distractor_collides(center_xy, size_xy, states):
    half_diag = float(np.linalg.norm(size_xy) * 0.5)
    for name, state in states.items():
        min_clearance = half_diag + object_radius_xy(name) + 0.025
        if float(np.linalg.norm(center_xy - state.center_xy)) < min_clearance:
            return True
    return False


def sample_distractor_states(rng, active_names, states):
    if ARGS.max_distractors <= 0:
        return []
    ratio = clamp01(ARGS.object_distractor_ratio if active_names else ARGS.distractor_ratio)
    if rng.random() >= ratio:
        return []

    count = int(rng.integers(1, min(int(ARGS.max_distractors), len(DISTRACTOR_PATHS)) + 1))
    sampled = []
    for _ in range(count):
        for _attempt in range(60):
            kind = int(rng.integers(0, 4))
            if kind == 0:
                size = np.array([sample_range(rng, 0.10, 0.22), sample_range(rng, 0.025, 0.055), 0.018])
            elif kind == 1:
                size = np.array([sample_range(rng, 0.035, 0.070), sample_range(rng, 0.09, 0.16), 0.018])
            elif kind == 2:
                size = np.array([sample_range(rng, 0.04, 0.075), sample_range(rng, 0.04, 0.075), 0.040])
            else:
                size = np.array([sample_range(rng, 0.12, 0.20), sample_range(rng, 0.12, 0.20), 0.007])
            center_xy = np.array([sample_range(rng, 0.12, 0.62), sample_range(rng, -0.30, 0.30)], dtype=np.float64)
            if not distractor_collides(center_xy, size[:2], states):
                sampled.append({"center_xy": center_xy, "size": size, "yaw_deg": sample_range(rng, -90.0, 90.0)})
                break
    return sampled


def set_mask_pose(obj, prim_path, state):
    position = np.array([state.center_xy[0], state.center_xy[1], float(state.size[2]) / 2.0], dtype=np.float64)
    obj.set_world_pose(
        position=position,
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, state.yaw_deg]), degrees=True),
    )
    set_object_scale(prim_path, state.size)


def set_asset_pose(prim_path, state):
    position = np.array([state.center_xy[0], state.center_xy[1], float(state.size[2]) / 2.0], dtype=np.float64)
    set_prim_xform(prim_path, position, state.yaw_deg)


def hide_asset(prim_path):
    set_prim_xform(prim_path, np.array([0.0, 0.0, -5.0], dtype=np.float64), 0.0)


def hide_object(obj):
    obj.set_world_pose(position=np.array([0.0, 0.0, -5.0], dtype=np.float64))


def set_distractors(distractors, distractor_states):
    for index, obj in enumerate(distractors):
        if index >= len(distractor_states):
            hide_object(obj)
            continue
        state = distractor_states[index]
        size = np.asarray(state["size"], dtype=np.float64)
        center_xy = np.asarray(state["center_xy"], dtype=np.float64)
        obj.set_world_pose(
            position=np.array([center_xy[0], center_xy[1], float(size[2]) / 2.0], dtype=np.float64),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, state["yaw_deg"]]), degrees=True),
        )
        set_object_scale(DISTRACTOR_PATHS[index], size)


def set_table_visible(scene, visible):
    position = np.array([0.35, 0.0, -0.004 if visible else -5.0], dtype=np.float64)
    scene.table.set_world_pose(position=position)


def set_render_scene(scene, active_names, states, distractor_states, mask_mode):
    active_set = set(active_names)
    set_table_visible(scene, not mask_mode)
    set_material(scene.table_shader, MASK_TABLE_COLOR if mask_mode else TABLE_COLOR, 0.0, 0.95)
    set_material(scene.metal_shader, NORMAL_METAL_COLOR, 1.0, 0.86)

    for name, prim_path in scene.object_paths.items():
        if not mask_mode and name in active_set:
            set_asset_pose(prim_path, states[name])
        else:
            hide_asset(prim_path)

    for name, obj in scene.mask_objects.items():
        if mask_mode and name in active_set:
            set_mask_pose(obj, MASK_OBJECT_PATHS[name], states[name])
        else:
            hide_object(obj)

    set_distractors(scene.distractors, [] if mask_mode else distractor_states)


def distance_limits():
    lo = max(0.10, min(float(ARGS.camera_distance_min), float(ARGS.camera_distance_max)))
    hi = min(MAX_CAMERA_OBJECT_DISTANCE_M, max(float(ARGS.camera_distance_min), float(ARGS.camera_distance_max)))
    if lo > hi:
        raise ValueError("Invalid camera distance range after clamping to [0.10, 1.00] m.")
    return lo, hi


def wrist_distance_limits(rng):
    lo, hi = wrist_base_distance_limits()
    if rng.random() < clamp01(ARGS.close_sample_ratio):
        close_lo = min(float(ARGS.wrist_close_distance_min), float(ARGS.wrist_close_distance_max))
        close_hi = max(float(ARGS.wrist_close_distance_min), float(ARGS.wrist_close_distance_max))
        close_lo = max(lo, close_lo)
        close_hi = min(hi, close_hi)
        if close_lo <= close_hi:
            return close_lo, close_hi
    return lo, hi


def wrist_base_distance_limits():
    global_lo, global_hi = distance_limits()
    lo = max(global_lo, min(float(ARGS.wrist_camera_distance_min), float(ARGS.wrist_camera_distance_max)))
    hi = min(global_hi, max(float(ARGS.wrist_camera_distance_min), float(ARGS.wrist_camera_distance_max)))
    if lo > hi:
        raise ValueError("Invalid wrist camera distance range after clamping to [0.10, 1.00] m.")
    return lo, hi


def object_distances(camera_pos, states):
    return {name: float(np.linalg.norm(camera_pos - object_top_center(state))) for name, state in states.items()}


def object_max_corner_distances(camera_pos, states):
    return {
        name: float(np.max(np.linalg.norm(object_box_corners(state) - camera_pos.reshape(1, 3), axis=1)))
        for name, state in states.items()
    }


def set_camera_look_at(camera, camera_pos, target_pos):
    quat = gf_rot_utils.lookat_to_quatf(
        Gf.Vec3f(float(camera_pos[0]), float(camera_pos[1]), float(camera_pos[2])),
        Gf.Vec3f(float(target_pos[0]), float(target_pos[1]), float(target_pos[2])),
        Gf.Vec3f(0.0, 1.0, 0.0),
    )
    camera.set_world_pose(position=np.asarray(camera_pos, dtype=np.float64), orientation=rot_utils.gf_quat_to_tensor(quat))


def sample_camera_pose(rng, active_names, states):
    if active_names:
        centers = np.array([object_top_center(state) for state in states.values()], dtype=np.float64)
        target_pos = centers.mean(axis=0)
        target_pos[2] = max(0.02, target_pos[2] * 0.65)
    else:
        target_pos = np.array([sample_range(rng, 0.22, 0.50), sample_range(rng, -0.18, 0.18), 0.02])

    jitter = max(0.0, float(ARGS.wrist_target_jitter_xy))
    if jitter > 0.0:
        target_pos[:2] += rng.uniform(-jitter, jitter, size=2)

    if ARGS.camera_mode == "wrist-d455":
        dist_min, dist_max = wrist_distance_limits(rng)
        elev_lo = min(float(ARGS.wrist_elevation_min_deg), float(ARGS.wrist_elevation_max_deg))
        elev_hi = max(float(ARGS.wrist_elevation_min_deg), float(ARGS.wrist_elevation_max_deg))
        az_lo = min(float(ARGS.wrist_azimuth_min_deg), float(ARGS.wrist_azimuth_max_deg))
        az_hi = max(float(ARGS.wrist_azimuth_min_deg), float(ARGS.wrist_azimuth_max_deg))
    else:
        dist_min, dist_max = distance_limits()
        elev_lo = min(float(ARGS.lookat_elevation_min_deg), float(ARGS.lookat_elevation_max_deg))
        elev_hi = max(float(ARGS.lookat_elevation_min_deg), float(ARGS.lookat_elevation_max_deg))
        az_lo = min(float(ARGS.lookat_azimuth_min_deg), float(ARGS.lookat_azimuth_max_deg))
        az_hi = max(float(ARGS.lookat_azimuth_min_deg), float(ARGS.lookat_azimuth_max_deg))

    for _ in range(250):
        distance = sample_range(rng, dist_min, dist_max)
        elevation = math.radians(sample_range(rng, elev_lo, elev_hi))
        azimuth = math.radians(sample_range(rng, az_lo, az_hi))
        horizontal = distance * math.cos(elevation)
        camera_pos = target_pos + np.array(
            [
                horizontal * math.cos(azimuth),
                horizontal * math.sin(azimuth),
                distance * math.sin(elevation),
            ],
            dtype=np.float64,
        )
        camera_pos[2] = max(camera_pos[2], 0.18)
        distances = object_distances(camera_pos, states)
        max_corner_distances = object_max_corner_distances(camera_pos, states)
        if not distances or (
            all(dist_min <= value <= dist_max for value in distances.values())
            and all(value <= MAX_CAMERA_OBJECT_DISTANCE_M for value in max_corner_distances.values())
        ):
            return camera_pos, target_pos, distances

    raise RuntimeError("Could not sample a camera pose with every object within 1 m.")


def rgba_to_bgr(rgba):
    img = np.asarray(rgba)
    if img.size == 0:
        raise RuntimeError("Camera returned an empty RGBA buffer.")
    if img.dtype != np.uint8:
        if float(np.nanmax(img)) <= 1.0:
            img = img * 255.0
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)


def capture_bgr(scene, frames):
    rgba = None
    for _ in range(max(1, int(frames))):
        step_world(scene.world, render=True)
        rgba = scene.camera.get_rgba()

    for _ in range(max(1, int(ARGS.camera_retry_frames))):
        arr = np.asarray(rgba)
        if arr.size > 0 and arr.ndim >= 3 and arr.shape[0] > 0 and arr.shape[1] > 0:
            return rgba_to_bgr(rgba)
        step_world(scene.world, render=True)
        rgba = scene.camera.get_rgba()
    raise RuntimeError(f"Camera returned empty RGBA for {ARGS.camera_retry_frames} retry frames.")


def order_points_clockwise(points):
    points = np.asarray(points, dtype=np.float32)
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def box_intersects_image(box):
    box = np.asarray(box, dtype=np.float32)
    return not (
        float(np.max(box[:, 0])) < 0.0
        or float(np.min(box[:, 0])) > WIDTH - 1.0
        or float(np.max(box[:, 1])) < 0.0
        or float(np.min(box[:, 1])) > HEIGHT - 1.0
    )


def clip_box_to_image(box):
    clipped = np.asarray(box, dtype=np.float32).copy()
    clipped[:, 0] = np.clip(clipped[:, 0], 0.0, WIDTH - 1.0)
    clipped[:, 1] = np.clip(clipped[:, 1], 0.0, HEIGHT - 1.0)
    return order_points_clockwise(clipped)


def validate_label_box(box, reason_prefix):
    box = order_points_clockwise(np.asarray(box, dtype=np.float32))
    if not box_intersects_image(box):
        return None, f"{reason_prefix}_outside_image"

    inside = (box[:, 0] >= 0.0).all() and (box[:, 0] <= WIDTH - 1.0).all()
    inside = inside and (box[:, 1] >= 0.0).all() and (box[:, 1] <= HEIGHT - 1.0).all()
    if not inside:
        if not ARGS.clip_labels:
            return None, f"{reason_prefix}_crosses_image_edge"
        box = clip_box_to_image(box)

    (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(box.astype(np.float32))
    min_side = float(min(rect_w, rect_h))
    if min_side < float(ARGS.min_label_side_px):
        return None, f"{reason_prefix}_tiny_side={min_side:.1f}"

    area = float(abs(cv2.contourArea(box.astype(np.float32))))
    if area < float(ARGS.min_label_area_px):
        return None, f"{reason_prefix}_tiny_area={area:.1f}"

    return box, "ok"


def obb_label_from_box(class_id, box):
    box = order_points_clockwise(np.asarray(box, dtype=np.float32))
    norm = box / np.array([WIDTH, HEIGHT], dtype=np.float32)
    norm = np.clip(norm, 0.0, 1.0)
    values = [str(class_id)] + [f"{x:.6f} {y:.6f}" for x, y in norm]
    return " ".join(values)


def class_mask_for_bgr(mask_bgr, class_id):
    color_rgb = (MASK_CLASS_COLORS_RGB[class_id] * 255.0).astype(np.float32)
    color_bgr = color_rgb[::-1]
    diff = np.linalg.norm(mask_bgr.astype(np.float32) - color_bgr.reshape(1, 1, 3), axis=2)
    color_mask = diff < 170.0

    hsv = cv2.cvtColor(mask_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    if class_id == 0:
        hue_mask = (hue >= 35) & (hue <= 95) & (sat > 35) & (val > 20)
    else:
        hue_mask = (hue >= 3) & (hue <= 35) & (sat > 35) & (val > 20)

    mask = (color_mask | hue_mask).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def make_label_from_binary_mask(mask, class_id, reason_prefix):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, f"{reason_prefix}_no_component"

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < float(ARGS.min_label_area_px):
        return None, f"{reason_prefix}_tiny_area={area:.1f}"

    rect = cv2.minAreaRect(contour)
    box, reason = validate_label_box(cv2.boxPoints(rect), reason_prefix)
    if box is None:
        return None, reason
    return obb_label_from_box(class_id, box), "ok"


def foreground_mask_for_bgr(mask_bgr):
    hsv = cv2.cvtColor(mask_bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    mask = ((sat > 35) & (val > 20)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def make_mask_label(mask_bgr, name, allow_foreground_fallback=False):
    class_id = CLASS_IDS[name]
    label, reason = make_label_from_binary_mask(class_mask_for_bgr(mask_bgr, class_id), class_id, "mask")
    if label is not None:
        return label, reason

    if not allow_foreground_fallback:
        return None, reason

    fallback_label, fallback_reason = make_label_from_binary_mask(
        foreground_mask_for_bgr(mask_bgr),
        class_id,
        "mask_foreground",
    )
    if fallback_label is not None:
        return fallback_label, "ok"
    return None, f"{reason}; fallback={fallback_reason}"


def make_mask_labels(mask_bgr, active_names):
    labels = []
    allow_foreground_fallback = len(active_names) == 1
    for name in active_names:
        label, reason = make_mask_label(mask_bgr, name, allow_foreground_fallback)
        if label is None:
            return [], f"{name}:{reason}"
        labels.append(label)
    return labels, "ok"


def clean_dataset(root):
    for pattern in (
        "images/train/*.png",
        "images/val/*.png",
        "labels/train/*.txt",
        "labels/val/*.txt",
        "guides/*.png",
        "rejects/*.png",
        "rejects/*.txt",
        "metal_objects_obb.yaml",
    ):
        for path in root.glob(pattern):
            path.unlink()


def prepare_dirs(root):
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    (root / "guides").mkdir(parents=True, exist_ok=True)


def write_yaml(root):
    yaml_path = root / "metal_objects_obb.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {root.resolve()}",
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


def parse_label_class_ids(label_path):
    class_ids = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if parts:
            class_ids.append(int(parts[0]))
    return class_ids


def existing_sample_index(path):
    try:
        return int(path.stem.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return -1


def iter_existing_samples(root):
    records = []
    for split in ("val", "train"):
        for label_path in (root / "labels" / split).glob("*.txt"):
            image_path = root / "images" / split / f"{label_path.stem}.png"
            if image_path.is_file():
                records.append((existing_sample_index(label_path), image_path, label_path))
    for _, image_path, label_path in sorted(records, key=lambda item: (item[0] < 0, item[0], str(item[1]))):
        yield image_path, label_path


def scan_existing_dataset(root):
    sample_count = 0
    max_index = -1
    split_counts = {
        "train": {0: 0, 1: 0, "background": 0},
        "val": {0: 0, 1: 0, "background": 0},
    }
    for split in ("train", "val"):
        for label_path in (root / "labels" / split).glob("*.txt"):
            image_path = root / "images" / split / f"{label_path.stem}.png"
            if not image_path.is_file():
                continue
            sample_count += 1
            max_index = max(max_index, existing_sample_index(label_path))
            ids = parse_label_class_ids(label_path)
            if not ids:
                split_counts[split]["background"] += 1
            for class_id in ids:
                if class_id in (0, 1):
                    split_counts[split][class_id] += 1
    guide_count = sum(1 for _ in (root / "guides").glob("guide_*.png"))
    return sample_count, max_index + 1, guide_count, split_counts


def label_polygon_px(label):
    values = label.split()
    coords = np.array([float(v) for v in values[1:]], dtype=np.float64).reshape(4, 2)
    coords *= np.array([WIDTH, HEIGHT], dtype=np.float64)
    return coords.astype(np.int32)


def label_class_id(label):
    return int(label.split()[0])


def draw_text_tag(image, text, origin, color):
    x, y = int(origin[0]), int(origin[1])
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(2, min(WIDTH - tw - 8, x))
    y = max(th + 6, min(HEIGHT - baseline - 4, y))
    cv2.rectangle(image, (x - 4, y - th - 5), (x + tw + 4, y + baseline + 4), (25, 25, 25), -1)
    cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def make_guide_image(bgr, labels):
    guide = bgr.copy()
    overlay = guide.copy()
    if not labels:
        draw_text_tag(guide, "background sample", (16, 28), (255, 120, 80))
        return guide

    for label in labels:
        class_id = label_class_id(label)
        color = GUIDE_COLORS.get(class_id, (255, 255, 255))
        polygon = label_polygon_px(label)
        cv2.fillConvexPoly(overlay, polygon, color)
        cv2.polylines(guide, [polygon], True, color, 2, cv2.LINE_AA)
        center = polygon.mean(axis=0)
        draw_text_tag(guide, CLASS_NAMES.get(class_id, str(class_id)), (center[0] + 6, center[1] - 6), color)
    return cv2.addWeighted(overlay, 0.24, guide, 0.76, 0.0)


def write_guide_image(root, bgr, labels, guide_index, stem):
    if guide_index >= max(0, int(ARGS.guide_count)):
        return False
    guide_path = root / "guides" / f"guide_{guide_index:02d}_{stem}.png"
    if not cv2.imwrite(str(guide_path), make_guide_image(bgr, labels)):
        raise RuntimeError(f"Failed to write guide image: {guide_path}")
    return True


def backfill_guides(root, guide_count):
    target = max(0, int(ARGS.guide_count))
    if guide_count >= target:
        return guide_count
    for image_path, label_path in iter_existing_samples(root):
        if guide_count >= target:
            break
        bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        labels = [line.strip() for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        guide_path = root / "guides" / f"guide_{guide_count:02d}_{image_path.stem}.png"
        if cv2.imwrite(str(guide_path), make_guide_image(bgr, labels)):
            guide_count += 1
    return guide_count


def write_debug_reject(root, bgr, mask_bgr, stem, reason):
    if ARGS.max_debug_rejects <= 0:
        return
    reject_dir = Path(ARGS.debug_reject_dir).expanduser() if ARGS.debug_reject_dir else root / "rejects"
    reject_dir.mkdir(parents=True, exist_ok=True)
    existing = len(list(reject_dir.glob("reject_*.txt")))
    if existing >= int(ARGS.max_debug_rejects):
        return
    safe_stem = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in stem)
    prefix = reject_dir / f"reject_{existing:04d}_{safe_stem}"
    cv2.imwrite(str(prefix.with_suffix(".png")), bgr)
    if mask_bgr is not None:
        cv2.imwrite(str(reject_dir / f"{prefix.name}_mask.png"), mask_bgr)
    prefix.with_suffix(".txt").write_text(reason + "\n", encoding="utf-8")


def scene_stem(active_names, index):
    if not active_names:
        name = "background"
    else:
        name = "_".join(name.replace("steel_", "") for name in active_names)
    return f"{name}_{index:06d}"


def write_sample(root, split, stem, bgr, labels):
    image_path = root / "images" / split / f"{stem}.png"
    label_path = root / "labels" / split / f"{stem}.txt"
    if not cv2.imwrite(str(image_path), bgr):
        raise RuntimeError(f"Failed to write image: {image_path}")
    label_text = "\n".join(labels)
    if label_text:
        label_text += "\n"
    label_path.write_text(label_text, encoding="utf-8")


def format_distances(distances):
    if not distances:
        return "background"
    return ", ".join(f"{name}={distance:.3f}m" for name, distance in sorted(distances.items()))


def main():
    rng = np.random.default_rng(int(ARGS.seed))
    root = Path(ARGS.dataset_dir).expanduser()
    prepare_dirs(root)
    if ARGS.clean:
        clean_dataset(root)

    existing_count = 0
    next_index = 0
    guide_written = 0
    split_counts = {
        "train": {0: 0, 1: 0, "background": 0},
        "val": {0: 0, 1: 0, "background": 0},
    }
    if not ARGS.clean and not ARGS.no_resume:
        existing_count, next_index, guide_written, split_counts = scan_existing_dataset(root)
        guide_written = backfill_guides(root, guide_written)

    distance_min, distance_max = wrist_base_distance_limits()
    log(f"[dataset] output dir: {root}")
    log(f"[dataset] target images: {ARGS.count}")
    log(
        "[dataset] camera: "
        f"mode={ARGS.camera_mode} distance=[{distance_min:.2f}, {distance_max:.2f}]m "
        f"elevation=[{min(ARGS.wrist_elevation_min_deg, ARGS.wrist_elevation_max_deg):.1f}, "
        f"{max(ARGS.wrist_elevation_min_deg, ARGS.wrist_elevation_max_deg):.1f}]deg "
        f"azimuth=[{min(ARGS.wrist_azimuth_min_deg, ARGS.wrist_azimuth_max_deg):.1f}, "
        f"{max(ARGS.wrist_azimuth_min_deg, ARGS.wrist_azimuth_max_deg):.1f}]deg"
    )
    log("[dataset] labels: mask-render only")
    if existing_count:
        log(f"[dataset] resume: {existing_count} existing image/label pairs; next index={next_index}")

    scene = None
    try:
        scene = setup_scene()
        for _ in range(max(0, int(ARGS.warmup_frames))):
            step_world(scene.world, render=True)

        written = existing_count
        new_written = 0
        attempts = 0
        max_attempts = max(int(ARGS.count) + 200, int(ARGS.count) * int(ARGS.max_attempts_multiplier))
        val_count = int(round(int(ARGS.count) * float(ARGS.val_ratio)))
        target_background_count = int(round(int(ARGS.count) * clamp01(ARGS.background_ratio)))

        while written < int(ARGS.count):
            attempts += 1
            if attempts > max_attempts:
                raise RuntimeError(
                    f"Too many rejected samples: written={written}, attempts={attempts}, target={ARGS.count}"
                )

            accepted_background_count = split_counts["train"]["background"] + split_counts["val"]["background"]
            active_names = choose_scene_objects(rng, allow_background=accepted_background_count < target_background_count)
            states = sample_object_states(rng, active_names)
            distractor_states = sample_distractor_states(rng, active_names, states)
            camera_pos, target_pos, distances = sample_camera_pose(rng, active_names, states)
            set_camera_look_at(scene.camera, camera_pos, target_pos)

            set_render_scene(scene, active_names, states, distractor_states, mask_mode=False)
            step_world(scene.world, render=False)
            bgr = capture_bgr(scene, ARGS.frames_per_sample)

            if ARGS.debug_first_frame:
                debug_path = Path(ARGS.debug_first_frame).expanduser()
                debug_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(debug_path), bgr)
                log(f"[dataset] wrote debug first frame: {debug_path}")
                return

            labels = []
            mask_bgr = None
            reject_reason = "ok"
            if active_names:
                set_render_scene(scene, active_names, states, [], mask_mode=True)
                step_world(scene.world, render=False)
                mask_bgr = capture_bgr(scene, ARGS.mask_render_frames)
                labels, reject_reason = make_mask_labels(mask_bgr, active_names)

            if active_names and not labels:
                write_debug_reject(root, bgr, mask_bgr, scene_stem(active_names, attempts), reject_reason)
                continue

            split = "val" if written < val_count else "train"
            stem = scene_stem(active_names, next_index)
            write_sample(root, split, stem, bgr, labels)
            if write_guide_image(root, bgr, labels, guide_written, stem):
                guide_written += 1

            if not labels:
                split_counts[split]["background"] += 1
            for label in labels:
                split_counts[split][label_class_id(label)] += 1

            written += 1
            new_written += 1
            next_index += 1
            if written == 1 or written % 50 == 0 or written == int(ARGS.count):
                log(
                    f"[dataset] {written}/{ARGS.count} images written "
                    f"(new={new_written}, attempts={attempts}, last_dist={format_distances(distances)})"
                )

        yaml_path = write_yaml(root)
        log("\n=== YOLO OBB dataset ready ===")
        log(f"dataset:      {root}")
        log(f"yaml:         {yaml_path}")
        log(f"attempts:     {attempts}")
        log(f"new_images:   {new_written}")
        log(f"total_images: {written}/{ARGS.count}")
        log(f"guide_images: {guide_written}")
        log(f"train_plate:  {split_counts['train'][0]}")
        log(f"train_cube:   {split_counts['train'][1]}")
        log(f"train_bg:     {split_counts['train']['background']}")
        log(f"val_plate:    {split_counts['val'][0]}")
        log(f"val_cube:     {split_counts['val'][1]}")
        log(f"val_bg:       {split_counts['val']['background']}")
        log("==============================\n")
    finally:
        if scene is not None:
            scene.world.stop()
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        print("\n[dataset] FATAL: dataset generation failed", file=sys.stderr, flush=True)
        print(f"[dataset] exception: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        traceback.print_exc()
        try:
            simulation_app.close()
        except Exception:
            pass
        raise
