from __future__ import annotations

# Generate a synthetic YOLO OBB dataset from Isaac Sim.
# Classes:
#   0: steel_plate
#   1: steel_cube
#
# Output:
#   work/datasets/metal_objects_obb/
#     images/train/*.png
#     images/val/*.png
#     labels/train/*.txt
#     labels/val/*.txt
#     metal_objects_obb.yaml
#
# Run:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/generate_steel_plate_obb_dataset.py \
#   --count 1000

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    root = Path(__file__).resolve().parents[2]
    assets_dir = Path(__file__).resolve().parent / "assets"
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--cube-ratio", type=float, default=0.45)
    parser.add_argument(
        "--no-balance-classes",
        action="store_true",
        help="Use random class sampling only. By default the final dataset follows --cube-ratio.",
    )
    parser.add_argument(
        "--cube-yaw-random",
        action="store_true",
        help="Randomize cube yaw. Default keeps cube yaw at 0 because square/cube yaw is not useful.",
    )
    parser.add_argument(
        "--fixed-cube-size",
        type=float,
        default=0.05,
        help="Fixed steel cube side length in meters. Project default is 0.05 m.",
    )
    parser.add_argument(
        "--cube-usd",
        default=str(assets_dir / "iron_cube.usd"),
        help="Optional USD asset for steel_cube visuals when --use-cube-usd is set.",
    )
    parser.add_argument(
        "--use-cube-usd",
        action="store_true",
        help="Use --cube-usd for training images. Default uses a simple visible VisualCuboid.",
    )
    parser.add_argument(
        "--phys-iron-usd",
        default=str(assets_dir / "phys_iron.usd"),
        help="Iron physics-material USD noted for later pick/place integration.",
    )
    parser.add_argument(
        "--no-cube-usd",
        action="store_true",
        help="Ignore --cube-usd and use a simple VisualCuboid for steel_cube.",
    )
    parser.add_argument(
        "--cube-usd-scale",
        type=float,
        default=1.0,
        help="Manual wrapper scale for --cube-usd when auto scaling is disabled or cannot measure the asset.",
    )
    parser.add_argument(
        "--no-auto-scale-cube-usd",
        action="store_true",
        help="Do not auto-scale the USD cube visual to --fixed-cube-size.",
    )
    parser.add_argument("--frames-per-sample", type=int, default=2)
    parser.add_argument(
        "--difficulty",
        choices=("starter", "balanced", "hard"),
        default="balanced",
        help="starter is easier for first YOLO training; hard adds stronger glare/noise.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Initial render frames before the first camera capture.",
    )
    parser.add_argument(
        "--camera-retry-frames",
        type=int,
        default=60,
        help="Extra render frames to wait if the camera returns an empty RGBA buffer.",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--clean", action="store_true", help="Remove old images/labels in this dataset directory first.")
    parser.add_argument(
        "--max-attempts-multiplier",
        type=int,
        default=80,
        help="Stop if projected labels keep failing after count * this many tries.",
    )
    parser.add_argument(
        "--dataset-dir",
        default=str(root / "work" / "datasets" / "metal_objects_obb"),
    )
    parser.add_argument(
        "--no-visibility-check",
        action="store_true",
        help="Write projected labels even if the object is not visibly distinguishable in RGB.",
    )
    parser.add_argument(
        "--min-visible-contrast",
        type=float,
        default=8.0,
        help="Minimum LAB median contrast between label polygon and nearby background.",
    )
    parser.add_argument(
        "--min-visible-edge-density",
        type=float,
        default=0.012,
        help="Fallback edge density inside the label polygon for low-contrast shiny samples.",
    )
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import math

import cv2
import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils


WIDTH, HEIGHT = 640, 480
FX, FY = 500.0, 500.0
CX, CY = WIDTH / 2.0, HEIGHT / 2.0

CAMERA_PATH = "/World/top_camera"
PLATE_PATH = "/World/steel_plate"
CUBE_PATH = "/World/steel_cube"
CUBE_ASSET_PATH = "/World/steel_cube/asset"
TABLE_PATH = "/World/work_table"
KEY_LIGHT_PATH = "/World/key_light"

PLATE_SIZE = np.array([0.32, 0.14, 0.03], dtype=np.float64)
DEFAULT_CUBE_SIZE = np.array([0.05, 0.05, 0.05], dtype=np.float64)
OBJECT_SPECS = {
    "steel_plate": {"class_id": 0, "path": PLATE_PATH, "size": PLATE_SIZE},
    "steel_cube": {"class_id": 1, "path": CUBE_PATH, "size": DEFAULT_CUBE_SIZE},
}
CUBE_USD_WORLD_SCALE = ARGS.cube_usd_scale

T_GL_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])


def get_tf(path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64).T


def rgba_to_bgr(rgba):
    img = np.asarray(rgba)
    if img.size == 0:
        raise RuntimeError("Camera returned an empty RGBA buffer.")
    if img.dtype != np.uint8:
        if float(np.nanmax(img)) <= 1.0:
            img = img * 255.0
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)


def make_preview_surface(stage, material_path):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.55, 0.55))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material, shader


def bind_material(stage, prim_path, material):
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def bind_material_tree(stage, root_path, material):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() in ("Cube", "Mesh", "Xform"):
            UsdShade.MaterialBindingAPI(prim).Bind(material)


def set_material(shader, color, metallic, roughness):
    shader.GetInput("diffuseColor").Set(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    shader.GetInput("metallic").Set(float(metallic))
    shader.GetInput("roughness").Set(float(roughness))


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


def measured_cube_usd_scale(stage):
    if ARGS.no_auto_scale_cube_usd:
        return float(ARGS.cube_usd_scale)

    prim = stage.GetPrimAtPath(CUBE_ASSET_PATH)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    size = np.array(bbox_range.GetSize(), dtype=np.float64)
    max_dim = float(np.max(size)) if size.size else 0.0
    if max_dim <= 1e-6:
        print("[dataset] could not measure cube USD bounds; using --cube-usd-scale", flush=True)
        return float(ARGS.cube_usd_scale)
    return float(ARGS.fixed_cube_size / max_dim)


def add_cube_from_usd(stage):
    global CUBE_USD_WORLD_SCALE
    if ARGS.no_cube_usd or not ARGS.use_cube_usd:
        return False
    cube_usd = Path(ARGS.cube_usd).expanduser()
    if not cube_usd.is_file():
        print(f"[dataset] cube USD missing, using VisualCuboid fallback: {cube_usd}", flush=True)
        return False

    cube_root = stage.DefinePrim(CUBE_PATH, "Xform")
    asset_prim = stage.DefinePrim(CUBE_ASSET_PATH, "Xform")
    asset_prim.GetReferences().AddReference(str(cube_usd))
    CUBE_USD_WORLD_SCALE = measured_cube_usd_scale(stage)
    set_prim_xform(CUBE_PATH, np.array([0.0, 0.0, -5.0]), 0.0, np.full(3, CUBE_USD_WORLD_SCALE))
    print(f"[dataset] using steel_cube USD: {cube_usd}", flush=True)
    print(f"[dataset] cube USD wrapper scale: {CUBE_USD_WORLD_SCALE:.6f}", flush=True)

    phys_iron = Path(ARGS.phys_iron_usd).expanduser()
    if phys_iron.is_file():
        print(f"[dataset] noted iron physics material USD: {phys_iron}", flush=True)
    return True


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    stage.DefinePrim("/World/Materials", "Scope")

    light = UsdLux.DistantLight.Define(stage, KEY_LIGHT_PATH)
    light.CreateIntensityAttr(650.0)
    light_xform = UsdGeom.Xformable(light.GetPrim())
    light_xform.ClearXformOpOrder()
    light_rot_op = light_xform.AddRotateXYZOp()
    light_rot_op.Set(Gf.Vec3f(35.0, 0.0, 20.0))

    table = world.scene.add(
        VisualCuboid(
            prim_path=TABLE_PATH,
            name="work_table",
            position=np.array([0.35, 0.0, -0.004]),
            scale=np.array([1.10, 0.85, 0.008]),
            color=np.array([0.45, 0.55, 0.60]),
        )
    )
    plate = world.scene.add(
        VisualCuboid(
            prim_path=PLATE_PATH,
            name="steel_plate",
            position=np.array([0.25, 0.0, PLATE_SIZE[2] / 2.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
            scale=PLATE_SIZE,
            color=np.array([0.70, 0.70, 0.70]),
        )
    )
    cube_uses_usd = add_cube_from_usd(stage)
    cube = None
    if not cube_uses_usd:
        print("[dataset] using simple VisualCuboid for steel_cube", flush=True)
        cube = world.scene.add(
            VisualCuboid(
                prim_path=CUBE_PATH,
                name="steel_cube",
                position=np.array([0.25, 0.0, -5.0]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
                scale=DEFAULT_CUBE_SIZE,
                color=np.array([0.70, 0.70, 0.70]),
            )
        )

    object_mat, object_shader = make_preview_surface(stage, "/World/Materials/random_metal")
    table_mat, table_shader = make_preview_surface(stage, "/World/Materials/random_table")
    bind_material(stage, PLATE_PATH, object_mat)
    bind_material_tree(stage, CUBE_PATH, object_mat)
    bind_material(stage, TABLE_PATH, table_mat)

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.35, 0.0, 1.35]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 90.0, 180.0]), degrees=True),
        frequency=10,
        resolution=(WIDTH, HEIGHT),
    )
    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12)
    return world, camera, plate, cube, light, light_rot_op, object_shader, table_shader


def project_world_to_pixel(p_world, t_world_camera):
    p_h = np.array([p_world[0], p_world[1], p_world[2], 1.0], dtype=np.float64)
    p_gl = np.linalg.inv(t_world_camera) @ p_h
    p_cv = T_GL_TO_CV @ p_gl
    if p_cv[2] <= 1e-6:
        return None
    u = FX * (p_cv[0] / p_cv[2]) + CX
    v = FY * (p_cv[1] / p_cv[2]) + CY
    return np.array([u, v], dtype=np.float64)


def top_face_corners_world(center_xy, yaw_rad, size):
    lx = size[0] / 2.0
    ly = size[1] / 2.0
    top_z = float(size[2])
    local = np.array(
        [
            [-lx, -ly],
            [lx, -ly],
            [lx, ly],
            [-lx, ly],
        ],
        dtype=np.float64,
    )
    rot = np.array(
        [[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]],
        dtype=np.float64,
    )
    xy = local @ rot.T + np.array(center_xy, dtype=np.float64)
    return np.column_stack([xy, np.full(4, top_z)])


def make_label(class_id, center_xy, yaw_rad, size):
    t_world_camera = get_tf(CAMERA_PATH)
    corners_world = top_face_corners_world(center_xy, yaw_rad, size)
    corners_px = [project_world_to_pixel(p, t_world_camera) for p in corners_world]
    if any(p is None for p in corners_px):
        return None
    corners_px = np.array(corners_px, dtype=np.float64)
    if not ((corners_px[:, 0] >= 1).all() and (corners_px[:, 0] <= WIDTH - 2).all()):
        return None
    if not ((corners_px[:, 1] >= 1).all() and (corners_px[:, 1] <= HEIGHT - 2).all()):
        return None
    norm = corners_px / np.array([WIDTH, HEIGHT], dtype=np.float64)
    values = [str(class_id)] + [f"{x:.6f} {y:.6f}" for x, y in norm]
    return " ".join(values)


def clean_dataset(root):
    for pattern in ("images/train/*.png", "images/val/*.png", "labels/train/*.txt", "labels/val/*.txt"):
        for path in root.glob(pattern):
            path.unlink()
    yaml_path = root / "metal_objects_obb.yaml"
    if yaml_path.exists():
        yaml_path.unlink()


def prepare_dirs(root):
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)


def write_yaml(root):
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


def wait_for_camera_rgba(world, camera):
    for frame_index in range(max(1, ARGS.camera_retry_frames)):
        rgba = camera.get_rgba()
        arr = np.asarray(rgba)
        if arr.size > 0 and arr.ndim >= 3 and arr.shape[0] > 0 and arr.shape[1] > 0:
            return rgba
        world.step(render=True)
        if frame_index == 0:
            print("[dataset] camera buffer empty; waiting for render data...", flush=True)
    raise RuntimeError(f"Camera returned empty RGBA for {ARGS.camera_retry_frames} retry frames.")


def random_metal_params(rng, object_name):
    if ARGS.difficulty == "starter":
        base = rng.uniform(0.78, 0.96) if object_name == "steel_cube" else rng.uniform(0.72, 0.92)
        tint = rng.normal(0.0, 0.018, size=3)
        color = np.clip(np.array([base, base, base]) + tint, 0.55, 0.96)
        roughness = rng.uniform(0.38, 0.72) if object_name == "steel_cube" else rng.uniform(0.25, 0.55)
        return color, rng.uniform(0.70, 1.0), roughness

    base = rng.uniform(0.35, 0.88)
    tint = rng.normal(0.0, 0.035, size=3)
    color = np.clip(np.array([base, base, base]) + tint, 0.18, 0.95)
    metallic = rng.uniform(0.75, 1.0)
    roughness = rng.uniform(0.10, 0.70)
    return color, metallic, roughness


def random_table_params(rng):
    if ARGS.difficulty == "starter":
        color = np.array(
            [
                rng.uniform(0.20, 0.38),
                rng.uniform(0.36, 0.52),
                rng.uniform(0.45, 0.62),
            ],
            dtype=np.float64,
        )
        return color, 0.0, rng.uniform(0.60, 0.95)

    color = np.array(
        [
            rng.uniform(0.30, 0.65),
            rng.uniform(0.35, 0.70),
            rng.uniform(0.40, 0.75),
        ],
        dtype=np.float64,
    )
    return color, rng.uniform(0.0, 0.2), rng.uniform(0.45, 0.95)


def randomize_lighting(rng, light, light_rot_op):
    if ARGS.difficulty == "starter":
        light.GetIntensityAttr().Set(float(rng.uniform(550.0, 1050.0)))
    elif ARGS.difficulty == "balanced":
        light.GetIntensityAttr().Set(float(rng.uniform(450.0, 1400.0)))
    else:
        light.GetIntensityAttr().Set(float(rng.uniform(280.0, 1800.0)))
    light_rot_op.Set(
        Gf.Vec3f(
            float(rng.uniform(10.0, 75.0)),
            float(rng.uniform(-25.0, 25.0)),
            float(rng.uniform(-180.0, 180.0)),
        )
    )


def add_glare_and_noise(bgr, rng):
    img = bgr.astype(np.float32)

    if ARGS.difficulty == "starter":
        alpha = rng.uniform(0.90, 1.12)
        beta = rng.uniform(-8.0, 12.0)
        glare_prob = 0.04
        line_prob = 0.03
        noise_max = 2.0
        blur_prob = 0.03
    elif ARGS.difficulty == "balanced":
        alpha = rng.uniform(0.82, 1.22)
        beta = rng.uniform(-16.0, 22.0)
        glare_prob = 0.16
        line_prob = 0.12
        noise_max = 4.0
        blur_prob = 0.08
    else:
        alpha = rng.uniform(0.72, 1.35)
        beta = rng.uniform(-24.0, 32.0)
        glare_prob = 0.35
        line_prob = 0.35
        noise_max = 7.0
        blur_prob = 0.15

    img = img * alpha + beta

    if rng.random() < glare_prob:
        overlay = np.zeros_like(img)
        center = (int(rng.uniform(0, WIDTH)), int(rng.uniform(0, HEIGHT)))
        axes = (int(rng.uniform(40, 170)), int(rng.uniform(8, 40)))
        angle = float(rng.uniform(0, 180))
        cv2.ellipse(overlay, center, axes, angle, 0, 360, (255, 255, 255), -1)
        overlay = cv2.GaussianBlur(overlay, (0, 0), sigmaX=rng.uniform(12, 35))
        img = img * (1.0 - rng.uniform(0.06, 0.24)) + overlay * rng.uniform(0.18, 0.55)

    if rng.random() < line_prob:
        pt1 = (int(rng.uniform(0, WIDTH)), int(rng.uniform(0, HEIGHT)))
        pt2 = (int(rng.uniform(0, WIDTH)), int(rng.uniform(0, HEIGHT)))
        cv2.line(img, pt1, pt2, (255, 255, 255), int(rng.integers(1, 4)))

    noise_sigma = rng.uniform(0.0, noise_max)
    if noise_sigma > 0.1:
        img += rng.normal(0.0, noise_sigma, size=img.shape)

    img = np.clip(img, 0, 255).astype(np.uint8)
    if rng.random() < blur_prob:
        img = cv2.GaussianBlur(img, (3, 3), 0)
    return img


def target_class_counts(total_count):
    cube_count = int(round(total_count * ARGS.cube_ratio))
    cube_count = max(0, min(total_count, cube_count))
    return {0: total_count - cube_count, 1: cube_count}


def split_class_targets(split_count):
    if ARGS.no_balance_classes:
        return None
    return target_class_counts(split_count)


def choose_object(rng, class_counts, class_targets):
    if ARGS.no_balance_classes or class_targets is None:
        return "steel_cube" if rng.random() < ARGS.cube_ratio else "steel_plate"

    remaining = []
    for name, spec in OBJECT_SPECS.items():
        class_id = spec["class_id"]
        need = class_targets[class_id] - class_counts[class_id]
        if need > 0:
            remaining.append((name, need))
    if not remaining:
        return "steel_cube" if rng.random() < ARGS.cube_ratio else "steel_plate"
    if len(remaining) == 1:
        return remaining[0][0]

    total_need = float(sum(need for _, need in remaining))
    pick = float(rng.uniform(0.0, total_need))
    cursor = 0.0
    for name, need in remaining:
        cursor += need
        if pick <= cursor:
            return name
    return remaining[-1][0]


def sample_cube_size(rng):
    side = float(ARGS.fixed_cube_size)
    return np.array([side, side, side], dtype=np.float64)


def sample_yaw_deg(rng, object_name):
    if object_name == "steel_cube" and not ARGS.cube_yaw_random:
        return 0.0
    return float(rng.uniform(-85.0, 85.0))


def set_object_scale(prim_path, scale):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))
            return
    xform.AddScaleOp().Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))


def set_active_object(active_name, objects, center_xy, yaw_deg, active_size):
    for name, obj in objects.items():
        spec = OBJECT_SPECS[name]
        if name == active_name:
            position = np.array([center_xy[0], center_xy[1], active_size[2] / 2.0])
            if obj is None:
                set_prim_xform(spec["path"], position, yaw_deg, np.full(3, CUBE_USD_WORLD_SCALE))
            else:
                set_object_scale(spec["path"], active_size)
                obj.set_world_pose(
                    position=position,
                    orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, yaw_deg]), degrees=True),
                )
        else:
            if obj is None:
                set_prim_xform(spec["path"], np.array([0.0, 0.0, -5.0]), 0.0, np.full(3, CUBE_USD_WORLD_SCALE))
            else:
                obj.set_world_pose(position=np.array([0.0, 0.0, -5.0]))


def label_polygon_px(label):
    values = label.split()
    coords = np.array([float(v) for v in values[1:]], dtype=np.float64).reshape(4, 2)
    coords *= np.array([WIDTH, HEIGHT], dtype=np.float64)
    return coords.astype(np.int32)


def label_is_visible(bgr, label):
    if ARGS.no_visibility_check:
        return True

    polygon = label_polygon_px(label)
    mask = np.zeros((HEIGHT, WIDTH), np.uint8)
    cv2.fillConvexPoly(mask, polygon, 255)
    area = int(np.count_nonzero(mask))
    if area < 20:
        return False

    x, y, w, h = cv2.boundingRect(polygon)
    pad = 28
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(WIDTH, x + w + pad)
    y1 = min(HEIGHT, y + h + pad)
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
    return contrast >= ARGS.min_visible_contrast or edge_density >= ARGS.min_visible_edge_density


def main():
    rng = np.random.default_rng(ARGS.seed)
    root = Path(ARGS.dataset_dir)
    prepare_dirs(root)
    if ARGS.clean:
        clean_dataset(root)
    print(f"[dataset] output dir: {root}", flush=True)
    print(f"[dataset] target images: {ARGS.count}", flush=True)

    world = None
    try:
        print("[dataset] starting Isaac scene setup...", flush=True)
        world, camera, plate, cube, light, light_rot_op, object_shader, table_shader = setup_scene()
        print("[dataset] Isaac scene ready. generating images...", flush=True)
        objects = {"steel_plate": plate, "steel_cube": cube}

        if ARGS.warmup_frames > 0:
            print(f"[dataset] warming camera for {ARGS.warmup_frames} frames...", flush=True)
            for _ in range(ARGS.warmup_frames):
                world.step(render=True)

        written = 0
        attempts = 0
        max_attempts = max(ARGS.count + 100, ARGS.count * ARGS.max_attempts_multiplier)
        val_count = int(ARGS.count * ARGS.val_ratio)
        split_counts = {
            "val": {0: 0, 1: 0},
            "train": {0: 0, 1: 0},
        }
        split_targets = {
            "val": split_class_targets(val_count),
            "train": split_class_targets(ARGS.count - val_count),
        }
        if split_targets["train"] is not None and split_targets["val"] is not None:
            print(
                "[dataset] target class balance "
                f"train_plate={split_targets['train'][0]} train_cube={split_targets['train'][1]} "
                f"val_plate={split_targets['val'][0]} val_cube={split_targets['val'][1]}",
                flush=True,
            )

        while written < ARGS.count:
            attempts += 1
            if attempts > max_attempts:
                raise RuntimeError(
                    "No more valid projected labels. "
                    f"written={written}, attempts={attempts}, target={ARGS.count}. "
                    "Check camera pose/intrinsics or object sampling bounds."
                )
            split = "val" if written < val_count else "train"
            object_name = choose_object(rng, split_counts[split], split_targets[split])
            spec = OBJECT_SPECS[object_name]
            class_id = spec["class_id"]
            size = sample_cube_size(rng) if object_name == "steel_cube" else spec["size"]

            center_xy = np.array(
                [
                    rng.uniform(0.08, 0.58),
                    rng.uniform(-0.30, 0.30),
                ],
                dtype=np.float64,
            )
            yaw_deg = sample_yaw_deg(rng, object_name)
            yaw_rad = math.radians(float(yaw_deg))

            label = make_label(class_id, center_xy, yaw_rad, size)
            if label is None:
                if attempts <= 5 or attempts % 100 == 0:
                    print(
                        "[dataset] rejected projected label "
                        f"attempt={attempts} object={object_name} "
                        f"center=({center_xy[0]:+.3f},{center_xy[1]:+.3f}) yaw={yaw_deg:+.1f}",
                        flush=True,
                    )
                continue
            if written == 0:
                print(
                    "[dataset] first valid label "
                    f"attempt={attempts} object={object_name} label={label}",
                    flush=True,
                )

            if written == 0:
                print("[dataset] applying object pose, materials, and lighting...", flush=True)
            set_active_object(object_name, objects, center_xy, yaw_deg, size)
            metal_color, metallic, roughness = random_metal_params(rng, object_name)
            table_color, table_metallic, table_roughness = random_table_params(rng)
            set_material(object_shader, metal_color, metallic, roughness)
            set_material(table_shader, table_color, table_metallic, table_roughness)
            randomize_lighting(rng, light, light_rot_op)

            if written == 0:
                print("[dataset] stepping render frames...", flush=True)
            for frame_index in range(max(1, ARGS.frames_per_sample)):
                world.step(render=True)
                if written == 0:
                    print(f"[dataset] render frame {frame_index + 1} ok", flush=True)

            if written == 0:
                print("[dataset] reading camera rgba...", flush=True)
            rgba = wait_for_camera_rgba(world, camera)
            if written == 0:
                print(
                    f"[dataset] camera rgba shape={np.asarray(rgba).shape} dtype={np.asarray(rgba).dtype}",
                    flush=True,
                )
            bgr = add_glare_and_noise(rgba_to_bgr(rgba), rng)
            if not label_is_visible(bgr, label):
                if attempts <= 5 or attempts % 100 == 0:
                    print(
                        "[dataset] rejected low-visibility sample "
                        f"attempt={attempts} object={object_name}",
                        flush=True,
                    )
                continue

            stem = f"{object_name}_{written:06d}"
            image_path = root / "images" / split / f"{stem}.png"
            if written == 0:
                print(f"[dataset] writing image {image_path}", flush=True)
            if not cv2.imwrite(str(image_path), bgr):
                raise RuntimeError(f"Failed to write image: {image_path}")
            (root / "labels" / split / f"{stem}.txt").write_text(label + "\n", encoding="utf-8")

            written += 1
            split_counts[split][class_id] += 1
            if written % 50 == 0 or written == ARGS.count:
                print(f"[dataset] {written}/{ARGS.count} images written", flush=True)

        yaml_path = write_yaml(root)
        print("\n=== YOLO OBB dataset ready ===")
        print(f"dataset:      {root}")
        print(f"yaml:         {yaml_path}")
        print(f"attempts:     {attempts}")
        print(f"train_plate:  {split_counts['train'][0]}")
        print(f"train_cube:   {split_counts['train'][1]}")
        print(f"val_plate:    {split_counts['val'][0]}")
        print(f"val_cube:     {split_counts['val'][1]}")
        if split_targets["train"] is not None and split_targets["val"] is not None:
            print(f"target_train_plate: {split_targets['train'][0]}")
            print(f"target_train_cube:  {split_targets['train'][1]}")
            print(f"target_val_plate:   {split_targets['val'][0]}")
            print(f"target_val_cube:    {split_targets['val'][1]}")
        print(f"cube_size_m:  fixed={ARGS.fixed_cube_size}")
        print("==============================\n")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
