from __future__ import annotations

# Isaac Sim randomized evidence renderer for steel_plate / steel_cube YOLO validation.
#
# Default mode renders randomized lighting/material/glare/noise samples and writes
# GT evidence that can be evaluated from normal Python with
# yolo_confusion_from_samples.py.
#
# It writes:
#   - samples.csv
#   - render_summary.md
#   - images/raw/*.png
#
# Example:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/isaac_yolo_confusion_matrix.py \
#   --count 400 --difficulty hard --background-ratio 0.25 --seed 43 --clean

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import shutil
import sys

from isaacsim import SimulationApp


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--count", type=int, default=400)
    parser.add_argument("--seed", type=int, default=43)
    parser.add_argument("--difficulty", choices=("starter", "balanced", "hard"), default="hard")
    parser.add_argument("--cube-ratio", type=float, default=0.5)
    parser.add_argument(
        "--background-ratio",
        type=float,
        default=0.25,
        help="Fraction of samples rendered with no object, used as negative background training data.",
    )
    parser.add_argument("--no-balance-classes", action="store_true")
    parser.add_argument("--cube-yaw-random", action="store_true")
    parser.add_argument("--fixed-cube-size", type=float, default=0.05)
    parser.add_argument("--model", default=str(root / "vision" / "models" / "yolo11s_obb_metal_hard-v2_refinetune_best.pt"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument(
        "--evaluate-in-isaac",
        action="store_true",
        help="Also run YOLO inside Isaac Python. Usually disabled because Isaac Python may not have ultralytics.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=500.0)
    parser.add_argument("--fy", type=float, default=500.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--camera-z", type=float, default=1.35)
    parser.add_argument("--frames-per-sample", type=int, default=2)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--camera-retry-frames", type=int, default=60)
    parser.add_argument("--max-attempts-multiplier", type=int, default=50)
    parser.add_argument("--min-visible-contrast", type=float, default=8.0)
    parser.add_argument("--min-visible-edge-density", type=float, default=0.012)
    parser.add_argument(
        "--skip-low-visibility",
        action="store_true",
        help="Skip samples whose GT polygon has very low visual contrast/edge density.",
    )
    parser.add_argument(
        "--save-images",
        choices=("none", "failures", "all"),
        default="failures",
        help="Save annotated evidence images.",
    )
    parser.add_argument(
        "--out-dir",
        default=str(root / "work" / "confusion_matrix_hard_v2"),
    )
    parser.add_argument("--clean", action="store_true", help="Remove the output directory before writing new results.")
    return parser.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({"headless": not ARGS.gui})

import cv2
import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

sys.path.append(str(Path(__file__).resolve().parents[2]))
from vision.vision_contracts import ObbDetection, empty_obb  # noqa: E402


CLASS_NAMES = {0: "steel_plate", 1: "steel_cube"}
CLASS_IDS = {"steel_plate": 0, "steel_cube": 1, "background": -1}
TRUE_LABELS = ["steel_plate", "steel_cube", "background"]
PRED_LABELS = ["steel_plate", "steel_cube", "no_detection", "unknown"]

CAMERA_PATH = "/World/top_camera"
PLATE_PATH = "/World/steel_plate"
CUBE_PATH = "/World/steel_cube"
TABLE_PATH = "/World/work_table"
KEY_LIGHT_PATH = "/World/key_light"

PLATE_SIZE = np.array([0.32, 0.14, 0.03], dtype=np.float64)
CUBE_SIZE = np.array([ARGS.fixed_cube_size, ARGS.fixed_cube_size, ARGS.fixed_cube_size], dtype=np.float64)
SIZES = {"steel_plate": PLATE_SIZE, "steel_cube": CUBE_SIZE}
T_GL_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])


@dataclass(frozen=True)
class Visibility:
    visible: bool
    contrast: float
    edge_density: float
    area: int


def log(message: str):
    print(message, flush=True)


def make_preview_surface(stage, material_path):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.65, 0.65, 0.65))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material, shader


def bind_material(stage, prim_path, material):
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def set_material(shader, color, metallic, roughness):
    shader.GetInput("diffuseColor").Set(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    shader.GetInput("metallic").Set(float(metallic))
    shader.GetInput("roughness").Set(float(roughness))


def set_object_scale(prim_path, scale):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))
            return
    xform.AddScaleOp().Set(Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))


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

    world.scene.add(
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
            position=np.array([0.25, 0.0, -5.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
            scale=PLATE_SIZE,
            color=np.array([0.70, 0.70, 0.70]),
        )
    )
    cube = world.scene.add(
        VisualCuboid(
            prim_path=CUBE_PATH,
            name="steel_cube",
            position=np.array([0.25, 0.0, -5.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
            scale=CUBE_SIZE,
            color=np.array([0.70, 0.70, 0.70]),
        )
    )

    object_mat, object_shader = make_preview_surface(stage, "/World/Materials/random_metal")
    table_mat, table_shader = make_preview_surface(stage, "/World/Materials/random_table")
    bind_material(stage, PLATE_PATH, object_mat)
    bind_material(stage, CUBE_PATH, object_mat)
    bind_material(stage, TABLE_PATH, table_mat)

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.35, 0.0, ARGS.camera_z]),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 90.0, 180.0]), degrees=True),
        frequency=10,
        resolution=(ARGS.width, ARGS.height),
    )
    world.reset()
    camera.initialize()
    camera.set_opencv_pinhole_properties(cx=ARGS.cx, cy=ARGS.cy, fx=ARGS.fx, fy=ARGS.fy, pinhole=[0.0] * 12)
    return world, camera, {"steel_plate": plate, "steel_cube": cube}, light, light_rot_op, object_shader, table_shader


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


def wait_for_camera_rgba(world, camera):
    for frame_index in range(max(1, ARGS.camera_retry_frames)):
        rgba = camera.get_rgba()
        arr = np.asarray(rgba)
        if arr.size > 0 and arr.ndim >= 3 and arr.shape[0] > 0 and arr.shape[1] > 0:
            return rgba
        world.step(render=True)
        simulation_app.update()
        if frame_index == 0:
            log("[confusion] camera buffer empty; waiting for render data...")
    raise RuntimeError(f"Camera returned empty RGBA for {ARGS.camera_retry_frames} retry frames.")


def project_world_to_pixel(p_world, t_world_camera):
    p_h = np.array([p_world[0], p_world[1], p_world[2], 1.0], dtype=np.float64)
    p_gl = np.linalg.inv(t_world_camera) @ p_h
    p_cv = T_GL_TO_CV @ p_gl
    if p_cv[2] <= 1e-6:
        return None
    u = ARGS.fx * (p_cv[0] / p_cv[2]) + ARGS.cx
    v = ARGS.fy * (p_cv[1] / p_cv[2]) + ARGS.cy
    return np.array([u, v], dtype=np.float64)


def top_face_corners_world(center_xy, yaw_rad, size):
    lx = size[0] / 2.0
    ly = size[1] / 2.0
    top_z = float(size[2])
    local = np.array([[-lx, -ly], [lx, -ly], [lx, ly], [-lx, ly]], dtype=np.float64)
    rot = np.array(
        [[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]],
        dtype=np.float64,
    )
    xy = local @ rot.T + np.array(center_xy, dtype=np.float64)
    return np.column_stack([xy, np.full(4, top_z)])


def gt_polygon_px(center_xy, yaw_rad, size, t_world_camera):
    corners_world = top_face_corners_world(center_xy, yaw_rad, size)
    corners_px = [project_world_to_pixel(p, t_world_camera) for p in corners_world]
    if any(p is None for p in corners_px):
        return None
    corners_px = np.array(corners_px, dtype=np.float64)
    if not ((corners_px[:, 0] >= 1).all() and (corners_px[:, 0] <= ARGS.width - 2).all()):
        return None
    if not ((corners_px[:, 1] >= 1).all() and (corners_px[:, 1] <= ARGS.height - 2).all()):
        return None
    return corners_px


def visibility_metrics(bgr, polygon):
    polygon_i = np.round(polygon).astype(np.int32)
    mask = np.zeros((ARGS.height, ARGS.width), np.uint8)
    cv2.fillConvexPoly(mask, polygon_i, 255)
    area = int(np.count_nonzero(mask))
    if area < 20:
        return Visibility(False, 0.0, 0.0, area)

    x, y, w, h = cv2.boundingRect(polygon_i)
    pad = 28
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(ARGS.width, x + w + pad)
    y1 = min(ARGS.height, y + h + pad)
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
    visible = contrast >= ARGS.min_visible_contrast or edge_density >= ARGS.min_visible_edge_density
    return Visibility(visible, contrast, edge_density, area)


def random_metal_params(rng, object_name):
    if ARGS.difficulty == "starter":
        base = rng.uniform(0.78, 0.96) if object_name == "steel_cube" else rng.uniform(0.72, 0.92)
        tint = rng.normal(0.0, 0.018, size=3)
        color = np.clip(np.array([base, base, base]) + tint, 0.55, 0.96)
        roughness = rng.uniform(0.38, 0.72) if object_name == "steel_cube" else rng.uniform(0.25, 0.55)
        return color, float(rng.uniform(0.70, 1.0)), float(roughness)

    base = rng.uniform(0.35, 0.88)
    tint = rng.normal(0.0, 0.035, size=3)
    color = np.clip(np.array([base, base, base]) + tint, 0.18, 0.95)
    return color, float(rng.uniform(0.75, 1.0)), float(rng.uniform(0.10, 0.70))


def random_table_params(rng):
    if ARGS.difficulty == "starter":
        color = np.array([rng.uniform(0.20, 0.38), rng.uniform(0.36, 0.52), rng.uniform(0.45, 0.62)])
        return color, 0.0, float(rng.uniform(0.60, 0.95))

    color = np.array([rng.uniform(0.30, 0.65), rng.uniform(0.35, 0.70), rng.uniform(0.40, 0.75)])
    return color, float(rng.uniform(0.0, 0.2)), float(rng.uniform(0.45, 0.95))


def randomize_lighting(rng, light, light_rot_op):
    if ARGS.difficulty == "starter":
        intensity = float(rng.uniform(550.0, 1050.0))
    elif ARGS.difficulty == "balanced":
        intensity = float(rng.uniform(450.0, 1400.0))
    else:
        intensity = float(rng.uniform(280.0, 1800.0))
    rotation = np.array(
        [rng.uniform(10.0, 75.0), rng.uniform(-25.0, 25.0), rng.uniform(-180.0, 180.0)],
        dtype=np.float64,
    )
    light.GetIntensityAttr().Set(intensity)
    light_rot_op.Set(Gf.Vec3f(float(rotation[0]), float(rotation[1]), float(rotation[2])))
    return intensity, rotation


def add_glare_and_noise(bgr, rng):
    img = bgr.astype(np.float32)
    params = {
        "alpha": 1.0,
        "beta": 0.0,
        "glare": 0,
        "line_glare": 0,
        "noise_sigma": 0.0,
        "blur": 0,
    }

    if ARGS.difficulty == "starter":
        alpha = float(rng.uniform(0.90, 1.12))
        beta = float(rng.uniform(-8.0, 12.0))
        glare_prob = 0.04
        line_prob = 0.03
        noise_max = 2.0
        blur_prob = 0.03
    elif ARGS.difficulty == "balanced":
        alpha = float(rng.uniform(0.82, 1.22))
        beta = float(rng.uniform(-16.0, 22.0))
        glare_prob = 0.16
        line_prob = 0.12
        noise_max = 4.0
        blur_prob = 0.08
    else:
        alpha = float(rng.uniform(0.72, 1.35))
        beta = float(rng.uniform(-24.0, 32.0))
        glare_prob = 0.35
        line_prob = 0.35
        noise_max = 7.0
        blur_prob = 0.15

    params["alpha"] = alpha
    params["beta"] = beta
    img = img * alpha + beta

    if rng.random() < glare_prob:
        params["glare"] = 1
        overlay = np.zeros_like(img)
        center = (int(rng.uniform(0, ARGS.width)), int(rng.uniform(0, ARGS.height)))
        axes = (int(rng.uniform(40, 170)), int(rng.uniform(8, 40)))
        angle = float(rng.uniform(0, 180))
        cv2.ellipse(overlay, center, axes, angle, 0, 360, (255, 255, 255), -1)
        overlay = cv2.GaussianBlur(overlay, (0, 0), sigmaX=float(rng.uniform(12, 35)))
        img = img * (1.0 - float(rng.uniform(0.06, 0.24))) + overlay * float(rng.uniform(0.18, 0.55))

    if rng.random() < line_prob:
        params["line_glare"] = 1
        pt1 = (int(rng.uniform(0, ARGS.width)), int(rng.uniform(0, ARGS.height)))
        pt2 = (int(rng.uniform(0, ARGS.width)), int(rng.uniform(0, ARGS.height)))
        cv2.line(img, pt1, pt2, (255, 255, 255), int(rng.integers(1, 4)))

    noise_sigma = float(rng.uniform(0.0, noise_max))
    params["noise_sigma"] = noise_sigma
    if noise_sigma > 0.1:
        img += rng.normal(0.0, noise_sigma, size=img.shape)

    img = np.clip(img, 0, 255).astype(np.uint8)
    if rng.random() < blur_prob:
        params["blur"] = 1
        img = cv2.GaussianBlur(img, (3, 3), 0)
    return img, params


def set_active_object(active_name, objects, center_xy, yaw_deg):
    for name, obj in objects.items():
        size = SIZES[name]
        if name == active_name:
            set_object_scale(PLATE_PATH if name == "steel_plate" else CUBE_PATH, size)
            obj.set_world_pose(
                position=np.array([center_xy[0], center_xy[1], size[2] / 2.0]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, yaw_deg]), degrees=True),
            )
        else:
            obj.set_world_pose(position=np.array([0.0, 0.0, -5.0]))


def sample_yaw_deg(rng, object_name):
    if object_name == "steel_cube" and not ARGS.cube_yaw_random:
        return 0.0
    return float(rng.uniform(-85.0, 85.0))


def to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def corners_from_xywhr(cx, cy, width, height, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    local = np.array([[-width / 2.0, -height / 2.0], [width / 2.0, -height / 2.0], [width / 2.0, height / 2.0], [-width / 2.0, height / 2.0]])
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    pts = local @ rot.T + np.array([cx, cy], dtype=np.float64)
    return tuple((float(x), float(y)) for x, y in pts)


def best_obb_detection(result):
    obb = getattr(result, "obb", None)
    if obb is None or getattr(obb, "xywhr", None) is None:
        return empty_obb()

    xywhr = to_numpy(obb.xywhr)
    if xywhr.size == 0:
        return empty_obb()

    conf = to_numpy(obb.conf) if getattr(obb, "conf", None) is not None else np.ones(len(xywhr))
    cls = to_numpy(obb.cls) if getattr(obb, "cls", None) is not None else np.zeros(len(xywhr))
    best = int(np.argmax(conf))
    cx, cy, width, height, angle = xywhr[best].astype(float)

    if getattr(obb, "xyxyxyxy", None) is not None:
        corners_arr = to_numpy(obb.xyxyxyxy[best]).reshape(4, 2)
        corners = tuple((float(x), float(y)) for x, y in corners_arr)
    else:
        corners = corners_from_xywhr(cx, cy, width, height, angle)

    return ObbDetection(
        valid=True,
        class_id=int(cls[best]),
        confidence=float(conf[best]),
        center_px=(float(cx), float(cy)),
        corners_px=corners,
        width_px=float(width),
        height_px=float(height),
        angle_rad=float(angle),
    )


def load_yolo_model():
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "Isaac Sim Python cannot import ultralytics. Install ultralytics for Isaac Python, "
            "or run this validation in the Python environment where YOLO is available."
        ) from exc
    return YOLO(ARGS.model)


def infer(model, bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    results = model.predict(source=rgb, imgsz=ARGS.imgsz, conf=ARGS.conf, device=ARGS.device, verbose=False)
    return best_obb_detection(results[0]) if results else empty_obb()


def pred_name_from_det(det: ObbDetection):
    if not det.valid:
        return "no_detection"
    return CLASS_NAMES.get(det.class_id, "unknown")


def target_sequence(rng):
    if ARGS.no_balance_classes:
        return None
    background_count = int(round(ARGS.count * ARGS.background_ratio))
    background_count = max(0, min(ARGS.count, background_count))
    object_count = ARGS.count - background_count
    cube_count = int(round(object_count * ARGS.cube_ratio))
    cube_count = max(0, min(object_count, cube_count))
    names = ["background"] * background_count + ["steel_cube"] * cube_count + ["steel_plate"] * (object_count - cube_count)
    rng.shuffle(names)
    return names


def choose_object(rng, targets, index):
    if targets is not None:
        return targets[index]
    if rng.random() < ARGS.background_ratio:
        return "background"
    return "steel_cube" if rng.random() < ARGS.cube_ratio else "steel_plate"


def prepare_output():
    out_dir = Path(ARGS.out_dir)
    if ARGS.clean and out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "images").mkdir(exist_ok=True)
    (out_dir / "images" / "raw").mkdir(parents=True, exist_ok=True)
    return out_dir


def draw_debug(bgr, gt_polygon, det, sample_id, true_name, pred_name, correct, visibility):
    out = bgr.copy()
    if len(gt_polygon):
        gt = np.round(gt_polygon).astype(np.int32)
        cv2.polylines(out, [gt], True, (0, 255, 0), 2)
        cv2.putText(out, f"GT {true_name}", tuple(gt[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
    else:
        cv2.putText(out, "GT background", (16, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)

    if det.valid:
        pred = np.array(det.corners_px, dtype=np.int32)
        cv2.polylines(out, [pred], True, (0, 255, 255), 2)
        cv2.circle(out, tuple(np.round(det.center_px).astype(int)), 4, (0, 0, 255), -1)

    status = "OK" if correct else "FAIL"
    color = (0, 180, 0) if correct else (0, 0, 255)
    cv2.rectangle(out, (8, 8), (ARGS.width - 8, 78), (20, 20, 20), -1)
    cv2.putText(out, f"{sample_id:04d} {status} true={true_name} pred={pred_name}", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.68, color, 2)
    cv2.putText(
        out,
        f"conf={det.confidence:.2f} visible={int(visibility.visible)} contrast={visibility.contrast:.1f} edge={visibility.edge_density:.3f}",
        (16, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        2,
    )
    return out


def write_samples_csv(out_dir, rows):
    if not rows:
        return None
    path = out_dir / "samples.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_confusion_csv(out_dir, matrix):
    path = out_dir / "confusion_matrix.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["true\\pred"] + PRED_LABELS)
        for row_label, values in zip(TRUE_LABELS, matrix):
            writer.writerow([row_label] + [int(v) for v in values])
    return path


def write_confusion_png(out_dir, matrix):
    cell_w = 150
    cell_h = 82
    left = 150
    top = 90
    width = left + cell_w * len(PRED_LABELS) + 32
    height = top + cell_h * len(TRUE_LABELS) + 110
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.putText(canvas, "YOLO OBB Confusion Matrix", (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (30, 30, 30), 2)
    cv2.putText(canvas, f"difficulty={ARGS.difficulty} count={int(matrix.sum())}", (24, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1)

    for col, label in enumerate(PRED_LABELS):
        x = left + col * cell_w + 10
        cv2.putText(canvas, label, (x, top - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1)
    cv2.putText(canvas, "true", (24, top + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)
    cv2.putText(canvas, "predicted", (left + 5, top - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)

    max_value = max(1, int(matrix.max()))
    for row, row_label in enumerate(TRUE_LABELS):
        y = top + row * cell_h
        cv2.putText(canvas, row_label, (18, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 50, 50), 1)
        row_total = max(1, int(matrix[row].sum()))
        for col, _ in enumerate(PRED_LABELS):
            x = left + col * cell_w
            value = int(matrix[row, col])
            intensity = int(245 - 105 * (value / max_value))
            if col == row:
                fill = (intensity, 245, intensity)
            elif value > 0:
                fill = (intensity, intensity, 245)
            else:
                fill = (245, 245, 245)
            cv2.rectangle(canvas, (x, y), (x + cell_w, y + cell_h), fill, -1)
            cv2.rectangle(canvas, (x, y), (x + cell_w, y + cell_h), (170, 170, 170), 1)
            cv2.putText(canvas, str(value), (x + 54, y + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (20, 20, 20), 2)
            cv2.putText(canvas, f"{100.0 * value / row_total:.1f}%", (x + 48, y + 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)

    background_row = TRUE_LABELS.index("background") if "background" in TRUE_LABELS else None
    no_detection_col = PRED_LABELS.index("no_detection")
    correct = int(matrix[0, 0] + matrix[1, 1])
    if background_row is not None:
        correct += int(matrix[background_row, no_detection_col])
    accuracy = float(correct / max(1, matrix.sum()))
    cv2.putText(canvas, f"overall accuracy: {accuracy * 100.0:.2f}%", (24, height - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (30, 30, 30), 2)
    path = out_dir / "confusion_matrix.png"
    cv2.imwrite(str(path), canvas)
    return path


def write_summary(out_dir, matrix, rows, csv_path, matrix_csv_path, matrix_png_path):
    total = int(matrix.sum())
    correct = int(matrix[0, 0] + matrix[1, 1])
    background_row = TRUE_LABELS.index("background") if "background" in TRUE_LABELS else None
    no_detection_col = PRED_LABELS.index("no_detection")
    if background_row is not None:
        correct += int(matrix[background_row, no_detection_col])
    accuracy = correct / max(1, total)
    lines = [
        "# YOLO OBB Confusion Matrix Summary",
        "",
        f"- difficulty: `{ARGS.difficulty}`",
        f"- seed: `{ARGS.seed}`",
        f"- count: `{total}`",
        f"- correct: `{correct}`",
        f"- accuracy: `{accuracy * 100.0:.2f}%`",
        f"- model: `{ARGS.model}`",
        f"- samples_csv: `{csv_path}`",
        f"- confusion_csv: `{matrix_csv_path}`",
        f"- confusion_png: `{matrix_png_path}`",
        "",
        "| true class | total | recall | correct | no_detection | unknown |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row_idx, label in enumerate(TRUE_LABELS):
        row_total = int(matrix[row_idx].sum())
        correct_col = PRED_LABELS.index("no_detection") if label == "background" else row_idx
        recall = float(matrix[row_idx, correct_col] / max(1, row_total))
        lines.append(
            f"| `{label}` | {row_total} | {recall * 100.0:.2f}% | {int(matrix[row_idx, correct_col])} | "
            f"{int(matrix[row_idx, PRED_LABELS.index('no_detection')])} | {int(matrix[row_idx, PRED_LABELS.index('unknown')])} |"
        )

    if rows:
        low_visible = sum(1 for row in rows if int(row["visible"]) == 0)
        glare = sum(1 for row in rows if int(row["glare"]) == 1)
        line_glare = sum(1 for row in rows if int(row["line_glare"]) == 1)
        lines.extend(
            [
                "",
                "## Sample Conditions",
                "",
                f"- low_visibility_samples: `{low_visible}`",
                f"- ellipse_glare_samples: `{glare}`",
                f"- line_glare_samples: `{line_glare}`",
            ]
        )

    path = out_dir / "summary.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def write_render_summary(out_dir, rows, csv_path):
    by_class = {label: 0 for label in TRUE_LABELS}
    low_visible = 0
    glare = 0
    line_glare = 0
    for row in rows:
        by_class[row["true_class"]] = by_class.get(row["true_class"], 0) + 1
        low_visible += 1 if int(row["visible"]) == 0 else 0
        glare += 1 if int(row["glare"]) == 1 else 0
        line_glare += 1 if int(row["line_glare"]) == 1 else 0

    lines = [
        "# YOLO OBB Randomized Evidence Summary",
        "",
        f"- difficulty: `{ARGS.difficulty}`",
        f"- seed: `{ARGS.seed}`",
        f"- count: `{len(rows)}`",
        f"- samples_csv: `{csv_path}`",
        f"- raw_images: `{out_dir / 'images' / 'raw'}`",
        "",
        "| true class | count |",
        "| --- | ---: |",
    ]
    for label in TRUE_LABELS:
        lines.append(f"| `{label}` | {by_class.get(label, 0)} |")
    lines.extend(
        [
            "",
            "## Sample Conditions",
            "",
            f"- low_visibility_samples: `{low_visible}`",
            f"- ellipse_glare_samples: `{glare}`",
            f"- line_glare_samples: `{line_glare}`",
            "",
            "Evaluate these samples with `yolo_confusion_from_samples.py` from normal Python.",
        ]
    )

    path = out_dir / "render_summary.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main():
    rng = np.random.default_rng(ARGS.seed)
    out_dir = prepare_output()
    targets = target_sequence(rng)
    matrix = np.zeros((len(TRUE_LABELS), len(PRED_LABELS)), dtype=np.int64)
    rows = []
    world = None

    try:
        model = None
        if ARGS.evaluate_in_isaac:
            log("[confusion] loading YOLO model inside Isaac Python...")
            model = load_yolo_model()
        else:
            log("[confusion] rendering GT evidence only; run yolo_confusion_from_samples.py next.")
        log("[confusion] setting up Isaac scene...")
        world, camera, objects, light, light_rot_op, object_shader, table_shader = setup_scene()
        for _ in range(max(1, ARGS.warmup_frames)):
            world.step(render=True)
            simulation_app.update()
        log("[confusion] scene ready. collecting randomized evidence samples...")

        t_world_camera = get_tf(CAMERA_PATH)
        sample_index = 0
        attempts = 0
        max_attempts = max(ARGS.count + 100, ARGS.count * ARGS.max_attempts_multiplier)

        while sample_index < ARGS.count:
            attempts += 1
            if attempts > max_attempts:
                raise RuntimeError(f"Too many rejected samples: sample={sample_index}, attempts={attempts}")

            true_name = choose_object(rng, targets, sample_index)
            true_id = CLASS_IDS[true_name]
            is_background = true_name == "background"
            if is_background:
                size = np.zeros(3, dtype=np.float64)
                center_xy = np.array([0.0, 0.0], dtype=np.float64)
                yaw_deg = 0.0
                polygon = np.zeros((0, 2), dtype=np.float64)
            else:
                size = SIZES[true_name]
                center_xy = np.array([rng.uniform(0.08, 0.58), rng.uniform(-0.30, 0.30)], dtype=np.float64)
                yaw_deg = sample_yaw_deg(rng, true_name)
                yaw_rad = math.radians(yaw_deg)
                polygon = gt_polygon_px(center_xy, yaw_rad, size, t_world_camera)
                if polygon is None:
                    continue

            metal_color, metallic, roughness = random_metal_params(rng, true_name)
            table_color, table_metallic, table_roughness = random_table_params(rng)
            set_material(object_shader, metal_color, metallic, roughness)
            set_material(table_shader, table_color, table_metallic, table_roughness)
            light_intensity, light_rotation = randomize_lighting(rng, light, light_rot_op)
            set_active_object(true_name, objects, center_xy, yaw_deg)

            for _ in range(max(1, ARGS.frames_per_sample)):
                world.step(render=True)
                simulation_app.update()

            bgr = rgba_to_bgr(wait_for_camera_rgba(world, camera))
            bgr, image_params = add_glare_and_noise(bgr, rng)
            visibility = Visibility(True, 0.0, 0.0, 0) if is_background else visibility_metrics(bgr, polygon)
            if not is_background and ARGS.skip_low_visibility and not visibility.visible:
                continue

            sample_id = f"{sample_index:04d}"
            raw_image_path = str(out_dir / "images" / "raw" / f"{sample_id}_{true_name}.png")
            cv2.imwrite(raw_image_path, bgr)

            det = empty_obb()
            pred_name = ""
            correct = ""
            if model is not None:
                det = infer(model, bgr)
                pred_name = pred_name_from_det(det)
                pred_col = PRED_LABELS.index(pred_name) if pred_name in PRED_LABELS else PRED_LABELS.index("unknown")
                true_row = TRUE_LABELS.index(true_name)
                correct = pred_name == "no_detection" if is_background else pred_name == true_name
                matrix[true_row, pred_col] += 1

            image_path = ""
            if model is not None and (ARGS.save_images == "all" or (ARGS.save_images == "failures" and not correct)):
                debug = draw_debug(bgr, polygon, det, sample_index, true_name, pred_name, correct, visibility)
                image_path = str(out_dir / "images" / f"{sample_id}_{true_name}_as_{pred_name}.png")
                cv2.imwrite(image_path, debug)

            rows.append(
                {
                    "sample": sample_index,
                    "attempt": attempts,
                    "true_class": true_name,
                    "true_class_id": true_id,
                    "pred_class": pred_name,
                    "pred_class_id": det.class_id if det.valid else -1,
                    "correct": "" if correct == "" else int(correct),
                    "confidence": f"{det.confidence:.6f}",
                    "visible": int(visibility.visible),
                    "visibility_contrast": f"{visibility.contrast:.6f}",
                    "visibility_edge_density": f"{visibility.edge_density:.6f}",
                    "visibility_area": visibility.area,
                    "raw_image": raw_image_path,
                    "gt_x0": "" if is_background else f"{polygon[0, 0]:.6f}",
                    "gt_y0": "" if is_background else f"{polygon[0, 1]:.6f}",
                    "gt_x1": "" if is_background else f"{polygon[1, 0]:.6f}",
                    "gt_y1": "" if is_background else f"{polygon[1, 1]:.6f}",
                    "gt_x2": "" if is_background else f"{polygon[2, 0]:.6f}",
                    "gt_y2": "" if is_background else f"{polygon[2, 1]:.6f}",
                    "gt_x3": "" if is_background else f"{polygon[3, 0]:.6f}",
                    "gt_y3": "" if is_background else f"{polygon[3, 1]:.6f}",
                    "center_x": f"{center_xy[0]:.6f}",
                    "center_y": f"{center_xy[1]:.6f}",
                    "yaw_deg": f"{yaw_deg:.6f}",
                    "metal_r": f"{metal_color[0]:.6f}",
                    "metal_g": f"{metal_color[1]:.6f}",
                    "metal_b": f"{metal_color[2]:.6f}",
                    "metallic": f"{metallic:.6f}",
                    "roughness": f"{roughness:.6f}",
                    "table_r": f"{table_color[0]:.6f}",
                    "table_g": f"{table_color[1]:.6f}",
                    "table_b": f"{table_color[2]:.6f}",
                    "table_metallic": f"{table_metallic:.6f}",
                    "table_roughness": f"{table_roughness:.6f}",
                    "light_intensity": f"{light_intensity:.6f}",
                    "light_rot_x": f"{light_rotation[0]:.6f}",
                    "light_rot_y": f"{light_rotation[1]:.6f}",
                    "light_rot_z": f"{light_rotation[2]:.6f}",
                    "alpha": f"{image_params['alpha']:.6f}",
                    "beta": f"{image_params['beta']:.6f}",
                    "glare": image_params["glare"],
                    "line_glare": image_params["line_glare"],
                    "noise_sigma": f"{image_params['noise_sigma']:.6f}",
                    "blur": image_params["blur"],
                    "debug_image": image_path,
                }
            )

            sample_index += 1
            if sample_index % 10 == 0 or sample_index == ARGS.count:
                if model is not None:
                    correct_count = int(matrix[0, 0] + matrix[1, 1])
                    correct_count += int(matrix[TRUE_LABELS.index("background"), PRED_LABELS.index("no_detection")])
                    log(f"[confusion] {sample_index}/{ARGS.count} samples, accuracy={correct_count / max(1, sample_index) * 100.0:.2f}%")
                else:
                    log(f"[confusion] {sample_index}/{ARGS.count} rendered samples")

        samples_csv = write_samples_csv(out_dir, rows)
        if model is not None:
            matrix_csv = write_confusion_csv(out_dir, matrix)
            matrix_png = write_confusion_png(out_dir, matrix)
            summary = write_summary(out_dir, matrix, rows, samples_csv, matrix_csv, matrix_png)
        else:
            matrix_csv = None
            matrix_png = None
            summary = write_render_summary(out_dir, rows, samples_csv)

        if model is None:
            log("\n=== YOLO OBB randomized evidence ready ===")
            log(f"out_dir:      {out_dir}")
            log(f"samples_csv:  {samples_csv}")
            log(f"summary:      {summary}")
            log("next command:")
            log(
                "python3 tools/training/yolo_confusion_from_samples.py "
                f"--samples-csv {samples_csv} --save-images failures --device auto"
            )
            log("==========================================\n")
        else:
            log("\n=== YOLO OBB randomized confusion matrix ===")
            log(f"out_dir:      {out_dir}")
            log(f"samples_csv:  {samples_csv}")
            log(f"matrix_csv:   {matrix_csv}")
            log(f"matrix_png:   {matrix_png}")
            log(f"summary:      {summary}")
            log(f"matrix:\n{matrix}")
            log("===========================================\n")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
