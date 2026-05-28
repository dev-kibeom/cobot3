from __future__ import annotations

# Isaac Sim smoke test for:
# 1) trained YOLO11s-OBB inference
# 2) pixel -> world calibration through alignment_brain.py
# 3) optional ROS2/rqt debug publishing
#
# GUI scene test, no YOLO import inside Isaac Python:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/isaac_yolo_calibration_smoke.py --gui --object both --scene-only
#
# Publish Isaac camera for rqt/external YOLO:
# /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#   tools/isaac/isaac_yolo_calibration_smoke.py --gui --ros --ros-image-only

import argparse
import math
import os
from pathlib import Path
import sys
import time

from isaacsim import SimulationApp


def log(message):
    print(message, flush=True)


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--object", choices=("steel_plate", "steel_cube", "both"), default="both")
    parser.add_argument("--scene-only", action="store_true", help="Show the Isaac scene only. Do not import or run YOLO.")
    parser.add_argument("--keep-open", action="store_true", help="Keep Isaac Sim running until the user closes it.")
    parser.add_argument("--model", default=str(root / "vision" / "models" / "yolo11s_obb_metal_hard-v2_refinetune_best.pt"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--camera-path", default="/World/top_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=500.0)
    parser.add_argument("--fy", type=float, default=500.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--camera-z", type=float, default=1.35)
    parser.add_argument("--object-x", type=float, default=0.35)
    parser.add_argument("--object-y", type=float, default=-0.08)
    parser.add_argument("--object-yaw-deg", type=float, default=35.0)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--out", default=str(root / "work" / "yolo_calibration_smoke.png"))
    parser.add_argument("--ros", action="store_true")
    parser.add_argument(
        "--ros-image-only",
        action="store_true",
        help="Publish only Isaac camera images. Run yolo11s_obb_eye_node.py in normal Python.",
    )
    parser.add_argument("--ros-cell", default="global")
    parser.add_argument("--ros-rate", type=float, default=2.0)
    parser.add_argument("--ros-frames", type=int, default=0, help="0 means publish until interrupted.")
    parser.add_argument(
        "--exit-after-ros-frames",
        action="store_true",
        help="Allow --ros-frames to close a GUI run. By default GUI ROS runs stay open.",
    )
    parser.add_argument(
        "--publish-empty-obb",
        action="store_true",
        help="When --ros-image-only is set, also publish invalid empty OBB messages for legacy smoke tests.",
    )
    args = parser.parse_args()
    if args.gui and args.ros and args.ros_frames != 0 and not args.exit_after_ros_frames:
        log(f"[INFO] GUI ROS run ignores --ros-frames={args.ros_frames}; press Ctrl+C to stop.")
        args.ros_frames = 0
    return args


ARGS = parse_args()
log(
    "[isaac_yolo_calibration_smoke] "
    f"argv={sys.argv} gui={ARGS.gui} ros={ARGS.ros} ros_image_only={ARGS.ros_image_only} "
    f"ros_frames={ARGS.ros_frames}"
)
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
from vision.alignment_brain import BrainConfig, CameraModel, obb_to_plate_pose, T_CV_TO_GL  # noqa: E402
from vision.vision_contracts import ObbDetection, empty_obb, encode_obb  # noqa: E402


CLASS_NAMES = {0: "steel_plate", 1: "steel_cube"}
CLASS_IDS = {"steel_plate": 0, "steel_cube": 1}
SIZES = {
    "steel_plate": np.array([0.32, 0.14, 0.03], dtype=np.float64),
    "steel_cube": np.array([0.05, 0.05, 0.05], dtype=np.float64),
}

DEFAULT_OBJECT_POSES = {
    "steel_plate": (0.28, -0.10, 35.0),
    "steel_cube": (0.47, 0.13, 0.0),
}


def get_tf(path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {path}")
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


def to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def corners_from_xywhr(cx, cy, width, height, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    local = np.array(
        [
            [-width / 2.0, -height / 2.0],
            [width / 2.0, -height / 2.0],
            [width / 2.0, height / 2.0],
            [-width / 2.0, height / 2.0],
        ],
        dtype=np.float64,
    )
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


def project_world_to_pixel(point_world, t_world_camera):
    p_world = np.array([point_world[0], point_world[1], point_world[2], 1.0], dtype=np.float64)
    p_gl = np.linalg.inv(t_world_camera) @ p_world
    p_cv = T_CV_TO_GL @ p_gl
    if p_cv[2] <= 1e-9:
        return None
    u = ARGS.fx * (p_cv[0] / p_cv[2]) + ARGS.cx
    v = ARGS.fy * (p_cv[1] / p_cv[2]) + ARGS.cy
    return np.array([u, v], dtype=np.float64)


def make_preview_surface(stage, material_path, color, metallic=1.0, roughness=0.55):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(stage, prim_path, material):
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def active_object_names():
    return ["steel_plate", "steel_cube"] if ARGS.object == "both" else [ARGS.object]


def object_pose(name):
    if ARGS.object == "both":
        return DEFAULT_OBJECT_POSES[name]
    return ARGS.object_x, ARGS.object_y, ARGS.object_yaw_deg


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    stage.DefinePrim("/World/Materials", "Scope")

    light = UsdLux.DistantLight.Define(stage, "/World/yolo_test_light")
    light.CreateIntensityAttr(850.0)
    light_xform = UsdGeom.Xformable(light.GetPrim())
    light_xform.ClearXformOpOrder()
    light_xform.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 20.0))

    table_z = -0.004
    world.scene.add(
        VisualCuboid(
            prim_path="/World/work_table",
            name="work_table",
            position=np.array([0.35, 0.0, table_z]),
            scale=np.array([0.80, 0.60, 0.008]),
            color=np.array([0.32, 0.53, 0.60]),
        )
    )

    metal = make_preview_surface(stage, "/World/Materials/smoke_metal", np.array([0.82, 0.82, 0.78]), 1.0, 0.55)
    for name in active_object_names():
        size = SIZES[name]
        object_path = f"/World/{name}"
        object_x, object_y, yaw_deg = object_pose(name)
        object_z = float(size[2] / 2.0)
        world.scene.add(
            VisualCuboid(
                prim_path=object_path,
                name=name,
                position=np.array([object_x, object_y, object_z]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, yaw_deg]), degrees=True),
                scale=size,
                color=np.array([0.86, 0.86, 0.82]),
            )
        )
        bind_material(stage, object_path, metal)

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


def render_until_interrupted(world, label):
    log(label)
    try:
        while True:
            world.step(render=True)
            simulation_app.update()
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass


def camera_bgr_or_none(camera):
    try:
        return rgba_to_bgr(camera.get_rgba()), None
    except Exception as exc:
        return None, exc


def prefer_isaac_ros2_python():
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    if ros_distro not in ("humble", "jazzy"):
        ros_distro = "humble"

    executable = Path(sys.executable)
    release_roots = [executable.parents[3], executable.resolve().parents[3]]
    bridge_root = None
    rclpy_path = None
    lib_path = None
    for release_root in release_roots:
        candidate_bridge = release_root / "exts" / "isaacsim.ros2.bridge"
        candidate_rclpy = candidate_bridge / ros_distro / "rclpy"
        if candidate_rclpy.exists():
            bridge_root = candidate_bridge
            rclpy_path = candidate_rclpy
            lib_path = candidate_bridge / ros_distro / "lib"
            break

    if rclpy_path is None or bridge_root is None:
        checked = ", ".join(str(root / "exts" / "isaacsim.ros2.bridge" / ros_distro / "rclpy") for root in release_roots)
        log(f"[WARN] Isaac ROS2 bridge rclpy path not found. Checked: {checked}")
        return

    sys.path.insert(0, str(rclpy_path))
    os.environ.setdefault("ROS_DISTRO", ros_distro)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if lib_path.exists():
        current = os.environ.get("LD_LIBRARY_PATH", "")
        lib_text = str(lib_path)
        if lib_text not in current.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{current}:{lib_text}" if current else lib_text
    log(f"[isaac_yolo_calibration_smoke] using Isaac ROS2 {ros_distro} Python bindings")


def infer(model, bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    results = model.predict(source=rgb, imgsz=ARGS.imgsz, conf=ARGS.conf, device=ARGS.device, verbose=False)
    return best_obb_detection(results[0]) if results else empty_obb()


def load_yolo_model():
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "Isaac Sim Python cannot import ultralytics. Use '--gui --ros --ros-image-only' "
            "to publish Isaac camera images, then run visions/yolo11s_obb_eye_node.py from normal Python "
            "in another terminal."
        ) from exc
    return YOLO(ARGS.model)


def draw_debug(bgr, det, gt_pixel, pred_pose=None, error_mm=None):
    out = bgr.copy()
    if gt_pixel is not None:
        cv2.drawMarker(out, tuple(np.round(gt_pixel).astype(int)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(out, "GT", tuple(np.round(gt_pixel + np.array([8.0, -8.0])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if det.valid:
        pts = np.array(det.corners_px, dtype=np.int32)
        label = CLASS_NAMES.get(det.class_id, str(det.class_id))
        cv2.polylines(out, [pts], True, (0, 255, 255), 2)
        cv2.circle(out, tuple(np.round(det.center_px).astype(int)), 4, (0, 0, 255), -1)
        text = f"{label} conf={det.confidence:.2f}"
        if error_mm is not None:
            text += f" err={error_mm:.1f}mm"
        cv2.putText(out, text, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2)
        if pred_pose is not None:
            cv2.putText(out, f"pick=({pred_pose.x:+.3f},{pred_pose.y:+.3f},{pred_pose.z:+.3f})", (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
    else:
        cv2.putText(out, "no detection", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return out


def ground_truth_pixels(t_world_camera):
    pixels = {}
    for name in active_object_names():
        object_x, object_y, _ = object_pose(name)
        top_z = float(SIZES[name][2])
        pixels[name] = project_world_to_pixel(np.array([object_x, object_y, top_z], dtype=np.float64), t_world_camera)
    return pixels


def draw_scene_markers(bgr, gt_pixels):
    out = bgr.copy()
    for name, pixel in gt_pixels.items():
        if pixel is None:
            continue
        cv2.drawMarker(out, tuple(np.round(pixel).astype(int)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(out, name, tuple(np.round(pixel + np.array([8.0, -8.0])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return out


def publish_ros(world, camera, model, camera_model, top_z, gt_pixel, gt_pixels):
    node = None
    rclpy = None
    try:
        log("[isaac_yolo_calibration_smoke] importing ROS modules")
        prefer_isaac_ros2_python()
        import rclpy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32MultiArray

        log("[isaac_yolo_calibration_smoke] initializing rclpy")
        rclpy.init()
        node = rclpy.create_node("isaac_yolo_calibration_smoke")
        bridge = CvBridge()
        raw_pub = node.create_publisher(Image, f"/{ARGS.ros_cell}/top_camera/image", 10)
        debug_pub = node.create_publisher(Image, f"/{ARGS.ros_cell}/vision/debug_image", 10)
        obb_pub = None
        if not ARGS.ros_image_only or ARGS.publish_empty_obb:
            obb_pub = node.create_publisher(Float32MultiArray, f"/{ARGS.ros_cell}/vision/plate_obb", 10)
    except BaseException as exc:
        log(f"[ERROR] failed before ROS publish loop: {type(exc).__name__}: {exc}")
        if ARGS.gui:
            render_until_interrupted(world, "Keeping Isaac Sim open after ROS startup error. Press Ctrl+C in this terminal to stop.")
            return
        raise

    period = 1.0 / max(0.1, ARGS.ros_rate)
    frame_index = 0
    log("\n=== ROS/rqt publishing ===")
    log(f"raw:   /{ARGS.ros_cell}/top_camera/image")
    log(f"debug: /{ARGS.ros_cell}/vision/debug_image")
    log(f"obb:   {'disabled in image-only mode' if obb_pub is None else f'/{ARGS.ros_cell}/vision/plate_obb'}")
    log(f"mode:  {'camera image only' if ARGS.ros_image_only else 'camera + YOLO'}")
    log("Open rqt_image_view and select the raw/debug topic. Press Ctrl+C here to stop.")
    log("==========================\n")

    last_camera_warning_time = 0.0
    try:
        while ARGS.ros_frames == 0 or frame_index < ARGS.ros_frames:
            world.step(render=True)
            simulation_app.update()
            bgr, camera_error = camera_bgr_or_none(camera)
            if bgr is None:
                now = time.monotonic()
                if now - last_camera_warning_time > 1.0:
                    log(f"[WARN] waiting for camera RGBA frame: {camera_error}")
                    last_camera_warning_time = now
                time.sleep(period)
                continue

            det = empty_obb() if ARGS.ros_image_only else infer(model, bgr)
            pred_pose = None
            error_mm = None
            if det.valid and not ARGS.ros_image_only:
                detected_name = CLASS_NAMES.get(det.class_id, "steel_plate")
                if detected_name not in active_object_names():
                    detected_name = "steel_plate" if ARGS.object == "both" else ARGS.object
                gt_x, gt_y, _ = object_pose(detected_name)
                det_top_z = float(SIZES[detected_name][2])
                pred_pose = obb_to_plate_pose(det, camera_model, BrainConfig(plate_top_z=det_top_z, pick_z=det_top_z, place_z=det_top_z))
                error_mm = float(np.linalg.norm(np.array([pred_pose.x - gt_x, pred_pose.y - gt_y])) * 1000.0)

            raw_msg = bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            if ARGS.ros_image_only:
                debug_frame = draw_scene_markers(bgr, gt_pixels)
            else:
                debug_frame = draw_debug(bgr, det, gt_pixel, pred_pose, error_mm)
            debug_msg = bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            stamp = node.get_clock().now().to_msg()
            raw_msg.header.stamp = stamp
            debug_msg.header.stamp = stamp
            raw_msg.header.frame_id = ARGS.camera_path
            debug_msg.header.frame_id = ARGS.camera_path
            raw_pub.publish(raw_msg)
            debug_pub.publish(debug_msg)
            if obb_pub is not None:
                msg = Float32MultiArray()
                msg.data = encode_obb(det)
                obb_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            frame_index += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        log(f"[ERROR] ROS publishing stopped: {type(exc).__name__}: {exc}")
        if ARGS.gui:
            render_until_interrupted(world, "Keeping Isaac Sim open after ROS error. Press Ctrl+C in this terminal to stop.")
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()


def main():
    world = None
    try:
        log("[isaac_yolo_calibration_smoke] setting up scene")
        world, camera = setup_scene()
        log("[isaac_yolo_calibration_smoke] warming up")
        for _ in range(max(1, ARGS.warmup_frames)):
            world.step(render=True)
            simulation_app.update()
        log("[isaac_yolo_calibration_smoke] warmup done")

        try:
            model = None if ARGS.ros_image_only or ARGS.scene_only else load_yolo_model()
        except ModuleNotFoundError as exc:
            if (ARGS.gui or ARGS.keep_open) and not ARGS.ros:
                print(f"\n{exc}")
                print("Falling back to scene-only mode so the Isaac Sim window stays open.\n")
                ARGS.scene_only = True
                model = None
            else:
                raise
        t_world_camera = get_tf(ARGS.camera_path)
        camera_model = CameraModel(ARGS.fx, ARGS.fy, ARGS.cx, ARGS.cy, t_world_camera)
        primary_name = "steel_plate" if ARGS.object == "both" else ARGS.object
        primary_x, primary_y, _ = object_pose(primary_name)
        top_z = float(SIZES[primary_name][2])
        gt_world = np.array([primary_x, primary_y, top_z], dtype=np.float64)
        gt_pixel = project_world_to_pixel(gt_world, t_world_camera)
        gt_pixels = ground_truth_pixels(t_world_camera)

        if ARGS.ros:
            log("[isaac_yolo_calibration_smoke] entering ROS publisher")
            publish_ros(world, camera, model, camera_model, top_z, gt_pixel, gt_pixels)
            return

        bgr = rgba_to_bgr(camera.get_rgba())
        if ARGS.scene_only:
            out_path = Path(ARGS.out)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_path), draw_scene_markers(bgr, gt_pixels))

            print("\n=== Isaac scene-only smoke ===")
            print(f"object:       {ARGS.object}")
            print(f"debug_image:  {out_path}")
            print("GUI will stay open until you close the Isaac Sim window or press Ctrl+C.")
            print("==============================\n")

            render_until_interrupted(world, "Keeping Isaac Sim open. Press Ctrl+C in this terminal to stop.")
            return

        if model is None:
            model = load_yolo_model()
        det = infer(model, bgr)
        pred_pose = None
        error_mm = None
        if det.valid:
            pred_pose = obb_to_plate_pose(det, camera_model, BrainConfig(plate_top_z=top_z, pick_z=top_z, place_z=top_z))
            error_mm = float(np.linalg.norm(np.array([pred_pose.x - primary_x, pred_pose.y - primary_y])) * 1000.0)

        out_path = Path(ARGS.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_path), draw_debug(bgr, det, gt_pixel, pred_pose, error_mm))

        print("\n=== Isaac YOLO + calibration smoke ===")
        print(f"object:       {ARGS.object}")
        print(f"model:        {ARGS.model}")
        print(f"gt_world:     x={primary_x:+.4f}, y={primary_y:+.4f}, top_z={top_z:+.4f}")
        print(f"gt_pixel:     {None if gt_pixel is None else (round(float(gt_pixel[0]), 2), round(float(gt_pixel[1]), 2))}")
        print(f"det_valid:    {det.valid}")
        print(f"det_class:    {det.class_id} ({CLASS_NAMES.get(det.class_id, 'unknown')})")
        print(f"confidence:   {det.confidence:.3f}")
        if pred_pose is not None:
            print(f"pred_world:   x={pred_pose.x:+.4f}, y={pred_pose.y:+.4f}, z={pred_pose.z:+.4f}, yaw={math.degrees(pred_pose.yaw):+.1f}deg")
            print(f"xy_error_mm:  {error_mm:.3f}")
        print(f"debug_image:  {out_path}")
        print("=======================================\n")

        if ARGS.keep_open or ARGS.gui:
            render_until_interrupted(world, "Keeping Isaac Sim open. Press Ctrl+C in this terminal to stop.")
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
