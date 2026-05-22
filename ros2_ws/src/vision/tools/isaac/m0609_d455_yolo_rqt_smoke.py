from __future__ import annotations

# Isaac Sim smoke test for the M0609 wrist-mounted RealSense D455 path.
#
# Terminal 1:
#   export ROS_DOMAIN_ID=104
#   /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#     tools/isaac/m0609_d455_yolo_rqt_smoke.py --gui --execute-goals
#
# Terminal 2:
#   export ROS_DOMAIN_ID=104
#   source /home/rokey/smart_factory_project/cobot3/ros2_ws/install/setup.bash
#   ros2 run vision yolo \
#     --cells m0609 \
#     --image-topic-template /m0609/d455/color/image \
#     --obb-topic-template /m0609/vision/plate_obb \
#     --debug-topic-template /m0609/vision/debug_image \
#     --publish-debug true
#
# Terminal 3:
#   export ROS_DOMAIN_ID=104
#   rqt_image_view

import argparse
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Run Isaac Sim with GUI.")
    parser.add_argument("--ros-domain", type=int, default=104)
    parser.add_argument("--ros-cell", default="m0609")
    parser.add_argument("--ros-rate", type=float, default=10.0)
    parser.add_argument("--ros-frames", type=int, default=0, help="0 means publish until interrupted.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--color-topic", default="")
    parser.add_argument("--depth-vis-topic", default="")
    parser.add_argument("--publish-depth-vis", action="store_true")
    parser.add_argument("--goal-topic", default="")
    parser.add_argument("--camera-transform-topic", default="")
    parser.add_argument("--execute-goals", action="store_true", help="Subscribe to pick/place goals and move M0609.")
    parser.add_argument("--min-goal-confidence", type=float, default=0.35)
    parser.add_argument("--pick-ee-offset-z", type=float, default=0.20)
    parser.add_argument("--m0609-dir", default="/home/rokey/dev_ws/isaac_sim/src/m0609_vision/M0609")
    parser.add_argument("--cube-x", type=float, default=0.30)
    parser.add_argument("--cube-y", type=float, default=0.00)
    parser.add_argument("--cube-size", type=float, default=0.05)
    parser.add_argument("--cube-yaw-deg", type=float, default=0.0)
    parser.add_argument("--warmup-frames", type=int, default=60)
    parser.add_argument("--capture-dataset", action="store_true")
    parser.add_argument("--capture-count", type=int, default=300)
    parser.add_argument("--capture-val-ratio", type=float, default=0.2)
    parser.add_argument("--capture-background-ratio", type=float, default=0.35)
    parser.add_argument("--capture-seed", type=int, default=52)
    parser.add_argument("--capture-dir", default=str(root / "work" / "datasets" / "m0609_d455_wrist_obb"))
    parser.add_argument("--capture-clean", action="store_true")
    parser.add_argument("--capture-frames-per-sample", type=int, default=3)
    parser.add_argument("--capture-cube-x-range", default="0.22,0.42")
    parser.add_argument("--capture-cube-y-range", default="-0.16,0.16")
    parser.add_argument(
        "--home-joints-deg",
        default="0,0,70,0,90,0",
        help="Comma-separated joint_1..joint_6 target in degrees. Default points the D455 toward the cube.",
    )
    parser.add_argument("--out", default=str(root / "work" / "m0609_d455_yolo_rqt_smoke.png"))
    args = parser.parse_args()
    args.color_topic = args.color_topic or f"/{args.ros_cell}/d455/color/image"
    args.depth_vis_topic = args.depth_vis_topic or f"/{args.ros_cell}/d455/depth/vis"
    args.goal_topic = args.goal_topic or f"/{args.ros_cell}/vision/pick_place_goal"
    args.camera_transform_topic = args.camera_transform_topic or f"/{args.ros_cell}/d455/t_world_camera"
    return args


ARGS = parse_args()
os.environ["ROS_DOMAIN_ID"] = str(ARGS.ros_domain)

simulation_app = SimulationApp({"headless": not ARGS.gui})

import cv2
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.core.utils.numpy.rotations as rot_utils

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
from isaacsim.robot_setup.assembler import RobotAssembler  # noqa: E402

M0609_DIR = Path(ARGS.m0609_dir)
if str(M0609_DIR) not in sys.path:
    sys.path.append(str(M0609_DIR))

from realsense_mount import attach_realsense_d455  # noqa: E402
from wrist_camera import WristCamera  # noqa: E402
from m0609_pick_place_controller import PickPlaceController  # noqa: E402

VISION_ROOT = Path(__file__).resolve().parents[2]
if str(VISION_ROOT) not in sys.path:
    sys.path.append(str(VISION_ROOT))
from vision.vision_contracts import decode_goal  # noqa: E402


EE_LINK_NAME = "link_6"
GRIPPER_BASE_LINK = "angle_bracket"
CAM_OFFSET_T = (0.0, 0.045, 0.05)
CAM_OFFSET_RPY = (0.0, -90.0, 90.0)
CAM_SENSOR_EXTRA_YAW_DEG = 90.0
M0609_URDF_PATH = M0609_DIR / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf"
ONROBOT_URDF_PATH = M0609_DIR / "onrobot_rg2" / "urdf" / "onrobot_rg2.urdf"
M0609_RMPFLOW_CONFIG_PATH = M0609_DIR / "m0609_rmpflow_common.yaml"
M0609_DESCRIPTION_PATH = M0609_DIR / "m0609_rg2_description.yaml"
T_GL_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])
CLASS_STEEL_CUBE = 1


def log(message):
    print(message, flush=True)


def localized_urdf_path(urdf_path):
    source = Path(urdf_path)
    tree = ET.parse(source)
    root = tree.getroot()
    changed = False
    marker = "m0609_vision/M0609"

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        fixed_filename = filename
        if marker in filename:
            suffix = filename.split(marker, 1)[1].lstrip("/\\")
            fixed_filename = str(M0609_DIR / suffix)
        elif filename.startswith("../"):
            candidate = (source.parent / filename).resolve()
            if candidate.exists():
                fixed_filename = str(candidate)

        if fixed_filename != filename:
            mesh.set("filename", fixed_filename)
            changed = True

    if not changed:
        return str(source)

    out_dir = Path("/tmp/m0609_d455_yolo_rqt_urdf")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / source.name
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    log(f"[OK] localized URDF mesh paths: {source} -> {out_path}")
    return str(out_path)


def import_urdf(urdf_path, fix_base=True):
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file does not exist: {urdf_path}")
    urdf_path = localized_urdf_path(urdf_path)

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.fix_base = fix_base
    import_config.distance_scale = 1.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1e10
    import_config.default_position_drive_damping = 1e5

    _, articulation_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    if articulation_path is None:
        raise RuntimeError(f"URDF import failed: {urdf_path}")
    robot_root = articulation_path.rsplit("/", 1)[0] or articulation_path
    log(f"[OK] URDF import: {urdf_path}")
    log(f"     articulation={articulation_path}")
    log(f"     root={robot_root}")
    return robot_root


def assemble_robot(stage, robot_base, robot_base_mount, robot_attach, robot_attach_mount):
    assembler = RobotAssembler()
    assembler.begin_assembly(
        stage,
        robot_base,
        robot_base_mount,
        robot_attach,
        robot_attach_mount,
        "Gripper",
        "m0609_rg2",
    )
    assembler.assemble()
    assembler.finish_assemble()


def find_prim_path_by_name(root_path, link_name):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


def find_joint_indices(robot, joint_names):
    indices = []
    for fallback_index, joint_name in enumerate(joint_names):
        if joint_name in robot.dof_names:
            indices.append(robot.dof_names.index(joint_name))
            continue
        for index, dof_name in enumerate(robot.dof_names):
            if dof_name.endswith(joint_name):
                indices.append(index)
                break
        else:
            if fallback_index < len(robot.dof_names):
                indices.append(fallback_index)
            else:
                raise RuntimeError(f"Could not find joint {joint_name}: {robot.dof_names}")
    return np.array(indices, dtype=np.int64)


def make_preview_surface(stage, material_path, color, metallic=1.0, roughness=0.45):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(stage, prim_path, material):
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        UsdShade.MaterialBindingAPI(prim).Bind(material)


def apply_finger_material(robot_root, material):
    for index, link_name in enumerate(
        [
            "left_inner_finger",
            "right_inner_finger",
            "left_inner_knuckle",
            "right_inner_knuckle",
            "left_outer_knuckle",
            "right_outer_knuckle",
        ]
    ):
        link_path = find_prim_path_by_name(robot_root, link_name)
        if link_path:
            SingleGeometryPrim(prim_path=link_path, name=f"m0609_goal_finger_geom_{index}").apply_physics_material(material)


def disable_physics_under(root_path):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)


def parse_home_joints():
    values = [float(value.strip()) for value in ARGS.home_joints_deg.split(",") if value.strip()]
    if len(values) != 6:
        raise ValueError("--home-joints-deg must contain 6 comma-separated values.")
    return np.deg2rad(np.array(values, dtype=np.float64))


def get_tf(path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {path}")
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64).T


def format_transform(transform):
    return ",".join(f"{float(value):.9g}" for value in transform.reshape(-1))


def parse_range(value):
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) != 2 or parts[0] >= parts[1]:
        raise ValueError(f"range must be min,max: {value!r}")
    return parts[0], parts[1]


def project_world_to_pixel(p_world, t_world_camera):
    p_h = np.array([p_world[0], p_world[1], p_world[2], 1.0], dtype=np.float64)
    p_gl = np.linalg.inv(t_world_camera) @ p_h
    p_cv = T_GL_TO_CV @ p_gl
    if p_cv[2] <= 1e-6:
        return None
    u = 500.0 * (p_cv[0] / p_cv[2]) + ARGS.width / 2.0
    v = 500.0 * (p_cv[1] / p_cv[2]) + ARGS.height / 2.0
    return np.array([u, v], dtype=np.float64)


def cube_top_face_corners_world(center_xy, yaw_rad):
    half = ARGS.cube_size / 2.0
    local = np.array(
        [
            [-half, -half],
            [half, -half],
            [half, half],
            [-half, half],
        ],
        dtype=np.float64,
    )
    rot = np.array(
        [[np.cos(yaw_rad), -np.sin(yaw_rad)], [np.sin(yaw_rad), np.cos(yaw_rad)]],
        dtype=np.float64,
    )
    xy = local @ rot.T + np.array(center_xy, dtype=np.float64)
    return np.column_stack([xy, np.full(4, ARGS.cube_size)])


def make_cube_label(camera_path, center_xy, yaw_rad):
    t_world_camera = get_tf(camera_path)
    corners_world = cube_top_face_corners_world(center_xy, yaw_rad)
    corners_px = [project_world_to_pixel(point, t_world_camera) for point in corners_world]
    if any(point is None for point in corners_px):
        return None
    corners_px = np.array(corners_px, dtype=np.float64)
    if not ((corners_px[:, 0] >= 1).all() and (corners_px[:, 0] <= ARGS.width - 2).all()):
        return None
    if not ((corners_px[:, 1] >= 1).all() and (corners_px[:, 1] <= ARGS.height - 2).all()):
        return None
    norm = corners_px / np.array([ARGS.width, ARGS.height], dtype=np.float64)
    values = [str(CLASS_STEEL_CUBE)] + [f"{x:.6f} {y:.6f}" for x, y in norm]
    return " ".join(values)


def prefer_isaac_ros2_python():
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    if ros_distro not in ("humble", "jazzy"):
        ros_distro = "humble"

    executable = Path(sys.executable)
    release_roots = [executable.parents[3], executable.resolve().parents[3]]
    for release_root in release_roots:
        bridge_root = release_root / "exts" / "isaacsim.ros2.bridge"
        rclpy_path = bridge_root / ros_distro / "rclpy"
        lib_path = bridge_root / ros_distro / "lib"
        if not rclpy_path.exists():
            continue
        sys.path.insert(0, str(rclpy_path))
        os.environ.setdefault("ROS_DISTRO", ros_distro)
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        if lib_path.exists():
            current = os.environ.get("LD_LIBRARY_PATH", "")
            lib_text = str(lib_path)
            if lib_text not in current.split(":"):
                os.environ["LD_LIBRARY_PATH"] = f"{current}:{lib_text}" if current else lib_text
        log(f"[m0609_d455_yolo_rqt_smoke] using Isaac ROS2 {ros_distro} Python bindings")
        return

    checked = ", ".join(
        str(root / "exts" / "isaacsim.ros2.bridge" / ros_distro / "rclpy")
        for root in release_roots
    )
    raise RuntimeError(f"Isaac ROS2 bridge rclpy path not found. Checked: {checked}")


def depth_to_vis(depth):
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        return np.zeros(depth.shape, dtype=np.uint8)
    near = np.percentile(depth[valid], 5)
    far = np.percentile(depth[valid], 95)
    if far <= near:
        far = near + 1e-3
    clipped = np.clip((depth - near) / (far - near), 0.0, 1.0)
    return (255.0 * (1.0 - clipped)).astype(np.uint8)


def setup_scene():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    stage.DefinePrim("/World/Materials", "Scope")
    world.scene.add_default_ground_plane()

    light = UsdLux.DistantLight.Define(stage, "/World/m0609_test_light")
    light.CreateIntensityAttr(900.0)
    light_xform = UsdGeom.Xformable(light.GetPrim())
    light_xform.ClearXformOpOrder()
    light_xform.AddRotateXYZOp().Set(Gf.Vec3f(40.0, 0.0, 15.0))

    world.scene.add(
        VisualCuboid(
            prim_path="/World/work_table",
            name="work_table",
            position=np.array([0.35, 0.0, -0.004]),
            scale=np.array([0.70, 0.55, 0.008]),
            color=np.array([0.32, 0.46, 0.50]),
        )
    )

    cube_z = ARGS.cube_size / 2.0
    cube_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_contact",
        static_friction=1.2,
        dynamic_friction=1.0,
        restitution=0.0,
    )
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/steel_cube",
            name="steel_cube",
            position=np.array([ARGS.cube_x, ARGS.cube_y, cube_z]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            scale=np.array([ARGS.cube_size, ARGS.cube_size, ARGS.cube_size]),
            color=np.array([0.82, 0.82, 0.78]),
            mass=0.03,
            physics_material=cube_material,
        )
    )
    metal = make_preview_surface(stage, "/World/Materials/smoke_steel_cube", np.array([0.82, 0.82, 0.78]))
    bind_material(stage, "/World/steel_cube", metal)

    robot_root = import_urdf(str(M0609_URDF_PATH), fix_base=True)
    gripper_root = import_urdf(str(ONROBOT_URDF_PATH), fix_base=False)
    robot_ee_path = find_prim_path_by_name(robot_root, EE_LINK_NAME) or f"{robot_root}/{EE_LINK_NAME}"
    gripper_base_path = find_prim_path_by_name(gripper_root, GRIPPER_BASE_LINK) or f"{gripper_root}/{GRIPPER_BASE_LINK}"
    assemble_robot(stage, robot_root, robot_ee_path, gripper_root, gripper_base_path)

    for _ in range(10):
        simulation_app.update()

    robot_ee_path = find_prim_path_by_name(robot_root, EE_LINK_NAME) or robot_ee_path
    gripper = ParallelGripper(
        end_effector_prim_path=robot_ee_path,
        joint_prim_names=["finger_joint", "right_inner_knuckle_joint"],
        joint_opened_positions=np.array([0.0, 0.0]),
        joint_closed_positions=np.array([0.8, 0.8]),
        action_deltas=np.array([-0.5, -0.5]),
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=robot_root,
            name="m0609_robot",
            end_effector_prim_path=robot_ee_path,
            gripper=gripper,
        )
    )

    finger_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/finger_contact",
        static_friction=4.0,
        dynamic_friction=3.0,
        restitution=0.0,
    )
    apply_finger_material(robot_root, finger_material)

    camera_parent = find_prim_path_by_name(robot_root, GRIPPER_BASE_LINK)
    if camera_parent is None:
        raise RuntimeError(f"Could not find {GRIPPER_BASE_LINK} under {robot_root}.")
    realsense_path = attach_realsense_d455(
        parent_prim_path=camera_parent,
        child_name="realsense_d455",
        translation=CAM_OFFSET_T,
        rpy_deg=CAM_OFFSET_RPY,
    )
    for _ in range(5):
        simulation_app.update()
    disable_physics_under(realsense_path)

    ov_cam_path = find_prim_path_by_name(realsense_path, "Camera_OmniVision_OV9782_Color")
    if ov_cam_path:
        cam_prim = stage.GetPrimAtPath(ov_cam_path)
        cam_xform = UsdGeom.Xformable(cam_prim)
        existing_ops = [op.GetOpName() for op in cam_xform.GetOrderedXformOps()]
        if "xformOp:rotateZ:extra" not in existing_ops:
            rot_op = cam_xform.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat, opSuffix="extra")
            rot_op.Set(float(CAM_SENSOR_EXTRA_YAW_DEG))
            cam_prim.GetAttribute("xformOpOrder").Set(Vt.TokenArray(existing_ops + [rot_op.GetOpName()]))
        camera = WristCamera.from_existing_prim(ov_cam_path, resolution=(ARGS.width, ARGS.height))
    else:
        camera = WristCamera(
            parent_prim_path=realsense_path,
            name="wrist_rgb",
            resolution=(ARGS.width, ARGS.height),
            translation=(0.0, 0.0, 0.0),
            rpy_deg=(0.0, 0.0, CAM_SENSOR_EXTRA_YAW_DEG),
        )

    world.reset()
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    camera.initialize()

    joint_indices = find_joint_indices(robot, ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
    robot.set_joint_positions(parse_home_joints(), joint_indices=joint_indices)
    log(f"[OK] M0609 D455 camera prim: {camera._prim_path}")
    log(f"[OK] steel cube: x={ARGS.cube_x:+.3f}, y={ARGS.cube_y:+.3f}, size={ARGS.cube_size:.3f}")
    return world, camera, robot


def camera_rgb(camera):
    rgb = camera.get_rgb()
    if rgb is None or rgb.size == 0:
        return None
    if rgb.dtype != np.uint8:
        if float(np.nanmax(rgb)) <= 1.0:
            rgb = rgb * 255.0
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    return rgb


def camera_depth_vis(camera):
    frame = camera.camera.get_current_frame()
    depth = frame.get("distance_to_image_plane") if frame else None
    if depth is None:
        return None
    return depth_to_vis(depth)


class GoalExecutor:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        self.controller = PickPlaceController(
            name="m0609_ros_goal_pick_place_controller",
            gripper=robot.gripper,
            robot_articulation=robot,
            end_effector_initial_height=0.25,
            events_dt=[0.008, 0.005, 0.02, 0.02, 0.001, 0.01, 0.005, 0.05, 0.008, 0.08],
            urdf_path=str(M0609_URDF_PATH),
            robot_description_path=str(M0609_DESCRIPTION_PATH),
            rmpflow_config_path=str(M0609_RMPFLOW_CONFIG_PATH),
            end_effector_frame_name=EE_LINK_NAME,
        )
        self.active_goal = None
        self.completed_goals = 0

    def accept(self, msg):
        goal = decode_goal(msg.data)
        if not goal.valid:
            self.node.get_logger().info("ignored invalid pick/place goal")
            return
        if goal.confidence < ARGS.min_goal_confidence:
            self.node.get_logger().info(f"ignored low-confidence goal: {goal.confidence:.3f}")
            return
        if self.active_goal is not None:
            self.node.get_logger().info("ignored goal while pick/place is already active")
            return

        self.active_goal = goal
        self.controller.reset()
        self.node.get_logger().info(
            "accepted goal "
            f"conf={goal.confidence:.2f} "
            f"pick=({goal.pick.x:+.3f},{goal.pick.y:+.3f},{goal.pick.z:+.3f},{goal.pick.yaw:+.3f}) "
            f"place=({goal.place.x:+.3f},{goal.place.y:+.3f},{goal.place.z:+.3f},{goal.place.yaw:+.3f}) "
            f"dyaw={goal.dyaw:+.3f}"
        )

    def step(self):
        if self.active_goal is None:
            return

        goal = self.active_goal
        current_joints = self.robot.get_joint_positions()
        actions = self.controller.forward(
            picking_position=np.array([goal.pick.x, goal.pick.y, goal.pick.z], dtype=np.float64),
            placing_position=np.array([goal.place.x, goal.place.y, goal.place.z], dtype=np.float64),
            current_joint_positions=current_joints,
            end_effector_offset=np.array([0.0, 0.0, ARGS.pick_ee_offset_z], dtype=np.float64),
        )
        self.robot.apply_action(actions)
        if self.controller.is_done():
            self.completed_goals += 1
            self.node.get_logger().info(f"pick/place goal complete: count={self.completed_goals}")
            self.active_goal = None


def publish_ros(world, camera, robot):
    prefer_isaac_ros2_python()
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = rclpy.create_node("m0609_d455_yolo_rqt_smoke")
    bridge = CvBridge()
    color_pub = node.create_publisher(Image, ARGS.color_topic, 10)
    transform_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    camera_transform_pub = node.create_publisher(Float32MultiArray, ARGS.camera_transform_topic, transform_qos)
    depth_vis_pub = None
    if ARGS.publish_depth_vis:
        depth_vis_pub = node.create_publisher(Image, ARGS.depth_vis_topic, 10)
    goal_executor = None
    if ARGS.execute_goals:
        goal_executor = GoalExecutor(node, robot)
        node.create_subscription(Float32MultiArray, ARGS.goal_topic, goal_executor.accept, 10)

    period = 1.0 / max(0.1, ARGS.ros_rate)
    frame_index = 0
    last_wait_log = 0.0

    log("\n=== M0609 D455 ROS/rqt smoke ===")
    log(f"ROS_DOMAIN_ID: {ARGS.ros_domain}")
    log(f"color:         {ARGS.color_topic}")
    log(f"camera_tf:     {ARGS.camera_transform_topic}")
    if depth_vis_pub is not None:
        log(f"depth_vis:     {ARGS.depth_vis_topic}")
    log("YOLO debug:    /m0609/vision/debug_image")
    if goal_executor is not None:
        log(f"goal input:    {ARGS.goal_topic}")
    log("Open rqt_image_view and select the color/debug topic. Press Ctrl+C here to stop.")
    log("=================================\n")

    try:
        while ARGS.ros_frames == 0 or frame_index < ARGS.ros_frames:
            world.step(render=True)
            simulation_app.update()
            if goal_executor is not None:
                goal_executor.step()
            transform_msg = Float32MultiArray()
            transform_msg.data = [float(value) for value in get_tf(camera._prim_path).reshape(-1)]
            camera_transform_pub.publish(transform_msg)
            rgb = camera_rgb(camera)
            if rgb is None:
                now = time.monotonic()
                if now - last_wait_log > 1.0:
                    log("[WARN] waiting for D455 color frame...")
                    last_wait_log = now
                time.sleep(period)
                continue

            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            stamp = node.get_clock().now().to_msg()
            color_msg = bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = camera._prim_path
            color_pub.publish(color_msg)

            if depth_vis_pub is not None:
                depth_vis = camera_depth_vis(camera)
                if depth_vis is not None:
                    depth_msg = bridge.cv2_to_imgmsg(depth_vis, encoding="mono8")
                    depth_msg.header.stamp = stamp
                    depth_msg.header.frame_id = camera._prim_path
                    depth_vis_pub.publish(depth_msg)

            if frame_index % max(1, int(ARGS.ros_rate) * 5) == 0:
                out_path = Path(ARGS.out)
                out_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out_path), bgr)

            rclpy.spin_once(node, timeout_sec=0.0)
            frame_index += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def clean_capture_dir(root):
    for pattern in ("images/train/*.png", "images/val/*.png", "labels/train/*.txt", "labels/val/*.txt"):
        for path in root.glob(pattern):
            path.unlink()
    yaml_path = root / "m0609_d455_wrist_obb.yaml"
    if yaml_path.exists():
        yaml_path.unlink()


def prepare_capture_dirs(root):
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)


def write_capture_yaml(root):
    yaml_path = root / "m0609_d455_wrist_obb.yaml"
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


def capture_d455_dataset(world, camera):
    rng = np.random.default_rng(ARGS.capture_seed)
    root = Path(ARGS.capture_dir).expanduser().resolve()
    prepare_capture_dirs(root)
    if ARGS.capture_clean:
        clean_capture_dir(root)
        prepare_capture_dirs(root)

    cube = world.scene.get_object("steel_cube")
    x_range = parse_range(ARGS.capture_cube_x_range)
    y_range = parse_range(ARGS.capture_cube_y_range)
    val_count = int(round(ARGS.capture_count * ARGS.capture_val_ratio))
    counts = {"train_cube": 0, "train_background": 0, "val_cube": 0, "val_background": 0}

    log("\n=== D455 wrist-view dataset capture ===")
    log(f"output:      {root}")
    log(f"count:       {ARGS.capture_count}")
    log(f"background:  {ARGS.capture_background_ratio:.2f}")
    log("=======================================\n")

    written = 0
    attempts = 0
    max_attempts = max(ARGS.capture_count * 30, ARGS.capture_count + 100)
    while written < ARGS.capture_count:
        attempts += 1
        if attempts > max_attempts:
            raise RuntimeError(f"Too many rejected labels: written={written}, attempts={attempts}")

        split = "val" if written < val_count else "train"
        background = bool(rng.random() < ARGS.capture_background_ratio)
        label = ""
        if background:
            cube.set_world_pose(position=np.array([0.0, 0.0, -5.0]))
        else:
            center_xy = np.array([rng.uniform(*x_range), rng.uniform(*y_range)], dtype=np.float64)
            yaw_deg = float(rng.uniform(-45.0, 45.0))
            yaw_rad = np.deg2rad(yaw_deg)
            label = make_cube_label(camera._prim_path, center_xy, yaw_rad)
            if label is None:
                continue
            cube.set_world_pose(
                position=np.array([center_xy[0], center_xy[1], ARGS.cube_size / 2.0]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, yaw_deg]), degrees=True),
            )

        for _ in range(max(1, ARGS.capture_frames_per_sample)):
            world.step(render=True)
            simulation_app.update()

        rgb = camera_rgb(camera)
        if rgb is None:
            continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        stem = f"{split}_{written:05d}_{'background' if background else 'steel_cube'}"
        image_path = root / "images" / split / f"{stem}.png"
        label_path = root / "labels" / split / f"{stem}.txt"
        if not cv2.imwrite(str(image_path), bgr):
            raise RuntimeError(f"Failed to write image: {image_path}")
        label_path.write_text((label + "\n") if label else "", encoding="utf-8")

        key = f"{split}_{'background' if background else 'cube'}"
        counts[key] += 1
        written += 1
        if written == 1 or written % 25 == 0:
            log(f"[capture] written={written}/{ARGS.capture_count} attempts={attempts}")

    yaml_path = write_capture_yaml(root)
    log("\n=== D455 wrist-view dataset ready ===")
    log(f"yaml:             {yaml_path}")
    log(f"train_cube:       {counts['train_cube']}")
    log(f"train_background: {counts['train_background']}")
    log(f"val_cube:         {counts['val_cube']}")
    log(f"val_background:   {counts['val_background']}")
    log("=====================================\n")


def main():
    world = None
    try:
        world, camera, robot = setup_scene()
        log("[m0609_d455_yolo_rqt_smoke] warming up")
        for _ in range(max(1, ARGS.warmup_frames)):
            world.step(render=True)
            simulation_app.update()
        t_world_camera = get_tf(camera._prim_path)
        log("\n=== D455 brain calibration args ===")
        log(
            "ros2 run vision brain "
            "--cell m0609 "
            "--obb-topic /m0609/vision/plate_obb "
            "--goal-topic /m0609/vision/pick_place_goal "
            "--camera-transform-topic /m0609/d455/t_world_camera"
        )
        log("The camera transform is also published continuously for the brain node.")
        log(f"Initial T_world_camera: {format_transform(t_world_camera)}")
        log("===================================\n")
        if ARGS.capture_dataset:
            capture_d455_dataset(world, camera)
            return
        publish_ros(world, camera, robot)
    finally:
        if world is not None:
            world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
