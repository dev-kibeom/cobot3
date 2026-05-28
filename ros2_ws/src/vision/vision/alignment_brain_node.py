from __future__ import annotations

# Brain node: YOLO OBB payload -> calibrated pick/place goal payload.
#
# Default ROS2 package run:
# ros2 run vision brain

import argparse
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray

try:
    from .alignment_brain import (
        CLASS_STEEL_CUBE,
        BrainConfig,
        CameraModel,
        TargetFrame,
        make_pick_place_goal,
    )
    from .vision_contracts import PickPlaceGoal, Pose2D, cell_to_robot_index, decode_obb, encode_goal
except ImportError:  # Allows direct execution from the legacy visions directory.
    from alignment_brain import (
        CLASS_STEEL_CUBE,
        BrainConfig,
        CameraModel,
        TargetFrame,
        make_pick_place_goal,
    )
    from vision_contracts import PickPlaceGoal, Pose2D, cell_to_robot_index, decode_obb, encode_goal


DEFAULT_TOP_CAMERA_ROTATION = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell", default="global")
    parser.add_argument("--robot-index", type=int, default=None, help="Override robot index. Defaults to 1 for global.")
    parser.add_argument("--obb-topic", default="", help="Default: /{cell}/vision/plate_obb")
    parser.add_argument("--goal-topic", default="", help="Default: /{cell}/vision/pick_place_goal")
    parser.add_argument(
        "--camera-transform-topic",
        default="",
        help="Optional Float32MultiArray(16) T_world_camera topic. Default for --cell m0609: /m0609/d455/t_world_camera.",
    )

    parser.add_argument("--fx", type=float, default=500.0)
    parser.add_argument("--fy", type=float, default=500.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--camera-x", type=float, default=0.35)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=1.35)
    parser.add_argument(
        "--t-world-camera",
        default="",
        help="Optional 16 comma-separated row-major values. Overrides camera-x/y/z and default top-camera rotation.",
    )

    parser.add_argument("--place-x", type=float, default=0.45)
    parser.add_argument("--place-y", type=float, default=-0.22)
    parser.add_argument("--place-yaw-deg", type=float, default=0.0)
    parser.add_argument("--plate-top-z", type=float, default=0.03)
    parser.add_argument("--cube-top-z", type=float, default=0.05)
    parser.add_argument("--plate-place-z", type=float, default=None)
    parser.add_argument("--cube-place-z", type=float, default=None)
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--axis-sample-px", type=float, default=80.0)

    parser.add_argument("--max-hz", type=float, default=10.0, help="0 disables rate limiting.")
    parser.add_argument("--publish-invalid", action="store_true")
    parser.add_argument("--stats-period", type=float, default=5.0)
    return parser.parse_args(remove_ros_args()[1:])


def goal_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def transform_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def parse_transform(value: str) -> np.ndarray:
    parts = [part.strip() for part in value.split(",") if part.strip()]
    if len(parts) != 16:
        raise ValueError("--t-world-camera must contain exactly 16 comma-separated values.")
    return np.array([float(part) for part in parts], dtype=np.float64).reshape(4, 4)


def default_top_camera_transform(args) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = DEFAULT_TOP_CAMERA_ROTATION
    transform[:3, 3] = np.array([args.camera_x, args.camera_y, args.camera_z], dtype=np.float64)
    return transform


def resolved_camera_transform_topic(cell: str, override: str) -> str:
    if override:
        return override
    return f"/{cell}/d455/t_world_camera" if cell == "m0609" else ""


def resolved_robot_index(cell: str, override: int | None) -> int:
    if override is not None:
        return override
    index = cell_to_robot_index(cell)
    return 1 if index == 0 and cell == "global" else index


def with_robot_index(goal: PickPlaceGoal, robot_index: int) -> PickPlaceGoal:
    if goal.robot_index == robot_index:
        return goal
    return PickPlaceGoal(
        valid=goal.valid,
        robot_index=robot_index,
        confidence=goal.confidence,
        pick=goal.pick,
        place=goal.place,
        dx=goal.dx,
        dy=goal.dy,
        dyaw=goal.dyaw,
        ee_type=goal.ee_type,
    )


class AlignmentBrainNode(Node):
    def __init__(self, args):
        super().__init__("alignment_brain_node")
        self.args = args
        self.obb_topic = args.obb_topic or f"/{args.cell}/vision/plate_obb"
        self.goal_topic = args.goal_topic or f"/{args.cell}/vision/pick_place_goal"
        self.camera_transform_topic = resolved_camera_transform_topic(args.cell, args.camera_transform_topic)
        self.robot_index = resolved_robot_index(args.cell, args.robot_index)
        t_world_camera = parse_transform(args.t_world_camera) if args.t_world_camera else default_top_camera_transform(args)
        self.camera_model = CameraModel(args.fx, args.fy, args.cx, args.cy, t_world_camera)
        self.publisher = self.create_publisher(Float32MultiArray, self.goal_topic, goal_qos())
        self.subscription = self.create_subscription(Float32MultiArray, self.obb_topic, self.obb_callback, goal_qos())
        self.camera_transform_subscription = None
        if self.camera_transform_topic:
            self.camera_transform_subscription = self.create_subscription(
                Float32MultiArray,
                self.camera_transform_topic,
                self.camera_transform_callback,
                transform_qos(),
            )

        self.received = 0
        self.published = 0
        self.dropped = 0
        self.camera_transform_updates = 0
        self.waiting_for_camera_transform = bool(self.camera_transform_topic and not args.t_world_camera)
        self.last_camera_wait_warning = 0.0
        self.last_publish_time = 0.0
        self.last_stats_time = time.monotonic()

        self.get_logger().info(f"{self.obb_topic} -> {self.goal_topic}")
        self.get_logger().info(
            f"robot_index={self.robot_index}, place=({args.place_x:+.3f},{args.place_y:+.3f},{math.radians(args.place_yaw_deg):+.3f}rad)"
        )
        if self.camera_transform_topic:
            self.get_logger().info(f"camera transform topic: {self.camera_transform_topic}")
        elif args.t_world_camera:
            self.get_logger().info("camera transform source: --t-world-camera")
        else:
            self.get_logger().info("camera transform source: static top-camera defaults")

    def camera_transform_callback(self, msg: Float32MultiArray):
        values = list(msg.data)
        if len(values) != 16:
            self.get_logger().warning(f"ignored camera transform with {len(values)} values; expected 16")
            return
        try:
            t_world_camera = np.array(values, dtype=np.float64).reshape(4, 4)
        except Exception as exc:
            self.get_logger().warning(f"failed to decode camera transform: {exc}")
            return

        self.camera_model = CameraModel(
            self.args.fx,
            self.args.fy,
            self.args.cx,
            self.args.cy,
            t_world_camera,
        )
        self.camera_transform_updates += 1
        if self.waiting_for_camera_transform:
            self.waiting_for_camera_transform = False
            self.get_logger().info("received first camera transform; brain goal publishing enabled")

    def object_heights(self, class_id: int) -> tuple[float, float]:
        if class_id == CLASS_STEEL_CUBE:
            top_z = self.args.cube_top_z
            place_z = self.args.cube_place_z if self.args.cube_place_z is not None else top_z
            return top_z, place_z
        top_z = self.args.plate_top_z
        place_z = self.args.plate_place_z if self.args.plate_place_z is not None else top_z
        return top_z, place_z

    def make_goal(self, data) -> PickPlaceGoal:
        det = decode_obb(data)
        top_z, place_z = self.object_heights(det.class_id)
        target = TargetFrame(Pose2D(self.args.place_x, self.args.place_y, place_z, math.radians(self.args.place_yaw_deg)))
        config = BrainConfig(
            plate_top_z=top_z,
            pick_z=top_z,
            place_z=place_z,
            min_confidence=self.args.min_confidence,
            axis_sample_px=self.args.axis_sample_px,
        )
        goal = make_pick_place_goal(self.args.cell, det, target, self.camera_model, config)
        return with_robot_index(goal, self.robot_index)

    def should_rate_limit(self) -> bool:
        if self.args.max_hz <= 0.0:
            return False
        now = time.monotonic()
        if now - self.last_publish_time < 1.0 / self.args.max_hz:
            return True
        return False

    def maybe_log_stats(self):
        now = time.monotonic()
        if self.args.stats_period <= 0.0 or now - self.last_stats_time < self.args.stats_period:
            return
        self.get_logger().info(f"brain stats: received={self.received}, published={self.published}, dropped={self.dropped}")
        self.last_stats_time = now

    def obb_callback(self, msg: Float32MultiArray):
        self.received += 1
        if self.waiting_for_camera_transform:
            self.dropped += 1
            now = time.monotonic()
            if now - self.last_camera_wait_warning > self.args.stats_period:
                self.get_logger().warning(
                    f"waiting for camera transform on {self.camera_transform_topic}; dropping OBB until it arrives"
                )
                self.last_camera_wait_warning = now
            self.maybe_log_stats()
            return
        if self.should_rate_limit():
            self.dropped += 1
            self.maybe_log_stats()
            return

        try:
            goal = self.make_goal(msg.data)
        except Exception as exc:
            self.dropped += 1
            self.get_logger().warning(f"failed to make goal: {exc}")
            self.maybe_log_stats()
            return

        if not goal.valid and not self.args.publish_invalid:
            self.dropped += 1
            self.maybe_log_stats()
            return

        out = Float32MultiArray()
        out.data = encode_goal(goal)
        self.publisher.publish(out)
        self.published += 1
        self.last_publish_time = time.monotonic()

        self.get_logger().info(
            "goal "
            f"valid={goal.valid} robot={goal.robot_index} conf={goal.confidence:.2f} "
            f"pick=({goal.pick.x:+.3f},{goal.pick.y:+.3f},{goal.pick.z:+.3f},{goal.pick.yaw:+.3f}) "
            f"place=({goal.place.x:+.3f},{goal.place.y:+.3f},{goal.place.z:+.3f},{goal.place.yaw:+.3f})"
        )
        self.maybe_log_stats()


def main():
    args = parse_args()
    rclpy.init()
    node = AlignmentBrainNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
