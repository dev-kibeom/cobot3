from __future__ import annotations

# Thin ROS2 domain gateway for pick/place goals.
#
# It subscribes to a Float32MultiArray goal in the vision domain and republishes
# only that small payload into the motion domain. It intentionally does not
# bridge images, debug images, TF, joint states, or any high-rate internal topics.
#
# Example:
# python3 ros2 run vision goal_gateway \
#   --source-domain 104 \
#   --target-domain 102 \
#   --input-topic /global/vision/pick_place_goal \
#   --output-topic /m0609_vision/pick_place_goal

import argparse
import os
import time

import numpy as np

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

try:
    from .vision_contracts import GOAL_PAYLOAD_SIZE, decode_goal
except ImportError:  # Allows direct execution from the legacy visions directory.
    from vision_contracts import GOAL_PAYLOAD_SIZE, decode_goal


def default_domain() -> int:
    value = os.environ.get("ROS_DOMAIN_ID", "104")
    try:
        return int(value)
    except ValueError:
        return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-domain", type=int, default=default_domain(), help="Domain that publishes the vision goal.")
    parser.add_argument("--target-domain", type=int, default=102, help="Motion team ROS_DOMAIN_ID.")
    parser.add_argument("--input-topic", default="/global/vision/pick_place_goal")
    parser.add_argument("--output-topic", default="/m0609_vision/pick_place_goal")
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--max-hz", type=float, default=10.0, help="0 disables rate limiting.")
    parser.add_argument("--dedupe-epsilon", type=float, default=1e-6, help="0 disables duplicate filtering.")
    parser.add_argument("--allow-invalid", action="store_true", help="Forward invalid goals too. Default is to drop them.")
    parser.add_argument("--stats-period", type=float, default=5.0)
    return parser.parse_args()


def goal_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class GoalGateway:
    def __init__(self, args, source_node: Node, target_node: Node):
        self.args = args
        self.source_node = source_node
        self.target_node = target_node
        self.publisher = target_node.create_publisher(Float32MultiArray, args.output_topic, goal_qos())
        self.subscription = source_node.create_subscription(
            Float32MultiArray,
            args.input_topic,
            self.goal_callback,
            goal_qos(),
        )
        self.received = 0
        self.forwarded = 0
        self.dropped = 0
        self.last_forward_time = 0.0
        self.last_stats_time = time.monotonic()
        self.last_payload: np.ndarray | None = None

    def should_drop(self, payload: np.ndarray) -> tuple[bool, str]:
        if payload.size < GOAL_PAYLOAD_SIZE:
            return True, f"short payload: {payload.size} < {GOAL_PAYLOAD_SIZE}"

        goal = decode_goal(payload[:GOAL_PAYLOAD_SIZE])
        if not goal.valid and not self.args.allow_invalid:
            return True, "invalid goal"
        if goal.confidence < self.args.min_confidence:
            return True, f"low confidence: {goal.confidence:.3f}"

        now = time.monotonic()
        if self.args.max_hz > 0.0 and now - self.last_forward_time < 1.0 / self.args.max_hz:
            return True, "rate limited"

        if self.args.dedupe_epsilon > 0.0 and self.last_payload is not None:
            if np.allclose(payload[:GOAL_PAYLOAD_SIZE], self.last_payload, atol=self.args.dedupe_epsilon, rtol=0.0):
                return True, "duplicate"

        return False, ""

    def maybe_log_stats(self):
        now = time.monotonic()
        if self.args.stats_period <= 0.0 or now - self.last_stats_time < self.args.stats_period:
            return
        self.source_node.get_logger().info(
            f"goal gateway stats: received={self.received}, forwarded={self.forwarded}, dropped={self.dropped}"
        )
        self.last_stats_time = now

    def goal_callback(self, msg: Float32MultiArray):
        self.received += 1
        payload = np.asarray(msg.data, dtype=np.float64)
        drop, reason = self.should_drop(payload)
        if drop:
            self.dropped += 1
            if self.dropped <= 3 or self.dropped % 50 == 0:
                self.source_node.get_logger().info(f"dropped goal: {reason}")
            self.maybe_log_stats()
            return

        out = Float32MultiArray()
        out.data = [float(v) for v in payload[:GOAL_PAYLOAD_SIZE]]
        self.publisher.publish(out)
        self.forwarded += 1
        self.last_forward_time = time.monotonic()
        self.last_payload = payload[:GOAL_PAYLOAD_SIZE].copy()

        goal = decode_goal(out.data)
        self.source_node.get_logger().info(
            "forwarded goal "
            f"robot={goal.robot_index} conf={goal.confidence:.2f} "
            f"pick=({goal.pick.x:+.3f},{goal.pick.y:+.3f},{goal.pick.z:+.3f},{goal.pick.yaw:+.3f}) "
            f"place=({goal.place.x:+.3f},{goal.place.y:+.3f},{goal.place.z:+.3f},{goal.place.yaw:+.3f})"
        )
        self.maybe_log_stats()


def main():
    args = parse_args()
    source_context = Context()
    target_context = Context()
    rclpy.init(context=source_context, domain_id=args.source_domain)
    rclpy.init(context=target_context, domain_id=args.target_domain)

    source_node = Node(
        "goal_gateway_source",
        context=source_context,
        enable_rosout=True,
        start_parameter_services=False,
    )
    target_node = Node(
        "goal_gateway_target",
        context=target_context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    executor = SingleThreadedExecutor(context=source_context)
    executor.add_node(source_node)

    gateway = GoalGateway(args, source_node, target_node)
    source_node.get_logger().info(
        f"bridging domain {args.source_domain}:{args.input_topic} "
        f"-> domain {args.target_domain}:{args.output_topic}"
    )
    source_node.get_logger().info(
        "only pick/place Float32MultiArray goals are bridged; images/debug/tf/joint_states stay local"
    )

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        del gateway
        executor.remove_node(source_node)
        source_node.destroy_node()
        target_node.destroy_node()
        rclpy.shutdown(context=source_context)
        rclpy.shutdown(context=target_context)


if __name__ == "__main__":
    main()
