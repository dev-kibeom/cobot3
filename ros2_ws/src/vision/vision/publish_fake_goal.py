#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from .vision_contracts import PickPlaceGoal, Pose2D, encode_goal, EE_SUCTION
except Exception:
    from vision_contracts import PickPlaceGoal, Pose2D, encode_goal, EE_SUCTION


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a fake pick/place goal for M0609 testing")
    parser.add_argument("--topic", default="/m0609_vision/pick_place_goal")
    parser.add_argument("--robot-index", type=int, default=1)
    parser.add_argument("--confidence", type=float, default=0.95)
    parser.add_argument("--pick", default="0.35,0.00,0.05,0.0", help="x,y,z,yaw")
    parser.add_argument("--place", default="0.55,-0.35,0.03,0.0", help="x,y,z,yaw")
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dyaw", type=float, default=0.0)
    parser.add_argument("--once", action="store_true", help="Publish once and exit")
    parser.add_argument("--hz", type=float, default=0.5)
    return parser.parse_args()


def make_goal_from_args(args) -> PickPlaceGoal:
    p = [float(v) for v in args.pick.split(",")]
    q = [float(v) for v in args.place.split(",")]
    pick = Pose2D(p[0], p[1], p[2], p[3])
    place = Pose2D(q[0], q[1], q[2], q[3])
    return PickPlaceGoal(valid=True, robot_index=args.robot_index, confidence=args.confidence, pick=pick, place=place, dx=args.dx, dy=args.dy, dyaw=args.dyaw, ee_type=EE_SUCTION)


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("m0609_fake_goal_publisher")
    pub = node.create_publisher(Float32MultiArray, args.topic, 10)

    goal = make_goal_from_args(args)
    payload = encode_goal(goal)
    msg = Float32MultiArray()
    msg.data = [float(v) for v in payload]

    node.get_logger().info(f"publishing fake goal to {args.topic}: pick=({goal.pick.x:.3f},{goal.pick.y:.3f},{goal.pick.z:.3f}) place=({goal.place.x:.3f},{goal.place.y:.3f},{goal.place.z:.3f})")

    try:
        if args.once:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            return

        period = 1.0 / max(0.001, args.hz)
        while rclpy.ok():
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
