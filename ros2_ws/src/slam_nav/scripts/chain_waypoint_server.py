#!/usr/bin/env python3
"""DB(다른 PC)에서 PoseArray로 waypoint sequence 받아 chain 실행 — robot 01/02 공용.

사용:
  python3 chain_waypoint_server.py --robot iw_hub_ROS_01
  python3 chain_waypoint_server.py --robot iw_hub_ROS_02

토픽: `/<robot>/chain_waypoints` (geometry_msgs/PoseArray)
인코딩:
  - position.x, position.y: 목표 좌표 (world frame)
  - orientation: 목표 yaw (quaternion)
  - **position.z**: reverse 플래그 (0.0=forward, 1.0=후진)

PC-D(DB)가 두 robot에 동시에 명령 보내려면 각 robot topic에 따로 publish:
  ros2 topic pub --once /iw_hub_ROS_01/chain_waypoints geometry_msgs/PoseArray "..."  # 명령 A
  ros2 topic pub --once /iw_hub_ROS_02/chain_waypoints geometry_msgs/PoseArray "..."  # 명령 B

PC-C(이 launch 실행 PC)는 두 chain_waypoint_server 인스턴스(robot1/robot2)로 분리 처리.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import PoseArray

from chain_goal import ChainGoalSender, quat_to_yaw


class ChainWaypointServer(ChainGoalSender):
    def __init__(self, robot_ns):
        super().__init__(robot_ns=robot_ns)
        self._queue = []
        topic = f"{self._robot_ns}/chain_waypoints"
        self.create_subscription(PoseArray, topic, self._on_waypoints, 10)
        self.get_logger().info(f"chain server 시작 — '{topic}' PoseArray 대기")
        self.get_logger().info(
            "  Pose 인코딩: position.x/y=좌표, orientation=yaw, position.z=reverse(0/1)")

    def _on_waypoints(self, msg: PoseArray):
        if self._queue:
            self.get_logger().warn(
                f"이미 sequence 대기 중 ({len(self._queue)}) — 새 요청 무시")
            return
        wp_list = []
        for i, p in enumerate(msg.poses):
            x, y = p.position.x, p.position.y
            yaw_rad = quat_to_yaw(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w)
            reverse = bool(round(p.position.z))
            wp_list.append((f"W{i}", x, y, math.degrees(yaw_rad), reverse))
        mode_str = ", ".join(
            f"{w[0]}({w[1]:.2f},{w[2]:.2f},yaw={w[3]:.0f}°,{'REV' if w[4] else 'FWD'})"
            for w in wp_list)
        self.get_logger().info(f"새 sequence ({len(wp_list)}개): {mode_str}")
        self._queue.append(wp_list)


def main():
    parser = argparse.ArgumentParser(description="chain waypoint server (01/02 공용)")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        choices=["iw_hub_ROS_01", "iw_hub_ROS_02"])
    # ros2 launch가 --ros-args 등 부가 인자를 같이 전달 → parse_known_args로 무시
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = ChainWaypointServer(robot_ns=f"/{args.robot}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._queue:
                wp_list = node._queue.pop(0)
                node.get_logger().info("sequence 실행 시작")
                ok = node.execute_sequence(wp_list)
                node.get_logger().info(f"sequence {'완료' if ok else '중단'}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
