#!/usr/bin/env python3
"""DB(PC-D)에서 PoseArray로 waypoint sequence 받아 chain 실행 — robot 01/02 공용.

사용:
  python3 chain_waypoint_server.py --robot iw_hub_ROS_01
  python3 chain_waypoint_server.py --robot iw_hub_ROS_02

토픽: `/<robot>/chain_waypoints` (geometry_msgs/PoseArray)

Pose 인코딩 (2026-05-27 확장 — reverse_code 0/1/2/3):
  - position.x, position.y: 목표 좌표 (world frame, m)
  - orientation:            목표 yaw (quaternion)
  - **position.z** (round to int 0/1/2/3):
        0 — forward (NavigateToPose)
        1 — reverse (drive_backward, lift 동작 없음)
        2 — reverse + 도착 후 lift_up (0.04) + 5초 대기   (dolly 픽업)
        3 — reverse + 도착 후 lift_down (0.0) + 5초 대기  (dolly drop)

작업 ID 전달 (옵션):
  PoseArray.header.frame_id 를 "task_id:<숫자>" 형태로 채우면 sequence 완료 시
  `/<robot>/chain_done` (std_msgs/String) 에 `task_done:<숫자>` publish.
  PC-D dispatcher가 이걸 구독해 DB 상태 갱신.

PC-D 호출 예시:
  ros2 topic pub --once /iw_hub_ROS_01/chain_waypoints geometry_msgs/PoseArray \\
    "{header: {frame_id: 'task_id:42'}, poses: [...10 poses with z=0/1/2/3...]}"

PC-C는 launch 시 두 chain_waypoint_server 인스턴스(robot1/robot2)로 자동 분리 처리.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import PoseArray

from chain_goal import ChainGoalSender, quat_to_yaw


# reverse_code 표시명
RC_LABEL = {0: "FWD", 1: "REV", 2: "REV+UP", 3: "REV+DOWN"}


class ChainWaypointServer(ChainGoalSender):
    def __init__(self, robot_ns):
        super().__init__(robot_ns=robot_ns)
        self._queue = []        # list of (wp_list, task_id)
        topic = f"{self._robot_ns}/chain_waypoints"
        self.create_subscription(PoseArray, topic, self._on_waypoints, 10)
        self.get_logger().info(f"chain server 시작 — '{topic}' PoseArray 대기")
        self.get_logger().info(
            "  Pose 인코딩: position.x/y=좌표, orientation=yaw, "
            "position.z=reverse_code(0=FWD/1=REV/2=REV+lift_up/3=REV+lift_down)")
        self.get_logger().info(
            "  task_id (옵션): header.frame_id에 'task_id:<n>' 으로 전달 시 "
            "완료 시 chain_done topic으로 PC-D에 보고")

    def _parse_task_id(self, msg: PoseArray) -> str:
        """header.frame_id에서 task_id 파싱 (형식: 'task_id:<숫자>')."""
        f = msg.header.frame_id or ""
        if f.startswith("task_id:"):
            return f.split(":", 1)[1]
        return ""

    def _on_waypoints(self, msg: PoseArray):
        if self._queue:
            self.get_logger().warn(
                f"이미 sequence 대기 중 ({len(self._queue)}) — 새 요청 무시")
            return
        task_id = self._parse_task_id(msg)
        wp_list = []
        for i, p in enumerate(msg.poses):
            x, y = p.position.x, p.position.y
            yaw_rad = quat_to_yaw(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w)
            # reverse_code: 0/1/2/3 (round → int)
            rc = int(round(p.position.z))
            if rc < 0 or rc > 3:
                self.get_logger().warn(
                    f"pose[{i}] position.z={p.position.z} → 비정상 reverse_code, 0(forward)로 처리")
                rc = 0
            wp_list.append((f"W{i}", x, y, math.degrees(yaw_rad), rc))
        mode_str = ", ".join(
            f"{w[0]}({w[1]:.2f},{w[2]:.2f},yaw={w[3]:.0f}°,{RC_LABEL[w[4]]})"
            for w in wp_list)
        tid_str = f" task_id={task_id}" if task_id else ""
        self.get_logger().info(f"새 sequence ({len(wp_list)}개){tid_str}: {mode_str}")
        self._queue.append((wp_list, task_id))


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
                wp_list, task_id = node._queue.pop(0)
                node.get_logger().info(
                    f"sequence 실행 시작 (task_id={task_id or '<none>'})")
                ok = node.execute_sequence(wp_list, task_id=task_id)
                node.get_logger().info(f"sequence {'완료' if ok else '중단'}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
