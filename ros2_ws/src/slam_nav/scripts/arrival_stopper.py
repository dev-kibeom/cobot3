#!/usr/bin/env python3
"""iw_hub_ROS 도착 처리 daemon (01/02 공용).

기능:
  1. /<robot>/navigate_to_pose/_action/status 구독.
  2. SUCCEEDED 받으면:
     - /<robot>/cmd_vel에 Twist(0) 10회 burst + 1초간 20Hz 안전망 발행 (USD
       differential_drive에 cmd_vel watchdog 없어 잔존 속도가 남는 문제 차단).
     - /<robot>/goal_marker에 DELETEALL 발행 → random 좌표 marker 제거.
  3. ABORTED·CANCELED 시에도 cmd_vel 정지만 처리 (marker는 유지 — 사용자가 실패 지점 시각 확인).

전제:
  - Nav2가 stopped_goal_checker.yaw_goal_tolerance(0.3rad≈17°)까지 yaw 정렬해 SUCCEEDED 보고.

사용:
  python3 arrival_stopper.py --robot iw_hub_ROS_01
  python3 arrival_stopper.py --robot iw_hub_ROS_02

이전 robot{1,2}_arrival_stopper.py 두 파일을 통합 (2026-05-27).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
TERMINAL_STATUSES = (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED)
STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
STOP_BURST_COUNT = 10
STOP_BURST_RATE_HZ = 20


class ArrivalStopper(Node):
    def __init__(self, robot_ns: str):
        bare = robot_ns.lstrip("/")
        self._status_topic = f"/{bare}/navigate_to_pose/_action/status"
        self._cmd_vel_topic = f"/{bare}/cmd_vel"
        self._goal_marker_topic = f"/{bare}/goal_marker"
        self._marker_ns = f"{bare}_goal"

        super().__init__(f"{bare}_arrival_stopper")
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_marker_pub = self.create_publisher(MarkerArray, self._goal_marker_topic, marker_qos)

        self.create_subscription(GoalStatusArray, self._status_topic, self._status_cb, 10)
        self._handled = set()  # (goal_uuid, status) 중복 처리 차단
        self._safety_burst_count = 0
        self.get_logger().info(
            f"arrival_stopper 시작 (robot={bare}) — terminal status(SUCCEEDED/CANCELED/ABORTED) 시 "
            f"cmd_vel 정지, SUCCEEDED 시 goal_marker 제거"
        )

    def _status_cb(self, msg: GoalStatusArray):
        for st in msg.status_list:
            if st.status not in TERMINAL_STATUSES:
                continue
            key = (bytes(st.goal_info.goal_id.uuid), st.status)
            if key in self._handled:
                continue
            self._handled.add(key)
            self._publish_stop(st.status)
            if st.status == STATUS_SUCCEEDED:
                self._clear_goal_marker()

    def _publish_stop(self, status_code: int):
        name = STATUS_NAMES.get(status_code, f"status={status_code}")
        self.get_logger().info(
            f"Goal {name} — Twist(0) {STOP_BURST_COUNT}회 burst + 1초 safety로 정지"
        )
        stop = Twist()
        for _ in range(STOP_BURST_COUNT):
            self._cmd_vel_pub.publish(stop)
        self._safety_burst_count = STOP_BURST_RATE_HZ
        self.create_timer(1.0 / STOP_BURST_RATE_HZ, self._safety_tick)

    def _safety_tick(self):
        if self._safety_burst_count <= 0:
            return
        self._cmd_vel_pub.publish(Twist())
        self._safety_burst_count -= 1

    def _clear_goal_marker(self):
        ma = MarkerArray()
        m = Marker()
        m.ns = self._marker_ns
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self._goal_marker_pub.publish(ma)
        self.get_logger().info("goal_marker DELETEALL 발행 — random 좌표 marker 제거")


def main():
    parser = argparse.ArgumentParser(description="iw_hub_ROS arrival stopper daemon")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        help="robot bare name (iw_hub_ROS_01 or iw_hub_ROS_02)")
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = ArrivalStopper(robot_ns=args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
