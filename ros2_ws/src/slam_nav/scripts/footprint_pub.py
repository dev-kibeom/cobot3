#!/usr/bin/env python3
"""iw_hub_ROS footprint marker (01/02 공용) — 차체 위치/방향을 초록색 화살표로 시각화.

설계 (사용자 요구 2026-05-24):
  - 화살표 크기 = 차체 (L 1.064m × W 0.728m × H 0.330m, idealworks iw.hub 공식 스펙)
  - 화살표 방향 = +X = 카메라 시선 (전방)
  - 색: 초록
  - frame_id: chassis (차체 중앙 frame)
  - frame_locked: True (차체 이동·회전 자동 추적)

발행: /<robot>/footprint_marker (visualization_msgs/MarkerArray, TRANSIENT_LOCAL)

사용:
  python3 footprint_pub.py --robot iw_hub_ROS_01
  python3 footprint_pub.py --robot iw_hub_ROS_02

이전 robot{1,2}_footprint_pub.py 두 파일을 통합 (2026-05-27).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


# iw.hub 공식 스펙 (idealworks).
BODY_LENGTH = 1.064   # +X 방향 (전방 카메라)
BODY_WIDTH = 0.728    # ±Y 방향
BODY_HEIGHT = 0.330   # Z

ARROW_COLOR = (0.10, 0.85, 0.30, 0.85)  # 초록 RGBA
FRAME = "chassis"  # 차체 중앙 frame_locked


class FootprintArrowPublisher(Node):
    def __init__(self, robot_ns: str):
        bare = robot_ns.lstrip("/")
        self._namespace = f"{bare}_footprint"
        self._topic = f"/{bare}/footprint_marker"

        super().__init__(f"{bare}_footprint_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, self._topic, qos)
        # 1초마다 재발행 (TRANSIENT_LOCAL이라 late subscriber 받음 보장 + chassis frame 이동 추적).
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            f"footprint marker publisher 시작 (robot={bare}) — frame={FRAME}, "
            f"size {BODY_LENGTH}×{BODY_WIDTH}×{BODY_HEIGHT}m, 초록 ARROW +X(전방)"
        )

    def _publish(self):
        ma = MarkerArray()
        arrow = Marker()
        arrow.header.frame_id = FRAME
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = self._namespace
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.frame_locked = True

        # Marker.ARROW의 pose+scale 방식:
        #   scale.x = arrow length (꼬리에서 끝까지 = 차체 length)
        #   scale.y = arrow shaft 폭 (= 차체 width)
        #   scale.z = arrow head 폭 (= 차체 height)
        arrow.pose.position.x = -BODY_LENGTH / 2.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = BODY_HEIGHT / 2.0  # 차체 중간 높이
        arrow.pose.orientation.x = 0.0
        arrow.pose.orientation.y = 0.0
        arrow.pose.orientation.z = 0.0
        arrow.pose.orientation.w = 1.0  # identity → +X 방향

        arrow.scale.x = BODY_LENGTH
        arrow.scale.y = BODY_WIDTH
        arrow.scale.z = BODY_HEIGHT

        r, g, b, a = ARROW_COLOR
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, a

        ma.markers.append(arrow)
        self.pub.publish(ma)


def main():
    parser = argparse.ArgumentParser(description="iw_hub_ROS footprint marker publisher")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        help="robot bare name (iw_hub_ROS_01 or iw_hub_ROS_02)")
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = FootprintArrowPublisher(robot_ns=args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
