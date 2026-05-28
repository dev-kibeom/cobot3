#!/usr/bin/env python3
"""iw_hub_ROS world-pose static TF bridge (01/02 공용).

Isaac Sim의 `world_pose_publisher` OmniGraph(basic1.usda 안)가 /<robot>/tf로
`world → <robot>/world_pose` 를 라이브로 발행. 이 노드는 첫 메시지에서 spawn
pose를 캡쳐해 그 값으로 static TF `world → <robot>/map`을
/<robot>/tf_static에 발행.

효과: 사용자가 Isaac Sim Stage에서 로봇을 옮긴 뒤 launch 재실행만 하면 warehouse
절대맵(basic2.yaml의 world frame)과 SLAM/odom frame이 자동 정렬됨.

TF chain (이 노드 발행 + slam_nav launch의 다른 static):
  world → <robot>/map      (이 노드, static)
  <robot>/map → odom        (launch의 static_transform_publisher, identity)
  <robot>/odom → base_link  (USD wheel encoder, live)

사용:
  python3 world_tf_bridge.py --robot iw_hub_ROS_01
  python3 world_tf_bridge.py --robot iw_hub_ROS_02

이전 robot{1,2}_world_tf_bridge.py 두 파일을 통합 (2026-05-27).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


SOURCE_FRAME = "world"


class WorldToRobotMapBridge(Node):
    def __init__(self, robot_ns: str):
        # robot_ns 정규화: leading slash 제거 → bare name
        bare = robot_ns.lstrip("/")
        self._bare = bare
        self._isaac_live_child = f"{bare}/world_pose"
        self._slam_map_frame = f"{bare}/map"

        super().__init__(f"world_to_{bare}_map_bridge")
        self.captured_transform = None

        # /tf == /<robot>/tf via launch remap.
        self.tf_sub = self.create_subscription(
            TFMessage, "/tf", self._tf_cb, 100
        )
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)

        # 2초마다 재발행 — 늦게 뜨는 subscriber 보장.
        self.create_timer(2.0, self._republish_if_have)
        self.get_logger().info(
            f"Waiting for {SOURCE_FRAME} -> {self._isaac_live_child} on /{bare}/tf ..."
        )

    def _tf_cb(self, msg: TFMessage):
        if self.captured_transform is not None:
            return
        for t in msg.transforms:
            if (t.header.frame_id == SOURCE_FRAME
                    and t.child_frame_id == self._isaac_live_child):
                self.captured_transform = t.transform
                tx = t.transform.translation
                rot = t.transform.rotation
                self.get_logger().info(
                    f"Captured {self._bare} spawn pose: "
                    f"t=({tx.x:.3f}, {tx.y:.3f}, {tx.z:.3f}), "
                    f"q=({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})"
                )
                self._publish_static()
                return

    def _publish_static(self):
        if self.captured_transform is None:
            return
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = SOURCE_FRAME
        st.child_frame_id = self._slam_map_frame
        st.transform = self.captured_transform
        self.static_pub.publish(TFMessage(transforms=[st]))

    def _republish_if_have(self):
        if self.captured_transform is not None:
            self._publish_static()


def main():
    parser = argparse.ArgumentParser(description="world → <robot>/map static TF bridge")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        help="robot bare name (iw_hub_ROS_01 or iw_hub_ROS_02)")
    # ros2 launch가 --ros-args 등을 같이 전달하므로 parse_known_args 사용
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = WorldToRobotMapBridge(robot_ns=args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
