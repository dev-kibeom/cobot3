#!/usr/bin/env python3
"""두 iw_hub_ROS 간 회피(양보) 코디네이터.

배경:
  Nav2 obstacle_layer가 LiDAR로 상대 로봇을 보긴 하지만, 마주 접근 시 양쪽 모두 우회를
  시도해 deadlock 또는 충돌 발생. 명시적 좌표 교환 기반 양보 로직이 필요.

정책 (단순 robot_id 우선순위):
  - 두 로봇 거리 < YIELD_DISTANCE_M(2.0m) → robot_id 큰 쪽 (iw_hub_ROS_02) 양보.
    /iw_hub_ROS_02/cmd_vel에 Twist(0)을 25Hz로 jam → velocity_smoother(20Hz)보다 우세
    → 차체 정지. Nav2 작업(NavigateToPose)은 그대로, cmd_vel만 차단.
  - 거리 > RELEASE_DISTANCE_M(3.0m) → 해제. 02 Nav2가 자유롭게 cmd_vel 발행 재개.
  - hysteresis (2.0 → 3.0)로 임계값 근처에서 진동 방지.

TF chain 가정:
  world → iw_hub_ROS_NN/map         (world_tf_bridge가 발행한 static)
  iw_hub_ROS_NN/map → odom          (identity static)
  iw_hub_ROS_NN/odom → base_link    (USD wheel encoder, live)

world → base_link = map_origin ∘ odom_pose (map→odom identity라 합성 단순).

향후 개선 후보:
  - 단순 정지 → 옆으로 비키기 동작 (NavigateToPose cancel + 임시 좌표 → 통과 후 원경로)
  - 우선순위 규칙: idle 여부, 작업 긴급도, 진행 방향
  - 두 로봇 모두 movement 시 작은 robot_id가 통과 priority
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Transform, Twist
from tf2_msgs.msg import TFMessage


YIELD_DISTANCE_M = 2.0      # 이 거리 안이면 양보 발동
RELEASE_DISTANCE_M = 3.0    # 이 거리 넘어가면 양보 해제 (hysteresis)
JAM_RATE_HZ = 25.0          # cmd_vel jam 발행 빈도 (velocity_smoother 20Hz보다 우세)


def _compose_xy(a: Transform, b: Transform):
    """SE(2) transform composition (2D). a ∘ b 의 translation (x, y)."""
    qz, qw = a.rotation.z, a.rotation.w
    yaw_a = 2.0 * math.atan2(qz, qw)
    bx, by = b.translation.x, b.translation.y
    x = a.translation.x + bx * math.cos(yaw_a) - by * math.sin(yaw_a)
    y = a.translation.y + bx * math.sin(yaw_a) + by * math.cos(yaw_a)
    return x, y


class YieldCoordinator(Node):
    def __init__(self):
        super().__init__("robot_yield_coordinator")

        self._map_origin = {"01": None, "02": None}
        self._odom_pose = {"01": None, "02": None}

        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/iw_hub_ROS_01/tf_static",
            lambda m: self._tf_static_cb(m, "01"), static_qos
        )
        self.create_subscription(
            TFMessage, "/iw_hub_ROS_02/tf_static",
            lambda m: self._tf_static_cb(m, "02"), static_qos
        )
        self.create_subscription(
            TFMessage, "/iw_hub_ROS_01/tf",
            lambda m: self._tf_cb(m, "01"), 100
        )
        self.create_subscription(
            TFMessage, "/iw_hub_ROS_02/tf",
            lambda m: self._tf_cb(m, "02"), 100
        )

        self._cmd_pub = {
            "01": self.create_publisher(Twist, "/iw_hub_ROS_01/cmd_vel", 10),
            "02": self.create_publisher(Twist, "/iw_hub_ROS_02/cmd_vel", 10),
        }

        self._yielding_robot = None
        self._last_dist_log = None

        self.create_timer(1.0 / JAM_RATE_HZ, self._tick)
        self.get_logger().info(
            f"yield coordinator 시작 — dist < {YIELD_DISTANCE_M}m → iw_hub_ROS_02 정지 jam, "
            f"dist > {RELEASE_DISTANCE_M}m → 해제 (hysteresis)"
        )

    def _tf_static_cb(self, msg: TFMessage, robot: str):
        target = f"iw_hub_ROS_{robot}/map"
        for t in msg.transforms:
            if t.header.frame_id == "world" and t.child_frame_id == target:
                self._map_origin[robot] = t.transform

    def _tf_cb(self, msg: TFMessage, robot: str):
        target_parent = f"iw_hub_ROS_{robot}/odom"
        for t in msg.transforms:
            if t.child_frame_id == "base_link" and t.header.frame_id == target_parent:
                self._odom_pose[robot] = t.transform

    def _world_xy(self, robot: str):
        mo = self._map_origin[robot]
        op = self._odom_pose[robot]
        if mo is None or op is None:
            return None
        return _compose_xy(mo, op)

    def _tick(self):
        p1 = self._world_xy("01")
        p2 = self._world_xy("02")
        if p1 is None or p2 is None:
            return
        dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

        # 1m 이상 변화 시만 거리 로그 (스팸 방지).
        if self._last_dist_log is None or abs(dist - self._last_dist_log) > 1.0:
            self.get_logger().info(
                f"두 로봇 거리 {dist:.2f}m (양보 중: {self._yielding_robot or '없음'})"
            )
            self._last_dist_log = dist

        if self._yielding_robot is None:
            if dist < YIELD_DISTANCE_M:
                self._yielding_robot = "02"
                self.get_logger().warn(
                    f"양보 발동 — dist={dist:.2f}m < {YIELD_DISTANCE_M}m, iw_hub_ROS_02 정지"
                )
        else:
            if dist > RELEASE_DISTANCE_M:
                self.get_logger().info(
                    f"양보 해제 — dist={dist:.2f}m > {RELEASE_DISTANCE_M}m, "
                    f"iw_hub_ROS_{self._yielding_robot} 재개"
                )
                self._yielding_robot = None

        if self._yielding_robot is not None:
            self._cmd_pub[self._yielding_robot].publish(Twist())


def main():
    rclpy.init()
    node = YieldCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
