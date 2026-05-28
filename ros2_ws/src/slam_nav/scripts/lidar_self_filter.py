#!/usr/bin/env python3
"""iw_hub_ROS 2D LiDAR self/dolly-detection 필터 (01/02 공용).

배경:
  - 차체 자체의 일부(휠/마운트)가 LiDAR에 ~0.4m 거리로 dead zone처럼 잡힘
    (이전 슬램_navigation §6 진단치 — back LiDAR `-115°~-90°` 영역).
  - 2026-05-26 사용자 요구: dolly가 robot 전/후/좌/우 어디든 배치되어 있을 수 있고
    dolly의 바퀴/기둥이 Nav2 obstacle_layer에 marking되면 dolly 운반에 문제 발생 →
    dolly 영역 무시 필요.

해결 (multi-window):
  각 LiDAR마다 여러 (각도, 거리) 윈도우를 정의해 inf로 변환.
  - CHASSIS: 차체 자체 dead zone. 항상 활성.
  - DOLLY: Dolly가 차체 위/주변에 있을 때 활성. ROS param `dolly_loaded`로 toggle.
  - APPROACH: dolly 진입 모드. ROS param `pickup_approach`로 toggle.

필터링된 scan은 _filtered 토픽으로 발행. nav2 obstacle_layer가 이걸 구독.

사용:
  python3 lidar_self_filter.py --robot iw_hub_ROS_01
  python3 lidar_self_filter.py --robot iw_hub_ROS_02

이전 robot{1,2}_lidar_self_filter.py 두 파일을 통합 (2026-05-27).
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from sensor_msgs.msg import LaserScan


# ────────────────────────────────────────────────────────────────────────────
# 필터 윈도우 정의 — (angle_min_deg, angle_max_deg, max_range_m)
# 각도는 LiDAR frame 기준. angle_increment > 0 가정.
# ────────────────────────────────────────────────────────────────────────────
WINDOWS_FRONT_CHASSIS = [(-110.0, -100.0, 0.55)]
WINDOWS_BACK_CHASSIS = [(-115.0, -90.0, 0.6)]

# dolly_loaded=True: dolly가 robot에 적재된 상태. robot 주변 360° ≤2.0m 무시.
WINDOWS_FRONT_DOLLY = [(-180.0, 180.0, 2.0)]
WINDOWS_BACK_DOLLY = [(-180.0, 180.0, 2.0)]

# pickup_approach=True: dolly 진입 모드. LiDAR frame의 0° ±120° 영역 ≤4.0m 무시.
#   FRONT LiDAR 0° = robot 전방 (전진 픽업 시)
#   BACK  LiDAR 0° = robot 후방 (후진 픽업 시) — 사용자 시나리오
# 2026-05-26: ±90°·3m로도 dolly 바퀴 잔존 → ±120°·4m로 확대.
WINDOWS_FRONT_APPROACH = [(-120.0, 120.0, 4.0)]
WINDOWS_BACK_APPROACH = [(-120.0, 120.0, 4.0)]


def _deg_windows_to_rad(windows):
    return [(math.radians(a0), math.radians(a1), r) for (a0, a1, r) in windows]


class LidarSelfFilter(Node):
    def __init__(self, robot_ns: str):
        bare = robot_ns.lstrip("/")
        self._front_in = f"/{bare}/front_2d_lidar/scan"
        self._front_out = f"/{bare}/front_2d_lidar/scan_filtered"
        self._back_in = f"/{bare}/back_2d_lidar/scan"
        self._back_out = f"/{bare}/back_2d_lidar/scan_filtered"

        super().__init__(f"{bare}_lidar_self_filter")

        self.declare_parameter(
            "dolly_loaded",
            True,
            ParameterDescriptor(
                description="True면 dolly가 robot에 적재된 운반 모드 — 360° ≤2.0m 무시. "
                "False면 외부 dolly가 obstacle로 보임 (픽업 정렬 단계용)."
            ),
        )
        self.declare_parameter(
            "pickup_approach",
            False,
            ParameterDescriptor(
                description="True면 dolly 진입 모드 — LiDAR 진행 방향 ±120° ≤4.0m 무시. "
                "후진/전진 픽업 진입 단계에서 dolly 바퀴/기둥 detect 무시하고 plan 가능."
            ),
        )
        self._dolly_loaded = self.get_parameter("dolly_loaded").value
        self._pickup_approach = self.get_parameter("pickup_approach").value

        self._front_chassis = _deg_windows_to_rad(WINDOWS_FRONT_CHASSIS)
        self._back_chassis = _deg_windows_to_rad(WINDOWS_BACK_CHASSIS)
        self._front_dolly = _deg_windows_to_rad(WINDOWS_FRONT_DOLLY)
        self._back_dolly = _deg_windows_to_rad(WINDOWS_BACK_DOLLY)
        self._front_approach = _deg_windows_to_rad(WINDOWS_FRONT_APPROACH)
        self._back_approach = _deg_windows_to_rad(WINDOWS_BACK_APPROACH)

        self.add_on_set_parameters_callback(self._on_set_params)

        self._front_pub = self.create_publisher(LaserScan, self._front_out, 10)
        self._back_pub = self.create_publisher(LaserScan, self._back_out, 10)

        self.create_subscription(
            LaserScan, self._front_in, lambda m: self._handle(m, self._front_pub, "front"), 10
        )
        self.create_subscription(
            LaserScan, self._back_in, lambda m: self._handle(m, self._back_pub, "back"), 10
        )

        self._log_config(bare)

    def _on_set_params(self, params):
        for p in params:
            if p.name == "dolly_loaded":
                self._dolly_loaded = bool(p.value)
                self.get_logger().info(f"dolly_loaded 전환 → {self._dolly_loaded}")
            elif p.name == "pickup_approach":
                self._pickup_approach = bool(p.value)
                self.get_logger().info(f"pickup_approach 전환 → {self._pickup_approach}")
        return SetParametersResult(successful=True)

    def _log_config(self, bare: str):
        f_w, b_w = self._active_windows("front"), self._active_windows("back")

        def fmt(ws):
            return " ".join(
                f"[{math.degrees(a0):+.0f}°,{math.degrees(a1):+.0f}°]≤{r:.2f}m"
                for (a0, a1, r) in ws
            ) or "(none)"

        self.get_logger().info(
            f"LiDAR self-filter 시작 (robot={bare}, dolly_loaded={self._dolly_loaded}, "
            f"pickup_approach={self._pickup_approach})  "
            f"FRONT: {fmt(f_w)}  BACK: {fmt(b_w)}"
        )

    def _active_windows(self, side: str):
        chassis = self._front_chassis if side == "front" else self._back_chassis
        dolly = (self._front_dolly if side == "front" else self._back_dolly) if self._dolly_loaded else []
        approach = (self._front_approach if side == "front" else self._back_approach) if self._pickup_approach else []
        return chassis + dolly + approach

    def _handle(self, msg: LaserScan, pub, side: str):
        windows = self._active_windows(side)
        if not windows:
            pub.publish(msg)
            return

        ranges = list(msg.ranges)
        inf = float("inf")
        n = len(ranges)

        for (a_min, a_max, max_r) in windows:
            i_lo = max(0, int(math.ceil((a_min - msg.angle_min) / msg.angle_increment)))
            i_hi = min(
                n,
                int(math.floor((a_max - msg.angle_min) / msg.angle_increment)) + 1,
            )
            for i in range(i_lo, i_hi):
                r = ranges[i]
                if math.isfinite(r) and msg.range_min <= r < max_r:
                    ranges[i] = inf

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        pub.publish(out)


def main():
    parser = argparse.ArgumentParser(description="iw_hub_ROS LiDAR self/dolly filter")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        help="robot bare name (iw_hub_ROS_01 or iw_hub_ROS_02)")
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = LidarSelfFilter(robot_ns=args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
