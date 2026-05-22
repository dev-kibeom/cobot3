"""
ros2_supabase_bridge.py — ROS2 토픽 → Supabase 브릿지
=======================================================
Isaac Sim에서 오는 로봇 위치/상태 토픽을 받아
Supabase robot_state 테이블에 실시간 업데이트합니다.

실행: python3 ros2_supabase_bridge.py
필요: pip install supabase
      ROS2 환경 소스: source /opt/ros/humble/setup.bash
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import json
import threading
from supabase import create_client, Client

# =============================================================================
# 설정
# =============================================================================

SUPABASE_URL = "https://xxxx.supabase.co"   # 여기에 입력
SUPABASE_KEY = "eyJhbGciOiJIUzI1..."        # 여기에 입력

# 로봇 ID 매핑 — 토픽 이름 → robot_id
# 맵 완성 후 실제 토픽 이름으로 수정
ROBOT_TOPIC_MAP = {
    "/carter_1/odom": {"robot_id": 1, "robot_type": 1},  # Carter 1
    "/carter_2/odom": {"robot_id": 2, "robot_type": 1},  # Carter 2
    "/arm_1/odom":    {"robot_id": 3, "robot_type": 2},  # M0609 1
    "/arm_2/odom":    {"robot_id": 4, "robot_type": 2},  # M0609 2
    "/arm_3/odom":    {"robot_id": 5, "robot_type": 2},  # M0609 3
    "/arm_4/odom":    {"robot_id": 6, "robot_type": 2},  # M0609 4
    "/arm_5/odom":    {"robot_id": 7, "robot_type": 2},  # M0609 5
    "/arm_6/odom":    {"robot_id": 8, "robot_type": 2},  # M0609 6
}

# robot_status 코드
STATUS = {
    "idle":    1,
    "working": 2,
    "error":   3,
    "stopped": 4,
}

UPDATE_INTERVAL = 0.5   # Supabase 업데이트 주기 (초) — 너무 잦으면 rate limit

# =============================================================================
# Supabase 클라이언트
# =============================================================================

supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)

def update_robot_state(robot_id: int, robot_type: int,
                       pos_x: float, pos_y: float, pos_z: float,
                       rot_w: float = 1.0,
                       status: int = STATUS["idle"],
                       holding: int = 0):
    try:
        supabase.table("robot_state").upsert({
            "robot_id":   robot_id,
            "robot_type": robot_type,
            "pos_x":      round(pos_x, 4),
            "pos_y":      round(pos_y, 4),
            "pos_z":      round(pos_z, 4),
            "rot_w":      round(rot_w, 4),
            "status":     status,
            "holding":    holding,
        }).execute()
    except Exception as e:
        print(f"[WARN] Supabase update failed robot_id={robot_id}: {e}")

# =============================================================================
# ROS2 브릿지 노드
# =============================================================================

class SupabaseBridgeNode(Node):
    def __init__(self):
        super().__init__("supabase_bridge_node")

        self._last_update = {}   # robot_id → last update time
        self._lock = threading.Lock()

        # 토픽별 구독 등록
        self._subscribers = []
        for topic, info in ROBOT_TOPIC_MAP.items():
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda msg, i=info: self._odom_callback(msg, i),
                10
            )
            self._subscribers.append(sub)
            self.get_logger().info(
                f"Subscribed: {topic} → robot_id={info['robot_id']}"
            )

        self.get_logger().info(
            "Supabase bridge node ready — "
            f"{len(ROBOT_TOPIC_MAP)} topics registered"
        )

    def _odom_callback(self, msg: Odometry, robot_info: dict):
        robot_id   = robot_info["robot_id"]
        robot_type = robot_info["robot_type"]

        now = self.get_clock().now().nanoseconds / 1e9

        # rate limit — UPDATE_INTERVAL 이상 지났을 때만 업데이트
        with self._lock:
            last = self._last_update.get(robot_id, 0)
            if now - last < UPDATE_INTERVAL:
                return
            self._last_update[robot_id] = now

        pos = msg.pose.pose.position
        rot = msg.pose.pose.orientation

        # 별도 스레드에서 Supabase 업데이트 (ROS2 콜백 블로킹 방지)
        threading.Thread(
            target=update_robot_state,
            args=(robot_id, robot_type,
                  pos.x, pos.y, pos.z, rot.w),
            daemon=True
        ).start()


# =============================================================================
# 메인
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SupabaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bridge node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
