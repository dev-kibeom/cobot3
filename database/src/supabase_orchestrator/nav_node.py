"""
nav_node.py — Supabase → ROS2 Nav Goal 실행 노드
==================================================
Supabase robot_command_carter 테이블을 polling하여
Nav Goal을 ROS2 NavigateToPose 액션으로 실행합니다.

실행: python3 nav_node.py
필요: pip install supabase
      ROS2 + Nav2 환경
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import threading
import time
from supabase import create_client, Client

# =============================================================================
# 설정
# =============================================================================

SUPABASE_URL = "https://gppsbxlvbgfmcchjqrwu.supabase.co"
SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImdwcHNieGx2YmdmbWNjaGpxcnd1Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzkzMzczNjgsImV4cCI6MjA5NDkxMzM2OH0.7cAlleV_j8HE9_oHq8MmCoQu141H6qutd2FV5BY_vQo"


MY_ROBOT_ID   = 1       # 이 노드가 담당하는 Carter ID
POLL_INTERVAL = 0.5     # 커맨드 polling 주기 (초)
FRAME_ID      = "map"   # Nav2 좌표 프레임

# command 코드
CMD_MOVE    = 1
CMD_PICKUP  = 2
CMD_DELIVER = 3
CMD_STANDBY = 4
CMD_STOP    = 5

# status 코드
CMD_PENDING  = 0
CMD_RUNNING  = 1
CMD_DONE     = 2
CMD_FAILED   = 3

# location 좌표 (맵 완성 후 실제 값으로 교체)
LOCATION_POSES = {
    1: {"x": 0.0,  "y": 0.0,  "z": 0.0, "w": 1.0},  # 자재 창고
    2: {"x": 5.0,  "y": 0.0,  "z": 0.0, "w": 1.0},  # 제조 설비
    3: {"x": 10.0, "y": 0.0,  "z": 0.0, "w": 1.0},  # 조립대
    4: {"x": 0.0,  "y": 5.0,  "z": 0.0, "w": 1.0},  # 대기 위치
}

# =============================================================================
# Supabase 클라이언트
# =============================================================================

supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)

def update_command_status(cmd_id: int, status: int):
    try:
        supabase.table("robot_command_carter")\
                .update({"status": status})\
                .eq("id", cmd_id)\
                .execute()
    except Exception as e:
        print(f"[WARN] status update failed cmd_id={cmd_id}: {e}")

def update_robot_state(robot_id: int, status: int,
                        pos_x: float = 0.0, pos_y: float = 0.0,
                        holding: int = 0):
    try:
        supabase.table("robot_state").upsert({
            "robot_id": robot_id,
            "robot_type": 1,   # Carter
            "pos_x":   pos_x,
            "pos_y":   pos_y,
            "status":  status,
            "holding": holding,
        }).execute()
    except Exception as e:
        print(f"[WARN] robot_state update failed: {e}")

# =============================================================================
# ROS2 Nav 노드
# =============================================================================

class NavNode(Node):
    def __init__(self):
        super().__init__(f"nav_node_carter_{MY_ROBOT_ID}")

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._busy       = False
        self._lock       = threading.Lock()

        self.get_logger().info(
            f"Nav node ready — robot_id={MY_ROBOT_ID} "
            f"polling Supabase every {POLL_INTERVAL}s"
        )

        # polling 타이머
        self.create_timer(POLL_INTERVAL, self._poll_commands)

    # ── Supabase polling ──────────────────────────────────────────────────────
    def _poll_commands(self):
        with self._lock:
            if self._busy:
                return

        try:
            res = supabase.table("robot_command_carter")\
                          .select("*")\
                          .eq("robot_id", MY_ROBOT_ID)\
                          .eq("status", CMD_PENDING)\
                          .order("created_at")\
                          .limit(1)\
                          .execute()

            if not res.data:
                return

            cmd = res.data[0]
            self.get_logger().info(
                f"Command received — id={cmd['id']} "
                f"cmd={cmd['command']} to_loc={cmd['to_loc']}"
            )

            # 별도 스레드에서 실행 (ROS2 타이머 콜백 블로킹 방지)
            threading.Thread(
                target=self._execute_command,
                args=(cmd,),
                daemon=True
            ).start()

        except Exception as e:
            self.get_logger().warn(f"Polling error: {e}")

    # ── 커맨드 실행 ───────────────────────────────────────────────────────────
    def _execute_command(self, cmd: dict):
        with self._lock:
            self._busy = True

        cmd_id  = cmd["id"]
        command = cmd["command"]
        to_loc  = cmd["to_loc"]

        # STOP 커맨드 즉시 처리
        if command == CMD_STOP:
            self.get_logger().info("STOP command received")
            update_command_status(cmd_id, CMD_DONE)
            update_robot_state(MY_ROBOT_ID, 4)   # stopped
            with self._lock:
                self._busy = False
            return

        # STANDBY 커맨드
        if command == CMD_STANDBY:
            update_command_status(cmd_id, CMD_RUNNING)
            update_robot_state(MY_ROBOT_ID, 1)   # idle
            update_command_status(cmd_id, CMD_DONE)
            with self._lock:
                self._busy = False
            return

        # 이동 커맨드 (MOVE, PICKUP, DELIVER)
        pose = LOCATION_POSES.get(to_loc)
        if not pose:
            self.get_logger().warn(f"Unknown location_id={to_loc}")
            update_command_status(cmd_id, CMD_FAILED)
            with self._lock:
                self._busy = False
            return

        update_command_status(cmd_id, CMD_RUNNING)
        update_robot_state(MY_ROBOT_ID, 2)   # working

        success = self._navigate_to(pose)

        if success:
            self.get_logger().info(f"Arrived at loc={to_loc}")
            update_command_status(cmd_id, CMD_DONE)
            update_robot_state(MY_ROBOT_ID, 1,
                               pose["x"], pose["y"],
                               holding=cmd.get("cargo_code", 0) or 0)
        else:
            self.get_logger().error(f"Navigation failed to loc={to_loc}")
            update_command_status(cmd_id, CMD_FAILED)
            update_robot_state(MY_ROBOT_ID, 3)   # error

        with self._lock:
            self._busy = False

    # ── Nav2 NavigateToPose 실행 ──────────────────────────────────────────────
    def _navigate_to(self, pose: dict) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = FRAME_ID
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose["x"]
        goal.pose.pose.position.y = pose["y"]
        goal.pose.pose.position.z = pose["z"]
        goal.pose.pose.orientation.w = pose["w"]

        self.get_logger().info(
            f"Navigating to ({pose['x']:.2f}, {pose['y']:.2f})"
        )

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        return result is not None

# =============================================================================
# 메인
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nav node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
