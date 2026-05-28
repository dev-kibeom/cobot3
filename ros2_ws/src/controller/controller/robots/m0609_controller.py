import rclpy
import numpy as np
import time
import json

from std_msgs import msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from controller.robots.base_robot_controller import BaseRobotController
from controller.robots.gripper_controller import GripperController


class M0609Controller(BaseRobotController):
    def __init__(self):
        super().__init__("doosan_robot_controller_node")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.joint_sub = self.create_subscription(
            JointState, "/m0609/joint_states", self.joint_callback, qos_best_effort
        )
        self.joint_pub = self.create_publisher(JointState, "/m0609/joint_command", 10)

        self.trigger_sub = self.create_subscription(
            String, "amr_arrival_trigger", self.arrival_callback, 10
        )
        self.trigger_pub = self.create_publisher(String, "amr_arrival_trigger", 10)


        self.vision_sub = self.create_subscription(
            PoseStamped, "/vision/target_pose", self.vision_target_callback, 10
        )

        self.isaac_joint_names = []

        self.my_station_id = "STATION-003"
        self.is_amr_arrived = False

        # 작업 상태 자물쇠 (비전 폭주 방지용)
        self.is_processing_pnp = False

        # 고정된 Place 관절값
        self.HOME_JOINTS = np.deg2rad([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

        # 안전 상공 (예: 모든 관절이 90도나 0도로 곧게 뻗은 상태)
        self.PLACE_HOVER_JOINTS = np.deg2rad([-200.0, 0.0, 90.0, 0.0, 90.0, 0.0])

        # 내려놓을 위치 (예: 2번, 3번 관절만 살짝 굽혀서 바닥으로 내린 상태)
        self.PLACE_TARGET_JOINTS = np.deg2rad([-230.0, 70.0, 15.0, -180.0, -30.0, -90.0])

        # 그리퍼 모듈 장착 (7번째 관절)
        self.gripper = GripperController(
            joint_name="finger_joint", open_val=0.0, close_val=1.0
        )
        
        self.get_logger().info("🦾 두산(m0609) PnP & 그리퍼 제어 노드 가동 완료.")

    def arrival_callback(self, msg):
        """AMR이 도착했다는 신호를 받으면 비전 스위치를 켭니다."""
        # msg.data 에는 '{"event": "AMR_ARRIVED", "robot_id": "IW_HUB-01", "station_id": "STATION-001"}' 형태의 텍스트가 들어있습니다.
        data = json.loads(msg.data)
      
        # 🚀 수정: 1대만 시연하므로 작업대 ID 검사를 없애고, '도착(ARRIVED)' 상태인지만 확인!
        if data.get("status") == "ARRIVED":
            if not self.is_processing_pnp and not self.is_amr_arrived:
                self.get_logger().info(
                    f"🔔 AMR({data.get('robot_id')}) 도착 감지! 비전을 깨웁니다."
                )
                self.is_amr_arrived = True

    # 비전 콜백
    def vision_target_callback(self, msg):
        """비전에서 타겟 좌표가 들어왔을 때 호출되는 함수"""

        # AMR이 도착하지 않았거나, 이미 작업 중이면 비전 데이터 무시
        if not self.is_amr_arrived or self.is_processing_pnp:
            return
        
        if self.is_processing_pnp:
            # 이미 작업 중이면 새로 들어오는 비전 데이터는 쿨하게 무시!
            return

        # 작업 중이 아니라면 자물쇠를 걸고 PnP 시작
        self.is_processing_pnp = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z  # YOLO 노드가 계산한 정밀 뎁스(Z) 값 활용

        # Quaternion(z, w)으로 압축되어 넘어온 회전값을 다시 Yaw 각도(Radian)로 복원
        # 수식: yaw = 2 * atan2(z, w)
        yaw_angle = 2.0 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)

        target_base_pos = np.array([x, y, z])

        # PnP 풀 시퀀스 실행
        self.execute_pnp_sequence(target_base_pos, yaw_angle)

        # 작업이 다 끝나면 다시 다음 타겟을 잡을 수 있도록 자물쇠 해제
        self.is_processing_pnp = False

    def get_target_rotation(self, yaw_angle: float) -> np.ndarray:
        # 1. 기본 하방(바닥을 향하는) 자세
        base_down_rot = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

        # 2. Z축 기준 회전 행렬 (YOLO에서 알아낸 회전 각도)
        c, s = np.cos(yaw_angle), np.sin(yaw_angle)
        z_rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

        # 3. 바닥을 본 상태에서 손목을 비틂 (행렬 곱)
        return base_down_rot @ z_rot

    def execute_pnp_sequence(self, target_base_pos, yaw_angle):
        """Pick and Place 전체 시나리오 실행"""
        # 그리퍼 오프셋(15cm) 고려 및 안전 높이 설정
        z_grasp = target_base_pos[2] + self.gripper_offset
        z_hover = z_grasp + 0.15  # 15cm 위에서 대기

        x, y = target_base_pos[0], target_base_pos[1]
        target_rot = self.get_target_rotation(yaw_angle)

        # ---------------- [ PICK 단계 (IK 연산) ] ----------------
        self.get_logger().info("1️⃣ 그리퍼 열고 목표 상공으로 이동")
        _, g_val = self.gripper.open()
        self.calculate_and_move(np.array([x, y, z_hover]), target_rot, g_val)
        time.sleep(5.0)

        self.get_logger().info("2️⃣ 물체로 하강")
        self.calculate_and_move(np.array([x, y, z_grasp]), target_rot, g_val)
        time.sleep(5.5)
    
        self.get_logger().info("3️⃣ 그리퍼 닫기 (파지)")
        _, g_val = self.gripper.close()
        self.calculate_and_move(np.array([x, y, z_grasp]), target_rot, g_val)
        time.sleep(5.0)  # 파지 안정화 시간

        self.get_logger().info("4️⃣ 물체 들고 Pick 상공으로 상승")
        self.calculate_and_move(np.array([x, y, z_hover]), target_rot, g_val)
        time.sleep(2.0)

        # ---------------- [ PLACE 단계 (고정 관절 이동) ] ----------------
        self.get_logger().info("5️⃣ Place 안전 상공(Hover)으로 이동")
        self.publish_joint_command(self.PLACE_HOVER_JOINTS, g_val)
        time.sleep(5.0)

        self.get_logger().info("6️⃣ 작업대 위로 하강 (Place Target)")
        self.publish_joint_command(self.PLACE_TARGET_JOINTS, g_val)
        time.sleep(10.0)

        self.get_logger().info("7️⃣ 그리퍼 열고 놓기")
        _, g_val = self.gripper.open()
        self.publish_joint_command(self.PLACE_TARGET_JOINTS, g_val)
        time.sleep(5.0)

        self.get_logger().info("8️⃣ Place 안전 상공으로 복귀 및 작업 종료")
        self.publish_joint_command(self.PLACE_HOVER_JOINTS, g_val)
        time.sleep(5.0)

        self.get_logger().info("9️⃣ Home(탐지 대기) 자세로 복귀 및 1사이클 종료")
        self.publish_joint_command(self.HOME_JOINTS, g_val)
        time.sleep(5.5)

        # 사이클이 무사히 끝났으므로 비전 스위치를 끄고 다음 로봇을 기다립니다.
        self.is_amr_arrived = False
        done_msg = String()
        done_msg.data = json.dumps({"status": "DONE"})
        self.trigger_pub.publish(done_msg)
        self.get_logger().info("✅ PnP 작업 완료! 다음 AMR 도착을 대기합니다.")

    def joint_callback(self, msg):
        if not self.isaac_joint_names:
            self.isaac_joint_names = msg.name

        q_temp = [
            msg.position[msg.name.index(j)] if j in msg.name else 0.0
            for j in self.arm_joints
        ]
        self.current_q = np.array(q_temp)

    def publish_joint_command(self, target_q, gripper_val):
        if not self.isaac_joint_names:
            return

        final_positions = []
        for n in self.isaac_joint_names:
            if n in self.arm_joints:
                # 1~6번 팔 관절은 IK 연산 결과 삽입
                final_positions.append(target_q[self.arm_joints.index(n)])
            elif n == self.gripper.joint_name:
                # 7번째 마스터 그리퍼 관절 삽입
                final_positions.append(gripper_val)
            else:
                # 나머지 미믹 관절들은 0.0 또는 이전 값(더미) 유지
                final_positions.append(0.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.isaac_joint_names
        msg.position = final_positions
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = M0609Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()