import rclpy
import numpy as np
import time
from sensor_msgs.msg import JointState
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

        self.isaac_joint_names = []

        # 🚀 그리퍼 모듈 장착 (7번째 관절)
        self.gripper = GripperController(
            joint_name="finger_joint", open_val=0.0, close_val=1.0
        )

        self.get_logger().info("🦾 두산(m0609) PnP & 그리퍼 제어 노드 가동 완료.")

    def get_target_rotation(self, yaw_angle: float) -> np.ndarray:
        # 1. 기본 하방(바닥을 향하는) 자세
        base_down_rot = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

        # 2. Z축 기준 회전 행렬 (YOLO에서 알아낸 회전 각도)
        c, s = np.cos(yaw_angle), np.sin(yaw_angle)
        z_rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

        # 3. 바닥을 본 상태에서 손목을 비틂 (행렬 곱)
        return base_down_rot @ z_rot

    def execute_pnp_sequence(self, target_base_pos, yaw_angle):
        """실제 Pick and Place 순서 정의"""
        # 그리퍼 오프셋(15cm) 고려 및 안전 높이 설정
        z_grasp = target_base_pos[2] + self.gripper_offset
        z_hover = z_grasp + 0.15  # 15cm 위에서 대기

        x, y = target_base_pos[0], target_base_pos[1]
        target_rot = self.get_target_rotation(yaw_angle)

        self.get_logger().info("1️⃣ 그리퍼 열고 목표 상공으로 이동")
        _, g_val = self.gripper.open()
        self.calculate_and_move(np.array([x, y, z_hover]), target_rot, g_val)
        time.sleep(2.0)

        self.get_logger().info("2️⃣ 물체로 하강")
        self.calculate_and_move(np.array([x, y, z_grasp]), target_rot, g_val)
        time.sleep(1.5)

        self.get_logger().info("3️⃣ 그리퍼 닫기 (파지)")
        _, g_val = self.gripper.close()
        self.calculate_and_move(np.array([x, y, z_grasp]), target_rot, g_val)
        time.sleep(5.0)

        self.get_logger().info("4️⃣ 물체 들고 상승")
        self.calculate_and_move(np.array([x, y, z_hover]), target_rot, g_val)
        time.sleep(2.0)

        # 이후 다른 장소로 이동하는 로직을 추가할 수 있습니다.

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