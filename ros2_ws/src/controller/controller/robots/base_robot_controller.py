import math
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pinocchio as pin
from abc import ABC, abstractmethod
import numpy as np
from controller.pinocchio_core import PinocchioCore


class BaseRobotController(Node, ABC):
    def __init__(self, node_name):
        super().__init__(node_name)

        # ROS 2 파라미터 선언 및 YAML 값 자동 로드
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("end_effector_frame_name", "link_6")
        self.declare_parameter("gripper_offset", 0.0)
        self.declare_parameter(
            "robot_arm_joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )

        self.urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        self.ee_frame_name = (
            self.get_parameter("end_effector_frame_name")
            .get_parameter_value()
            .string_value
        )
        self.gripper_offset = (
            self.get_parameter("gripper_offset").get_parameter_value().double_value
        )
        self.arm_joints = (
            self.get_parameter("robot_arm_joint_names")
            .get_parameter_value()
            .string_array_value
        )

        # 파라미터 정상 로드 검증
        if not self.urdf_path:
            self.get_logger().error(
                "❌ URDF 경로 파라미터가 비어있습니다! YAML 설정을 확인하세요."
            )
            return

        # 로드된 파라미터로 Pinocchio 코어 초기화
        self.pino_core = PinocchioCore(
            urdf_path=self.urdf_path, ee_frame_name=self.ee_frame_name
        )

        self.is_running = False
        self.current_q = np.zeros(len(self.arm_joints))

        # 공통 통신: 비전 노드로부터 타겟 수신
        self.target_sub = self.create_subscription(
            PoseStamped, "/vision/target_pose", self.target_callback, 10
        )

    @abstractmethod
    def get_target_rotation(self, yaw_angle: float) -> np.ndarray:
        pass

    @abstractmethod
    def publish_joint_command(self, target_q: np.ndarray, gripper_val: float):
        pass
    
    @abstractmethod
    def execute_pnp_sequence(self, target_base_pos, yaw_angle):
        pass

    def target_callback(self, msg):
        if self.is_running:
            self.get_logger().warn("⚠️ 이전 작업이 진행 중입니다. 새 명령을 무시합니다.")
            return
        
        self.get_logger().info("📥 비전 목표 수신. 기구학 연산 시작.")
        cam_pt = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # 쿼터니언(z, w)에서 Yaw 각도(라디안) 다시 추출
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw_angle = 2.0 * math.atan2(qz, qw)

        # FK 연산
        oM_ee = self.pino_core.get_fk_pose(self.current_q)
        camera_offset = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.05]))
        oM_camera = oM_ee * camera_offset

        target_base_pos = oM_camera.act(cam_pt)

        # 🚀 3. PnP 시퀀스를 스레드로 실행 (ROS 2 루프 블로킹 방지)
        self.is_running = True
        thread = threading.Thread(
            target=self._run_sequence, args=(target_base_pos, yaw_angle), daemon=True
        )
        thread.start()

    def _run_sequence(self, target_base_pos, yaw_angle):
        try:
            self.execute_pnp_sequence(target_base_pos, yaw_angle)
        except Exception as e:
            self.get_logger().error(f"❌ 시퀀스 실행 중 오류: {e}")
        finally:
            self.is_running = False
            self.get_logger().info("✅ PnP 시퀀스 완료! 다음 목표를 대기합니다.")

    def calculate_and_move(self, target_pos, target_rot, gripper_val):
        """IK를 풀고 즉시 이동하는 공용 함수"""
        is_success, target_q = self.pino_core.solve_ik(
            target_pos, target_rot, self.current_q
        )
        if is_success:
            self.publish_joint_command(target_q, gripper_val)
            return True
        else:
            self.get_logger().error(f"❌ IK 계산 실패: {target_pos}")
            return False
