import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import pinocchio as pin
from abc import ABC, abstractmethod
import numpy as np
from controller.pinocchio_core import PinocchioCore


class BaseRobotController(Node, ABC):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 🚀 [업그레이드] ROS 2 파라미터 선언 및 YAML 값 자동 로드
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

        self.current_q = np.zeros(len(self.arm_joints))

        # 공통 통신: 비전 노드로부터 타겟 수신
        self.target_sub = self.create_subscription(
            PointStamped, "/vision/target_point", self.target_callback, 10
        )

    @abstractmethod
    def get_target_rotation(self) -> np.ndarray:
        pass

    @abstractmethod
    def publish_joint_command(self, target_q: np.ndarray):
        pass

    def target_callback(self, msg):
        self.get_logger().info("📥 비전 목표 수신. 기구학 연산 시작.")
        cam_pt = np.array([msg.point.x, msg.point.y, msg.point.z])

        # 1. FK 연산
        oM_ee = self.pino_core.get_fk_pose(self.current_q)
        camera_offset = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.05]))
        oM_camera = oM_ee * camera_offset

        # 2. 좌표 변환 및 그리퍼 오프셋 보정 (파라미터값 사용)
        target_base_pos = oM_camera.act(cam_pt)
        target_base_pos[2] += self.gripper_offset

        self.get_logger().info(f"📍 계산된 절대 좌표 (TCP 보정): {target_base_pos}")

        # 3. IK 연산 수행
        target_rot = self.get_target_rotation()
        is_success, target_q = self.pino_core.solve_ik(
            target_base_pos, target_rot, self.current_q
        )

        if is_success:
            self.get_logger().info("✅ IK 계산 성공! 명령 송출을 위임합니다.")
            self.publish_joint_command(target_q)
        else:
            self.get_logger().error("❌ IK 계산 실패 (도달 영역 밖)")
