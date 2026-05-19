import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("doosan_pick_and_place_node")

        # Action Graph에서 설정한 토픽 이름으로 퍼블리셔 생성
        self.publisher_ = self.create_publisher(JointState, "/joint_command", 10)

        # Isaac Sim 관절 명부와 동일한 순서로 이름 배열 생성
        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "finger_joint",
            "right_inner_knuckle_joint",
        ]

        # 과거 코드 기반 그리퍼 세팅
        self.gripper_open = [0.0, 0.0]
        self.gripper_close = [0.7, -0.7]

        # Pick & Place 시나리오 웨이포인트 (동작 확인용 임의 각도)
        self.waypoints = [
            {
                "name": "1_Home",
                "pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "2_Ready",
                "pos": [0.0, -0.5, 1.0, 0.0, 1.0, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "3_Pick_Hover",
                "pos": [0.5, -0.5, 1.0, 0.0, 1.0, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "4_Pick_Down",
                "pos": [0.5, -0.2, 1.2, 0.0, 0.5, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "5_Grasp",
                "pos": [0.5, -0.2, 1.2, 0.0, 0.5, 0.0],
                "gripper": self.gripper_close,
            },
            {
                "name": "6_Pick_Up",
                "pos": [0.5, -0.5, 1.0, 0.0, 1.0, 0.0],
                "gripper": self.gripper_close,
            },
            {
                "name": "7_Place_Hover",
                "pos": [-0.5, -0.5, 1.0, 0.0, 1.0, 0.0],
                "gripper": self.gripper_close,
            },
            {
                "name": "8_Place_Down",
                "pos": [-0.5, -0.2, 1.2, 0.0, 0.5, 0.0],
                "gripper": self.gripper_close,
            },
            {
                "name": "9_Release",
                "pos": [-0.5, -0.2, 1.2, 0.0, 0.5, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "10_Place_Up",
                "pos": [-0.5, -0.5, 1.0, 0.0, 1.0, 0.0],
                "gripper": self.gripper_open,
            },
            {
                "name": "11_Home",
                "pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "gripper": self.gripper_open,
            },
        ]

        self.current_wp_idx = 0
        self.current_pos = np.array(
            self.waypoints[0]["pos"] + self.waypoints[0]["gripper"]
        )

        # 0.1초(10Hz)마다 실행되며 로봇을 부드럽게 보간(Interpolation) 제어
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.interpolation_steps = 20  # 웨이포인트 간 이동 스텝 (2초 소요)
        self.current_step = 0

    def timer_callback(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info("✅ Pick and Place 시나리오 완료!")
            self.timer.cancel()
            return

        target_wp = self.waypoints[self.current_wp_idx]
        target_pos = np.array(target_wp["pos"] + target_wp["gripper"])

        # 현재 위치와 목표 위치 사이를 부드럽게 쪼개서 전송 (Isaac Sim 관절 튕김 방지)
        alpha = self.current_step / self.interpolation_steps
        interpolated_pos = (1.0 - alpha) * self.current_pos + alpha * target_pos

        # ROS2 JointState 메시지 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = interpolated_pos.tolist()
        self.publisher_.publish(msg)

        self.current_step += 1

        # 타겟 도달 시 다음 스텝으로 넘어감
        if self.current_step > self.interpolation_steps:
            self.get_logger().info(f"도달 완료: {target_wp['name']}")
            self.current_pos = target_pos
            self.current_wp_idx += 1
            self.current_step = 0
            time.sleep(0.5)  # 동작 완료 후 0.5초 대기


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()