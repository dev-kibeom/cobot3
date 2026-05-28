import rclpy
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from controller.robots.base_robot_controller import BaseRobotController

class M0609Controller(BaseRobotController):
    def __init__(self):
        # 부모 클래스 생성자에서 파라미터 로드가 먼저 일어납니다.
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
        self.get_logger().info(
            "🦾 두산(m0609) 전용 파라미터 제어 노드가 가동되었습니다."
        )

    def get_target_rotation(self) -> np.ndarray:
        return np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

    def joint_callback(self, msg):
        if not self.isaac_joint_names:
            self.isaac_joint_names = msg.name

        # 부모가 파라미터 서버에서 읽어온 self.arm_joints 리스트 기준 동기화
        q_temp = [
            msg.position[msg.name.index(j)] if j in msg.name else 0.0
            for j in self.arm_joints
        ]
        self.current_q = np.array(q_temp)

    def publish_joint_command(self, target_q):
        if not self.isaac_joint_names:
            return

        final_positions = [
            target_q[self.arm_joints.index(n)] if n in self.arm_joints else 0.0
            for n in self.isaac_joint_names
        ]

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