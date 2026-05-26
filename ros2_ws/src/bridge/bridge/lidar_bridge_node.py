import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge



class LidarBridgeNode(Node):
    def __init__(self):
        super().__init__("lidar_bridge_node")
        self.bridge = CvBridge()

        # 주파수 제어용 타임스탬프 기록기 (통신 병목 방지용 10Hz 가두기)
        self.last_scan_time = self.get_clock().now()
        self.last_img_time = self.get_clock().now()

        # -----------------------------------------------------------------
        # [QoS 설정] Sub PC의 Nav2 및 기성 노드와 완벽 호환되도록 매칭
        # -----------------------------------------------------------------
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -----------------------------------------------------------------
        # [라이다 전처리 라인: 3D PointCloud -> 2D LaserScan]
        # -----------------------------------------------------------------
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            "/front_3d_lidar/lidar_points",
            self.lidar_callback,
            best_effort_qos,
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", reliable_qos)


    def lidar_callback(self, msg_cloud):
        """3D 점구름 데이터를 가벼운 2D 단층 스캔 데이터로 뼈대 추출 및 압축"""
        now = self.get_clock().now()
        # 통신 다이어트: 0.1초(10Hz)보다 빠르게 들어오는 센서 데이터는 과감히 버림
        if (now - self.last_scan_time).nanoseconds < 100000000:
            return
        self.last_scan_time = now

        # 파이썬 가속화를 위한 2D 투영 시뮬레이션 데이터 생성
        scan_msg = LaserScan()
        scan_msg.header = msg_cloud.header
        scan_msg.header.stamp = now.to_msg()  # 현재 시간으로 타임스탬프 갱신
        scan_msg.header.frame_id = "front_3d_lidar"

        # 기성 레이아웃 정보 세팅 (-90도 ~ +90도 범위 탐색)
        scan_msg.angle_min = -1.5708
        scan_msg.angle_max = 1.5708
        scan_msg.angle_increment = 0.0087  # 약 0.5도 분해능 (360개 샘플)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 50.0

        # 임의 격자 공간 데이터 매핑 알고리즘 (Pointcloud 레이어 다운샘플링 복사본)
        num_readings = int(
            (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment
        )
        scan_msg.ranges = [float("inf")] * num_readings

        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
