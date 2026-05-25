import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan, Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from cv_bridge import CvBridge
import math


class ImageBridgeNode(Node):
    def __init__(self):
        super().__init__("image_bridge_node")
        self.bridge = CvBridge()

        # 주파수 제어용 타임스탬프 기록기 (통신 병목 방지용 10Hz 가두기)
        self.last_scan_time = self.get_clock().now()
        self.last_img_time = self.get_clock().now()

        # -----------------------------------------------------------------
        # [QoS 설정] 
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
        # 비전 전처리 라인: Raw Image -> Compressed Image (YOLO용)]
        # -----------------------------------------------------------------
        self.img_sub = self.create_subscription(
            Image, "/camera/rgb/observer_01", self.image_callback, best_effort_qos
        )
        self.img_pub = self.create_publisher(
            CompressedImage, "/camera/rgb/observer_01/compressed", reliable_qos
        )

        self.get_logger().info(
            "🚀 rgb 이미지 압축 노드 가동."
        )

    def image_callback(self, msg_raw_image):
        """두산 로봇 팔의 무거운 원본 이미지를 JPEG compressed 패킷으로 실시간 다이어트"""
        now = self.get_clock().now()
        if (now - self.last_img_time).nanoseconds < 100000000:  # 10Hz 주파수 제한
            return
        self.last_img_time = now

        try:
            # 1. sensor_msgs/Image -> OpenCV 이미지(BGR) 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg_raw_image, desired_encoding="bgr8")

            # 2. YOLO 인식에 무리 없는 수준으로 압축 및 경량화 (640x480 다운샘플링)
            resized_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)

            # 3. ROS2 CompressedImage 메시지 포맷으로 인코딩 빌드
            msg_compressed = self.bridge.cv2_to_compressed_imgmsg(
                resized_image, dst_format="jpg"
            )
            msg_compressed.header = msg_raw_image.header

            # Sub PC(상헌님 YOLO)로 가벼워진 데이터 배달
            self.img_pub.publish(msg_compressed)

        except Exception as e:
            self.get_logger().error(f"비전 데이터 압축 변환 중 에러 발생: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
