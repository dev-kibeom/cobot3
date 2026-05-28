import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan, Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np  # 🚀 추가
from cv_bridge import CvBridge
import json


class ImageBridgeNode(Node):
    def __init__(self):
        super().__init__("image_bridge_node")
        self.bridge = CvBridge()

        self.last_img_time = self.get_clock().now()

        # QoS 설정 (기존과 동일)
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

        self.is_amr_arrived = False

        # -----------------------------------------------------------------
        # 🚀 [추가 및 수정] RGB와 Depth 모두 구독 및 압축 퍼블리시
        # -----------------------------------------------------------------
        self.img_sub = self.create_subscription(
            Image, "/m0609/camera/rgb", self.image_callback, best_effort_qos
        )
        self.img_pub = self.create_publisher(
            CompressedImage, "/m0609/camera/rgb/compressed", reliable_qos
        )

        self.depth_sub = self.create_subscription(
            Image, "/m0609/camera/depth", self.depth_callback, best_effort_qos
        )
        self.depth_pub = self.create_publisher(
            CompressedImage, "/m0609/camera/depth/compressed", reliable_qos
        )
        self.trigger_sub = self.create_subscription(
            String, "amr_arrival_trigger", self.arrival_callback, 10
        )

        self.get_logger().info("🚀 RGB & Depth 이미지 압축 전송 브릿지 가동.")

    def arrival_callback(self, msg):
        try:
            data = json.loads(msg.data)
            status = data.get("status")

            if status == "ARRIVED" and not self.is_amr_arrived:
                self.get_logger().info(
                    "🔔 AMR 도착 감지! 카메라 스트림 압축 및 전송을 시작합니다."
                )
                self.is_amr_arrived = True

            elif status == "DONE" and self.is_amr_arrived:
                self.get_logger().info(
                    "✅ 작업 완료 감지! 카메라 스트림 전송을 중단(절전)합니다."
                )
                self.is_amr_arrived = False
        except Exception as e:
            pass
        
    def image_callback(self, msg_raw_image):
        """RGB 이미지를 손실 압축(JPEG)하여 가볍게 전송"""
        if not self.is_amr_arrived:
            return
            
        now = self.get_clock().now()
        if (now - self.last_img_time).nanoseconds < 100000000:  # 10Hz 제한
            return
        self.last_img_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg_raw_image, desired_encoding="bgr8")
            resized_image = cv2.resize(
                cv_image, (640, 480), interpolation=cv2.INTER_AREA
            )

            msg_compressed = self.bridge.cv2_to_compressed_imgmsg(
                resized_image, dst_format="jpg"
            )
            msg_compressed.header = msg_raw_image.header
            self.img_pub.publish(msg_compressed)
        except Exception as e:
            self.get_logger().error(f"RGB 압축 에러: {str(e)}")

    def depth_callback(self, msg_raw_depth):
        """Depth 이미지를 무손실 압축(PNG)하여 안전하게 전송"""
        # (RGB와 동일한 10Hz 주파수로 제한됨)
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(
                msg_raw_depth, desired_encoding="passthrough"
            )

            # 🚀 주의: Depth 맵 리사이즈 시에는 값이 섞이지 않도록 무조건 INTER_NEAREST 사용!
            resized_depth = cv2.resize(
                cv_depth, (640, 480), interpolation=cv2.INTER_NEAREST
            )

            # Isaac Sim의 32FC1(미터) 소수점 데이터를 16UC1(밀리미터) 정수로 변환해야 무손실 압축 가능
            if msg_raw_depth.encoding == "32FC1":
                depth_16u = (resized_depth * 1000.0).astype(np.uint16)
            else:
                depth_16u = resized_depth

            # PNG 무손실 압축 진행
            msg_compressed = self.bridge.cv2_to_compressed_imgmsg(
                depth_16u, dst_format="png"
            )
            msg_compressed.header = msg_raw_depth.header
            msg_compressed.format = "16UC1; png compressed"  # 명시적 포맷 기록

            self.depth_pub.publish(msg_compressed)
        except Exception as e:
            self.get_logger().error(f"Depth 압축 에러: {str(e)}")


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
