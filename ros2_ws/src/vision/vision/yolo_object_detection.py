import os
import rclpy
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time

# YOLO 라이브러리 임포트 (ultralytics 설치 필요: pip install ultralytics)
from ultralytics import YOLO


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__(
            "yolo_vision_node",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.bridge = CvBridge()

        self.intrinsics = None
        self.depth_encoding = None

        # 모델 로드
        vision_share_dir = get_package_share_directory("vision")
        model_path = os.path.join(vision_share_dir, "resource", "best.pt")
        self.get_logger().info(f"🧠 YOLO 모델 로딩 중... 경로: {model_path}")
        self.model = YOLO(model_path)

        # 제어기로 쏠 목표 좌표 퍼블리셔
        self.target_pub = self.create_publisher(
            PoseStamped, "/vision/target_pose", 10
        )

        # 토픽 구독
        self.info_sub = self.create_subscription(
            CameraInfo, "/m0609/camera/info", self.info_callback, 10
        )
        self.color_sub = message_filters.Subscriber(self, Image, "/m0609/camera/rgb")
        self.depth_sub = message_filters.Subscriber(self, Image, "/m0609/camera/depth")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 10, 0.1
        )
        self.ts.registerCallback(self.image_callback)

        self.last_publish_time = 0.0  # 너무 잦은 퍼블리시 방지용 쿨다운 타이머

        cv2.namedWindow("YOLO OBB Inference")
        self.get_logger().info("👁️ YOLO 비전 노드 가동! 카메라 화면을 주시합니다.")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg

    def image_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        # ROS 이미지를 OpenCV 포맷으로 변환
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.depth_encoding = depth_msg.encoding

        # ========================================================
        # 💡 YOLO OBB 추론 (신뢰도 50% 이상만 필터링)
        # ========================================================
        results = self.model(color_img, conf=0.5, verbose=False)

        # 모델이 기본 제공하는 예쁜 박스/라벨 시각화 이미지 가져오기
        annotated_img = results[0].plot()

        # OBB 박스가 탐지되었는지 확인
        if results[0].obb is not None:
            # obb.xywhr: 중심X, 중심Y, 너비, 높이, 회전각도(라디안)
            obbs = results[0].obb.xywhr.cpu().numpy()

            for obb in obbs:
                cx, cy, w, h, r = obb
                u, v = int(cx), int(cy)

                # 뎁스맵에서 객체 중심의 Z(깊이)값 추출
                z = (
                    float(depth_img[v, u])
                    if self.depth_encoding == "32FC1"
                    else float(depth_img[v, u]) / 1000.0
                )

                # Z값이 비정상(0 이하)이면 무시
                if z <= 0.0:
                    continue

                # 3D 렌즈 기준 좌표 계산
                fx, fy = self.intrinsics.k[0], self.intrinsics.k[4]
                px, py = self.intrinsics.k[2], self.intrinsics.k[5]
                cam_x, cam_y, cam_z = (u - px) * z / fx, (v - py) * z / fy, z

                # ========================================================
                # 💡 중복 송출 방지 (2초에 1번만 쏘도록 쿨다운 적용)
                # ========================================================
                current_time = time.time()
                if (current_time - self.last_publish_time) > 2.0:
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_color_optical_frame"  # 실제 카메라 프레임 이름 확인 필요

                    # 3D 위치 입력
                    msg.pose.position.x = cam_x
                    msg.pose.position.y = cam_y
                    msg.pose.position.z = cam_z

                    # YOLO OBB의 회전 각도(r)를 Z축 회전(Quaternion)으로 변환
                    msg.pose.orientation.z = math.sin(r / 2.0)
                    msg.pose.orientation.w = math.cos(r / 2.0)
                    
                    self.target_pub.publish(msg)
                    self.get_logger().info(
                        f"🎯 YOLO 목표 포착! 좌표 전송: [{cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}] (각도: {np.degrees(r):.1f}도)"
                    )

                    self.last_publish_time = current_time

        cv2.imshow("YOLO OBB Inference", annotated_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
