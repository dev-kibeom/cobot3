import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped


class VisionNode(Node):
    def __init__(self):
        super().__init__(
            "vision_node",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.bridge = CvBridge()

        self.intrinsics = None
        self.latest_depth = None
        self.depth_encoding = None

        # 로봇 제어기에게 좌표를 쏴줄 전용 퍼블리셔
        self.target_pub = self.create_publisher(
            PointStamped, "/vision/target_point", 10
        )

        self.info_sub = self.create_subscription(
            CameraInfo, "/m0609/camera/info", self.info_callback, 10
        )
        self.color_sub = message_filters.Subscriber(self, Image, "/m0609/camera/rgb")
        self.depth_sub = message_filters.Subscriber(self, Image, "/m0609/camera/depth")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 10, 0.1
        )
        self.ts.registerCallback(self.image_callback)

        cv2.namedWindow("Click to Pick")
        cv2.setMouseCallback("Click to Pick", self.mouse_callback)
        self.get_logger().info("👁️ 비전 인지 노드 가동! 화면을 클릭하세요.")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg

    def image_callback(self, color_msg, depth_msg):
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        self.depth_encoding = depth_msg.encoding
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        cv2.imshow("Click to Pick", color_img)
        cv2.waitKey(1)

    def mouse_callback(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.intrinsics is None or self.latest_depth is None:
                return

            z = (
                float(self.latest_depth[v, u])
                if self.depth_encoding == "32FC1"
                else float(self.latest_depth[v, u]) / 1000.0
            )
            if z <= 0.0:
                return

            fx, fy = self.intrinsics.k[0], self.intrinsics.k[4]
            cx, cy = self.intrinsics.k[2], self.intrinsics.k[5]

            # 카메라 렌즈 기준 3D 좌표
            cam_x, cam_y, cam_z = (u - cx) * z / fx, (v - cy) * z / fy, z

            # 계산된 좌표를 통신망으로 쏨 (프레임 이름표를 붙여서)
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
            msg.point.x, msg.point.y, msg.point.z = cam_x, cam_y, cam_z

            self.target_pub.publish(msg)
            self.get_logger().info(
                f"📡 목표 발견! 카메라 기준 좌표 전송: [{cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
