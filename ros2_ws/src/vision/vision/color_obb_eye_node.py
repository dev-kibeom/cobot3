from __future__ import annotations

# Eye node: ROS2 Image -> color/shape OBB -> Float32MultiArray OBB.
#
# This is the lightweight alternative to yolo11s_obb_eye_node.py for the
# controlled top-view Isaac Sim cells. It publishes the same OBB payload, so
# alignment_brain.py can consume either detector without changing its logic.
#
# Example for one camera:
# python3 ros2 run vision color_obb_eye_node \
#   --cells cell_01 \
#   --image-topic-template /sim/top_camera/image \
#   --obb-topic-template /vision/plate_obb \
#   --publish-debug
#
# Example for six cells:
# python3 ros2 run vision color_obb_eye_node \
#   --cells cell_01,cell_02,cell_03,cell_04,cell_05,cell_06

import argparse
import functools
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    from .color_shape_obb import ColorShapeConfig, detect_color_shape_obb, draw_detection
    from .vision_contracts import encode_obb
except ImportError:  # Allows direct execution from the legacy visions directory.
    from color_shape_obb import ColorShapeConfig, detect_color_shape_obb, draw_detection
    from vision_contracts import encode_obb


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cells", default="global")
    parser.add_argument("--image-topic-template", default="/{cell}/top_camera/image")
    parser.add_argument("--obb-topic-template", default="/{cell}/vision/plate_obb")
    parser.add_argument("--debug-topic-template", default="/{cell}/vision/debug_image")
    parser.add_argument("--max-hz", type=float, default=10.0)
    parser.add_argument("--publish-debug", action="store_true")
    parser.add_argument("--no-empty", action="store_true", help="Do not publish invalid detections.")
    parser.add_argument("--min-area-px", type=float, default=180.0)
    parser.add_argument("--plate-aspect-min", type=float, default=1.55)
    parser.add_argument("--lab-delta-threshold", type=float, default=16.0)
    return parser.parse_args()


class ColorObbEyeNode(Node):
    def __init__(self, args):
        super().__init__("color_obb_eye_node")
        self.args = args
        self.bridge = CvBridge()
        self.config = ColorShapeConfig(
            min_area_px=args.min_area_px,
            plate_aspect_min=args.plate_aspect_min,
            lab_delta_threshold=args.lab_delta_threshold,
        )
        self.min_period = 1.0 / args.max_hz if args.max_hz > 0.0 else 0.0
        self.last_infer_time = {}
        self.publishers_by_cell = {}
        self.debug_publishers_by_cell = {}

        cells = [cell.strip() for cell in args.cells.split(",") if cell.strip()]
        if not cells:
            raise ValueError("At least one cell is required.")

        for cell in cells:
            image_topic = args.image_topic_template.format(cell=cell)
            obb_topic = args.obb_topic_template.format(cell=cell)
            self.publishers_by_cell[cell] = self.create_publisher(Float32MultiArray, obb_topic, 10)
            self.create_subscription(
                Image,
                image_topic,
                functools.partial(self.image_callback, cell=cell),
                qos_profile_sensor_data,
            )
            if args.publish_debug:
                debug_topic = args.debug_topic_template.format(cell=cell)
                self.debug_publishers_by_cell[cell] = self.create_publisher(Image, debug_topic, 10)
            self.get_logger().info(f"{cell}: {image_topic} -> {obb_topic}")

    def image_callback(self, msg: Image, cell: str):
        now = time.monotonic()
        last = self.last_infer_time.get(cell, 0.0)
        if self.min_period and now - last < self.min_period:
            return
        self.last_infer_time[cell] = now

        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        det = detect_color_shape_obb(frame_bgr, self.config)

        if det.valid or not self.args.no_empty:
            out = Float32MultiArray()
            out.data = encode_obb(det)
            self.publishers_by_cell[cell].publish(out)

        if self.args.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(draw_detection(frame_bgr, det), encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_publishers_by_cell[cell].publish(debug_msg)


def main():
    args = parse_args()
    rclpy.init()
    node = ColorObbEyeNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
