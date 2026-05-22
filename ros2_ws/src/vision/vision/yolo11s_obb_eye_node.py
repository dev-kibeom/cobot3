from __future__ import annotations

# Eye node: ROS2 Image/CompressedImage -> YOLO11s-OBB -> Float32MultiArray OBB.
#
# Default ROS2 package run:
# ros2 run vision yolo

import argparse
import functools
import math
from pathlib import Path
import time

import cv2
import numpy as np

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray

from ultralytics import YOLO

try:
    from .vision_contracts import ObbDetection, empty_obb, encode_obb
except ImportError:  # Allows direct execution from the legacy visions directory.
    from vision_contracts import ObbDetection, empty_obb, encode_obb


def default_model_path() -> Path:
    model_names = (
        "yolo11s_obb_metal_hard-v2_refinetune_best.pt",
    )
    for model_name in model_names:
        local_model = Path(__file__).resolve().parent / "models" / model_name
        if local_model.exists():
            return local_model
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("vision")) / "models"
        for model_name in model_names:
            share_model = share_dir / model_name
            if share_model.exists():
                return share_model
    except Exception:
        pass
    return Path(__file__).resolve().parent / "models" / model_names[0]


DEFAULT_MODEL = default_model_path()


def str_to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"expected boolean value, got {value!r}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--cells", default="global")
    parser.add_argument("--image-topic-template", default="/{cell}/top_camera/image")
    parser.add_argument("--obb-topic-template", default="/{cell}/vision/plate_obb")
    parser.add_argument("--debug-topic-template", default="/{cell}/vision/debug_image")
    parser.add_argument(
        "--image-transport",
        choices=("auto", "raw", "compressed"),
        default="auto",
        help="auto treats topics ending in /compressed as sensor_msgs/CompressedImage.",
    )
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--max-hz", type=float, default=5.0)
    parser.add_argument("--publish-debug", nargs="?", const=True, default=False, type=str_to_bool)
    parser.add_argument("--no-empty", action="store_true", help="Do not publish invalid detections.")
    parser.add_argument(
        "--allowed-class-ids",
        default="",
        help="Comma-separated class ids to accept. Empty accepts all classes.",
    )
    parser.add_argument(
        "--roi",
        default="0.0,0.0,1.0,1.0",
        help="Normalized center ROI x_min,y_min,x_max,y_max. Detections outside are rejected.",
    )
    parser.add_argument(
        "--reject-edge-margin",
        type=float,
        default=0.0,
        help="Normalized image margin. Reject OBBs touching the image edge/gripper border.",
    )
    parser.add_argument("--min-area-ratio", type=float, default=0.0)
    parser.add_argument("--max-area-ratio", type=float, default=1.0)
    parser.add_argument(
        "--aruco-roi-marker-ids",
        default="",
        help="Optional comma-separated 4 marker ids defining a detection ROI polygon, e.g. 0,1,2,3.",
    )
    parser.add_argument("--aruco-dict", default="DICT_6X6_250")
    return parser.parse_args(remove_ros_args()[1:])


def to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def corners_from_xywhr(cx: float, cy: float, width: float, height: float, angle: float) -> tuple[tuple[float, float], ...]:
    c = math.cos(angle)
    s = math.sin(angle)
    local = np.array(
        [
            [-width / 2.0, -height / 2.0],
            [width / 2.0, -height / 2.0],
            [width / 2.0, height / 2.0],
            [-width / 2.0, height / 2.0],
        ],
        dtype=np.float64,
    )
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    pts = local @ rot.T + np.array([cx, cy], dtype=np.float64)
    return tuple((float(x), float(y)) for x, y in pts)


def parse_allowed_class_ids(value: str) -> set[int] | None:
    if not value.strip():
        return None
    return {int(part.strip()) for part in value.split(",") if part.strip()}


def parse_roi(value: str) -> tuple[float, float, float, float]:
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) != 4:
        raise ValueError("--roi must contain x_min,y_min,x_max,y_max")
    x_min, y_min, x_max, y_max = parts
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        raise ValueError("--roi values must satisfy 0 <= min < max <= 1")
    return x_min, y_min, x_max, y_max


def parse_marker_ids(value: str) -> tuple[int, ...]:
    if not value.strip():
        return ()
    ids = tuple(int(part.strip()) for part in value.split(",") if part.strip())
    if len(ids) != 4:
        raise ValueError("--aruco-roi-marker-ids must contain exactly 4 marker ids")
    return ids


def aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is not available.")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detection_passes_filters(
    det: ObbDetection,
    image_shape: tuple[int, int, int],
    allowed_class_ids: set[int] | None,
    roi: tuple[float, float, float, float],
    edge_margin: float,
    min_area_ratio: float,
    max_area_ratio: float,
    aruco_roi_px: np.ndarray | None = None,
) -> bool:
    if allowed_class_ids is not None and det.class_id not in allowed_class_ids:
        return False

    height, width = image_shape[:2]
    if width <= 0 or height <= 0:
        return False

    cx_norm = det.center_px[0] / float(width)
    cy_norm = det.center_px[1] / float(height)
    x_min, y_min, x_max, y_max = roi
    if not (x_min <= cx_norm <= x_max and y_min <= cy_norm <= y_max):
        return False
    if aruco_roi_px is not None:
        center = (float(det.center_px[0]), float(det.center_px[1]))
        if cv2.pointPolygonTest(aruco_roi_px.astype(np.float32), center, False) < 0:
            return False

    corners = np.array(det.corners_px, dtype=np.float64)
    if edge_margin > 0.0:
        margin_x = edge_margin * width
        margin_y = edge_margin * height
        if (
            np.any(corners[:, 0] <= margin_x)
            or np.any(corners[:, 0] >= width - margin_x)
            or np.any(corners[:, 1] <= margin_y)
            or np.any(corners[:, 1] >= height - margin_y)
        ):
            return False

    area_ratio = abs(float(cv2.contourArea(corners.astype(np.float32)))) / float(width * height)
    return min_area_ratio <= area_ratio <= max_area_ratio


def best_obb_detection(
    result,
    image_shape: tuple[int, int, int],
    allowed_class_ids: set[int] | None,
    roi: tuple[float, float, float, float],
    edge_margin: float,
    min_area_ratio: float,
    max_area_ratio: float,
    aruco_roi_px: np.ndarray | None = None,
) -> ObbDetection:
    obb = getattr(result, "obb", None)
    if obb is None or getattr(obb, "xywhr", None) is None:
        return empty_obb()

    xywhr = to_numpy(obb.xywhr)
    if xywhr.size == 0:
        return empty_obb()

    conf = to_numpy(obb.conf) if getattr(obb, "conf", None) is not None else np.ones(len(xywhr))
    cls = to_numpy(obb.cls) if getattr(obb, "cls", None) is not None else np.zeros(len(xywhr))
    xyxyxyxy = to_numpy(obb.xyxyxyxy) if getattr(obb, "xyxyxyxy", None) is not None else None

    for index in np.argsort(conf)[::-1]:
        cx, cy, width, height, angle = xywhr[int(index)].astype(float)
        if xyxyxyxy is not None:
            corners_arr = xyxyxyxy[int(index)].reshape(4, 2)
            corners = tuple((float(x), float(y)) for x, y in corners_arr)
        else:
            corners = corners_from_xywhr(cx, cy, width, height, angle)

        det = ObbDetection(
            valid=True,
            class_id=int(cls[int(index)]),
            confidence=float(conf[int(index)]),
            center_px=(float(cx), float(cy)),
            corners_px=corners,
            width_px=float(width),
            height_px=float(height),
            angle_rad=float(angle),
        )
        if detection_passes_filters(
            det,
            image_shape,
            allowed_class_ids,
            roi,
            edge_margin,
            min_area_ratio,
            max_area_ratio,
            aruco_roi_px,
        ):
            return det

    return empty_obb()


class YoloObbEyeNode(Node):
    def __init__(self, args):
        super().__init__("yolo11s_obb_eye_node")
        self.args = args
        self.bridge = CvBridge()
        self.model = YOLO(args.model)
        self.get_logger().info(f"model: {args.model}")
        self.get_logger().info(f"model classes: {getattr(self.model, 'names', {})}")
        self.min_period = 1.0 / args.max_hz if args.max_hz > 0.0 else 0.0
        self.allowed_class_ids = parse_allowed_class_ids(args.allowed_class_ids)
        self.roi = parse_roi(args.roi)
        self.aruco_roi_marker_ids = parse_marker_ids(args.aruco_roi_marker_ids)
        self.aruco_detector = None
        if self.aruco_roi_marker_ids:
            self.aruco_detector = cv2.aruco.ArucoDetector(
                aruco_dictionary(args.aruco_dict),
                cv2.aruco.DetectorParameters(),
            )
        self.last_infer_time = {}
        self.publishers_by_cell = {}
        self.debug_publishers_by_cell = {}
        self.debug_compressed_publishers_by_cell = {}

        cells = [cell.strip() for cell in args.cells.split(",") if cell.strip()]
        if not cells:
            raise ValueError("At least one cell is required.")

        for cell in cells:
            image_topic = args.image_topic_template.format(cell=cell)
            obb_topic = args.obb_topic_template.format(cell=cell)
            compressed = self.uses_compressed_image(image_topic)
            image_msg_type = CompressedImage if compressed else Image
            self.publishers_by_cell[cell] = self.create_publisher(Float32MultiArray, obb_topic, 10)
            self.create_subscription(
                image_msg_type,
                image_topic,
                functools.partial(self.image_callback, cell=cell),
                qos_profile_sensor_data,
            )
            if args.publish_debug:
                debug_topic = args.debug_topic_template.format(cell=cell)
                self.debug_publishers_by_cell[cell] = self.create_publisher(Image, debug_topic, qos_profile_sensor_data)
                self.debug_compressed_publishers_by_cell[cell] = self.create_publisher(
                    CompressedImage,
                    f"{debug_topic}/compressed",
                    qos_profile_sensor_data,
                )
                self.get_logger().info(f"{cell}: debug -> {debug_topic}, {debug_topic}/compressed")
            transport = "compressed" if compressed else "raw"
            self.get_logger().info(f"{cell}: {image_topic} ({transport}) -> {obb_topic}")
        self.get_logger().info(
            "filters "
            f"allowed_class_ids={sorted(self.allowed_class_ids) if self.allowed_class_ids is not None else 'all'} "
            f"roi={self.roi} edge_margin={args.reject_edge_margin:.3f} "
            f"area=[{args.min_area_ratio:.4f},{args.max_area_ratio:.4f}] "
            f"aruco_roi={self.aruco_roi_marker_ids or 'off'}"
        )

    def uses_compressed_image(self, image_topic: str) -> bool:
        if self.args.image_transport == "compressed":
            return True
        if self.args.image_transport == "raw":
            return False
        return image_topic.endswith("/compressed")

    def frame_from_image_msg(self, msg: Image | CompressedImage) -> np.ndarray:
        if isinstance(msg, CompressedImage):
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            frame_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                raise ValueError("Failed to decode compressed image.")
            return frame_bgr
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detect_aruco_roi(self, frame_bgr: np.ndarray) -> np.ndarray | None:
        if self.aruco_detector is None:
            return None
        corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return None
        centers_by_id = {}
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if marker_id in self.aruco_roi_marker_ids:
                centers_by_id[marker_id] = marker_corners[0].mean(axis=0)
        if any(marker_id not in centers_by_id for marker_id in self.aruco_roi_marker_ids):
            return None
        return np.array([centers_by_id[marker_id] for marker_id in self.aruco_roi_marker_ids], dtype=np.float32)

    def publish_detection(self, cell: str, det: ObbDetection) -> None:
        if det.valid or not self.args.no_empty:
            out = Float32MultiArray()
            out.data = encode_obb(det)
            self.publishers_by_cell[cell].publish(out)

    def publish_debug_image(
        self,
        cell: str,
        msg: Image | CompressedImage,
        frame_bgr: np.ndarray,
        det: ObbDetection,
        aruco_roi_px: np.ndarray | None,
        status: str = "",
    ) -> None:
        debug = frame_bgr.copy()
        if aruco_roi_px is not None:
            cv2.polylines(debug, [aruco_roi_px.astype(np.int32)], True, (0, 255, 0), 2)
        if det.valid:
            pts = np.array(det.corners_px, dtype=np.int32)
            label = "steel_plate" if det.class_id == 0 else "steel_cube"
            cv2.polylines(debug, [pts], True, (0, 255, 255), 2)
            cv2.circle(debug, tuple(np.array(det.center_px, dtype=np.int32)), 4, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"{label} {det.confidence:.2f}",
                tuple(pts[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
        else:
            cv2.putText(debug, status or "no detection", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_publishers_by_cell[cell].publish(debug_msg)
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        ok, encoded = cv2.imencode(".jpg", debug, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            compressed_msg.data = encoded.tobytes()
            self.debug_compressed_publishers_by_cell[cell].publish(compressed_msg)

    def image_callback(self, msg: Image | CompressedImage, cell: str):
        now = time.monotonic()
        last = self.last_infer_time.get(cell, 0.0)
        if self.min_period and now - last < self.min_period:
            return
        self.last_infer_time[cell] = now

        try:
            frame_bgr = self.frame_from_image_msg(msg)
        except Exception as exc:
            self.get_logger().warning(f"{cell}: failed to decode image: {exc}")
            return

        if self.args.publish_debug:
            self.publish_debug_image(cell, msg, frame_bgr, empty_obb(), None, status="running yolo")

        aruco_roi_px = self.detect_aruco_roi(frame_bgr)
        if self.aruco_detector is not None and aruco_roi_px is None:
            det = empty_obb()
            self.publish_detection(cell, det)
            if self.args.publish_debug:
                self.publish_debug_image(cell, msg, frame_bgr, det, None, status="aruco roi missing")
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.model.predict(
            source=frame_rgb,
            imgsz=self.args.imgsz,
            conf=self.args.conf,
            device=self.args.device,
            verbose=False,
        )

        det = (
            best_obb_detection(
                results[0],
                frame_bgr.shape,
                self.allowed_class_ids,
                self.roi,
                self.args.reject_edge_margin,
                self.args.min_area_ratio,
                self.args.max_area_ratio,
                aruco_roi_px,
            )
            if results
            else empty_obb()
        )
        self.publish_detection(cell, det)

        if self.args.publish_debug:
            self.publish_debug_image(cell, msg, frame_bgr, det, aruco_roi_px)


def main():
    args = parse_args()
    rclpy.init()
    node = YoloObbEyeNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
