#!/usr/bin/env python3
"""iw_hub_ROS 통합 chain — robot 01/02 × plus/minus 4 경로 지원.

사용:
  python3 chain_goal.py --robot iw_hub_ROS_01 --mode plus
  python3 chain_goal.py --robot iw_hub_ROS_02 --mode minus --start B --end D

import:
  from chain_goal import ChainGoalSender
  node = ChainGoalSender(robot_ns="/iw_hub_ROS_01")
  node.send("B", 0.0, -12.5, 90.0)
  node.drive_backward("C", 0.0, -15.25, 90.0)

설계:
  - 각 waypoint: NavigateToPose xy 이동 → NEAR-GOAL cancel → align_yaw P-controller
  - reverse 모드: drive_backward (cmd_vel 직접, cross-track 보정)
  - 후진 진입 직전 pickup_approach=True (dolly 영역 lidar dead zone) 자동 toggle
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformException
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# ──────────────────────────────────────────────────────────────────────────
# WAYPOINTS — robot 별 plus/minus 4개 경로
# 형식: (name, x, y, yaw_deg, reverse)
# 고정값(대기/도로 인프라)과 DB값(dolly 픽업/drop) 모두 포함. 추후 DB 통합 시 chain_waypoint_server로 외부 주입.
# ──────────────────────────────────────────────────────────────────────────
ROUTES = {
    ("iw_hub_ROS_01", "plus"): [
        ("A",  0.0,    -8.0,    90.0, False),
        ("B",  0.0,   -12.5,    90.0, False),
        ("C",  0.0,   -15.25,   90.0, True),
        ("D",  0.0,   -12.5,    90.0, False),
        ("E", -1.5,    -5.5,    90.0, False),
        ("F", -1.5,     0.0,     0.0, False),
        ("G", -4.125,   0.0,     0.0, True),
        ("H", -1.5,     0.0,   -90.0, False),
        ("I", -1.5,    -5.5,   -90.0, False),
        ("J",  0.0,    -8.0,    90.0, False),
    ],
    ("iw_hub_ROS_01", "minus"): [
        ("A",  0.0,    -8.0,    90.0, False),
        ("B", -1.5,    -5.5,    90.0, False),
        ("C", -1.5,     0.0,     0.0, False),
        ("D", -4.125,   0.0,     0.0, True),
        ("E", -1.5,     0.0,   -90.0, False),
        ("F", -1.5,    -5.5,   -90.0, False),
        ("G",  0.0,   -12.5,    90.0, False),
        ("H",  0.0,   -15.25,   90.0, True),
        ("I",  0.0,   -12.5,    90.0, False),
        ("J",  0.0,    -8.0,    90.0, False),
    ],
    ("iw_hub_ROS_02", "plus"): [
        ("A",  0.0,    20.0,   -90.0, False),
        ("B",  0.0,    25.0,   -90.0, False),
        ("C",  0.0,    27.75,  -90.0, True),    # 2026-05-27 정정 28.75→27.75
        ("D",  0.0,    25.0,   -90.0, False),
        ("E",  1.5,    17.5,   -90.0, False),
        ("F",  1.5,    12.0,   180.0, False),
        ("G",  4.125,  12.0,   180.0, True),
        ("H",  1.5,    12.0,    90.0, False),
        ("I",  1.5,    17.5,    90.0, False),
        ("J",  0.0,    20.0,   -90.0, False),
    ],
    ("iw_hub_ROS_02", "minus"): [
        ("A",  0.0,    20.0,   -90.0, False),
        ("B",  1.5,    17.5,   -90.0, False),   # 2026-05-26 정정 yaw 90→-90
        ("C",  1.5,    12.0,   180.0, False),   # 2026-05-26 정정 yaw 90→180 (D와 일치)
        ("D",  4.125,  12.0,   180.0, True),    # reverse: C/D yaw 180 일치 → 직선 후진
        ("E",  1.5,    12.0,    90.0, False),   # 2026-05-26 정정 yaw 180→90
        ("F",  1.5,    17.5,    90.0, False),   # 2026-05-26 정정 yaw -90→90
        ("G",  0.0,    25.0,   -90.0, False),
        ("H",  0.0,    27.75,  -90.0, True),    # 2026-05-27 정정 28.75→27.75
        ("I",  0.0,    25.0,   -90.0, False),
        ("J",  0.0,    20.0,   -90.0, False),
    ],
}

# ──────────────────────────────────────────────────────────────────────────
# 공통 상수 (모든 robot/mode 공통)
# ──────────────────────────────────────────────────────────────────────────
WORLD_FRAME = "world"
BASE_FRAME = "base_link"

DIST_PRINT_DELTA = 0.20
STUCK_WARN_SEC = 10.0
NEAR_GOAL_DIST = 0.5
NEAR_GOAL_HOLD = 8.0

YAW_TOL_RAD = 0.0175       # ~1°
YAW_KP = 2.0
YAW_MAX_OMEGA = 1.6
YAW_MIN_OMEGA = 0.20
YAW_ALIGN_TIMEOUT = 60.0
YAW_LOG_EVERY = 1.0

REVERSE_SPEED = 0.3       # 2026-05-27 drift 분석용 — 1.0→0.3 (저속 후진 시 drift 패턴 확인)
REVERSE_DIST_TOL = 0.05
REVERSE_TIMEOUT = 150.0   # 2026-05-27 저속 후진용 — 60→150s (3m ÷ 0.036m/s ≈ 85s 필요)
REVERSE_LOG_EVERY = 1.0
REVERSE_YAW_KP = 3.0
REVERSE_YAW_MAX_OMEGA = 1.0
CROSS_TRACK_KP = 1.5
CROSS_TRACK_MAX_RAD = 0.3


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle(a):
    return ((a + math.pi) % (2.0 * math.pi)) - math.pi


class ChainGoalSender(Node):
    """robot_ns 인자로 어떤 robot이든 제어. 노드 이름은 robot_ns 기반."""

    def __init__(self, robot_ns="/iw_hub_ROS_01"):
        # robot_ns 정규화: 항상 leading slash
        if not robot_ns.startswith("/"):
            robot_ns = "/" + robot_ns
        self._robot_ns = robot_ns
        bare = robot_ns.lstrip("/")
        super().__init__(f"chain_goal_{bare}")

        self._client = ActionClient(self, NavigateToPose, f"{robot_ns}/navigate_to_pose")
        # cmd_vel은 velocity_smoother input topic으로 (jam 방지)
        self._cmd_pub = self.create_publisher(Twist, f"{robot_ns}/cmd_vel_nav", 10)

        # lidar self-filter pickup_approach toggle service
        lidar_filter_node = f"/{bare}_lidar_self_filter"
        self._set_param_client = self.create_client(
            SetParameters, f"{lidar_filter_node}/set_parameters")
        self._lidar_filter_node = lidar_filter_node

        # TF buffer + namespaced subscribe
        self._tf_buffer = Buffer()
        qos_dyn = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE)
        qos_static = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            TFMessage, f"{robot_ns}/tf",
            lambda m: [self._tf_buffer.set_transform(t, "chain") for t in m.transforms],
            qos_dyn)
        self.create_subscription(
            TFMessage, f"{robot_ns}/tf_static",
            lambda m: [self._tf_buffer.set_transform_static(t, "chain") for t in m.transforms],
            qos_static)

        self.get_logger().info(f"[{bare}] action server 대기...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"[{bare}] action server not available!")
            sys.exit(1)
        self.get_logger().info(f"[{bare}] action server OK")

        self.get_logger().info(f"[{bare}] TF buffer 초기 채우기 중 (2s)...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        cur = self.get_robot_yaw()
        if cur is not None:
            self.get_logger().info(f"[{bare}] 초기 robot yaw = {math.degrees(cur):.1f}°  OK")
        else:
            self.get_logger().warn(f"[{bare}] 초기 robot yaw 못 가져옴")

    def get_robot_yaw(self):
        try:
            t = self._tf_buffer.lookup_transform(
                WORLD_FRAME, BASE_FRAME, Time(),
                timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        q = t.transform.rotation
        return quat_to_yaw(q.x, q.y, q.z, q.w)

    def get_robot_xy_yaw(self):
        try:
            t = self._tf_buffer.lookup_transform(
                WORLD_FRAME, BASE_FRAME, Time(),
                timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x,
                t.transform.translation.y,
                quat_to_yaw(q.x, q.y, q.z, q.w))

    def set_pickup_approach(self, enable: bool, timeout=3.0):
        """lidar_self_filter의 pickup_approach toggle (dolly 영역 ±120°·4m dead zone)."""
        if not self._set_param_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(
                f"pickup_approach set skip — {self._lidar_filter_node}/set_parameters 없음")
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = "pickup_approach"
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enable)
        req.parameters = [p]
        future = self._set_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        res = future.result()
        if res and all(r.successful for r in res.results):
            self.get_logger().info(f"pickup_approach → {enable}")
            return True
        self.get_logger().warn(f"pickup_approach set 실패 (enable={enable})")
        return False

    def drive_backward(self, name, tx, ty, hold_yaw_deg):
        """후진 + pickup_approach auto toggle. 시작 전 True, 종료 시 False."""
        self.set_pickup_approach(True)
        try:
            return self._drive_backward_inner(name, tx, ty, hold_yaw_deg)
        finally:
            self.set_pickup_approach(False)

    def _drive_backward_inner(self, name, tx, ty, hold_yaw_deg):
        t0 = time.time()
        start = None
        while start is None and time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            start = self.get_robot_xy_yaw()
        if start is None:
            self.get_logger().warn(f"[{name}] REVERSE aborted — TF lookup fail")
            return False
        sx, sy, _ = start
        target_dist = math.hypot(tx - sx, ty - sy)
        if target_dist > 1e-3:
            ux = (tx - sx) / target_dist
            uy = (ty - sy) / target_dist
        else:
            ux, uy = 0.0, 0.0
        self.get_logger().info(
            f"[{name}] REVERSE start ({sx:.2f},{sy:.2f}) → ({tx:.2f},{ty:.2f}) "
            f"dist={target_dist:.2f}m")

        hold_yaw = math.radians(hold_yaw_deg)
        last_log_t = 0.0
        while time.time() - t0 < REVERSE_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self.get_robot_xy_yaw()
            if cur is None:
                continue
            cx, cy, cyaw = cur
            traveled = math.hypot(cx - sx, cy - sy)
            remaining = target_dist - traveled
            lateral = (cx - sx) * (-uy) + (cy - sy) * ux
            now = time.time()
            if now - last_log_t >= REVERSE_LOG_EVERY:
                self.get_logger().info(
                    f"[{name}] REVERSE traveled={traveled:.2f}m "
                    f"remaining={remaining:.2f}m lateral={lateral:+.3f}m")
                last_log_t = now
            if remaining <= REVERSE_DIST_TOL:
                self._cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"[{name}] REVERSE done at ({cx:.2f},{cy:.2f}) "
                    f"traveled={traveled:.2f}m lateral={lateral:+.3f}m")
                err_x = cx - tx
                err_y = cy - ty
                err_yaw_deg = math.degrees(shortest_angle(hold_yaw - cyaw))
                self.get_logger().info(
                    f"[{name}] ARRIVAL target=({tx:+.3f},{ty:+.3f},{hold_yaw_deg:+.1f}°) "
                    f"actual=({cx:+.3f},{cy:+.3f},{math.degrees(cyaw):+.1f}°) "
                    f"err=({err_x:+.3f},{err_y:+.3f},{err_yaw_deg:+.2f}°)")
                return True
            yaw_corr = -CROSS_TRACK_KP * lateral
            if yaw_corr > CROSS_TRACK_MAX_RAD:
                yaw_corr = CROSS_TRACK_MAX_RAD
            elif yaw_corr < -CROSS_TRACK_MAX_RAD:
                yaw_corr = -CROSS_TRACK_MAX_RAD
            yaw_target_dyn = hold_yaw + yaw_corr
            yaw_err = shortest_angle(yaw_target_dyn - cyaw)
            omega = REVERSE_YAW_KP * yaw_err
            if abs(omega) > REVERSE_YAW_MAX_OMEGA:
                omega = REVERSE_YAW_MAX_OMEGA * (1.0 if omega > 0 else -1.0)
            twist = Twist()
            twist.linear.x = -REVERSE_SPEED
            twist.angular.z = omega
            self._cmd_pub.publish(twist)

        self._cmd_pub.publish(Twist())
        self.get_logger().warn(f"[{name}] REVERSE timeout")
        return False

    def align_yaw(self, name, target_yaw_deg):
        target = math.radians(target_yaw_deg)
        t0 = time.time()
        last_log_t = 0.0

        start_yaw = None
        while start_yaw is None and time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            start_yaw = self.get_robot_yaw()
        if start_yaw is None:
            self.get_logger().warn(f"[{name}] YAW-ALIGN aborted — TF lookup fail")
            return False

        start_err = shortest_angle(target - start_yaw)
        self.get_logger().info(
            f"[{name}] YAW-ALIGN start cur={math.degrees(start_yaw):.1f}° "
            f"target={target_yaw_deg:.1f}° err={math.degrees(start_err):.1f}°")

        while time.time() - t0 < YAW_ALIGN_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self.get_robot_yaw()
            if cur is None:
                continue
            err = shortest_angle(target - cur)
            now = time.time()
            if now - last_log_t >= YAW_LOG_EVERY:
                self.get_logger().info(
                    f"[{name}] YAW cur={math.degrees(cur):.1f}° err={math.degrees(err):.1f}°")
                last_log_t = now
            if abs(err) < YAW_TOL_RAD:
                self._cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"[{name}] YAW-ALIGN done cur={math.degrees(cur):.1f}° "
                    f"err={math.degrees(err):.2f}°")
                return True
            omega = YAW_KP * err
            mag = max(YAW_MIN_OMEGA, min(YAW_MAX_OMEGA, abs(omega)))
            twist = Twist()
            twist.angular.z = mag * (1.0 if err > 0 else -1.0)
            self._cmd_pub.publish(twist)

        self._cmd_pub.publish(Twist())
        cur = self.get_robot_yaw()
        cur_str = f"{math.degrees(cur):.1f}°" if cur is not None else "?"
        self.get_logger().warn(
            f"[{name}] YAW-ALIGN timeout cur={cur_str} target={target_yaw_deg:.1f}°")
        return False

    def send(self, name, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = WORLD_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"[{name}] GOAL ({x}, {y}) yaw={yaw_deg}°")
        last_dist = [None]
        last_change_t = [self.get_clock().now().nanoseconds * 1e-9]
        stuck_warned = [False]
        near_since = [None]
        gh_holder = [None]
        near_done = [False]

        def fb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            d = msg.feedback.distance_remaining
            if last_dist[0] is None or abs(d - last_dist[0]) >= DIST_PRINT_DELTA:
                self.get_logger().info(f"[{name}] dist={d:.2f}m")
                last_dist[0] = d
                last_change_t[0] = now
                stuck_warned[0] = False
            elif now - last_change_t[0] >= STUCK_WARN_SEC and not stuck_warned[0]:
                self.get_logger().warn(f"[{name}] STUCK — dist={d:.2f}m 변동 없음 (>{STUCK_WARN_SEC:.0f}s)")
                stuck_warned[0] = True
            if d <= NEAR_GOAL_DIST:
                if near_since[0] is None:
                    near_since[0] = now
                elif (not near_done[0]) and (now - near_since[0] >= NEAR_GOAL_HOLD) and gh_holder[0] is not None:
                    near_done[0] = True
                    self.get_logger().info(f"[{name}] NEAR-GOAL ({d:.2f}m {NEAR_GOAL_HOLD:.0f}s) → cancel")
                    gh_holder[0].cancel_goal_async()
            else:
                near_since[0] = None

        future = self._client.send_goal_async(goal, feedback_callback=fb)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f"[{name}] REJECTED"); return False
        gh_holder[0] = gh
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        sname = {1:"ACCEPTED",2:"EXECUTING",3:"CANCELING",4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}.get(status, str(status))
        xy_ok = (status == 4) or near_done[0]
        self.get_logger().info(f"[{name}] XY-RESULT: {sname}")
        if not xy_ok:
            return False

        time.sleep(0.5)
        yaw_ok = self.align_yaw(name, yaw_deg)
        self.get_logger().info(f"[{name}] DONE (xy={xy_ok}, yaw={yaw_ok})")
        # 도착 좌표 vs 목표 비교 log
        actual = self.get_robot_xy_yaw()
        if actual is not None:
            ax, ay, ayaw = actual
            err_x = ax - x
            err_y = ay - y
            err_yaw_deg = math.degrees(
                shortest_angle(math.radians(yaw_deg) - ayaw))
            self.get_logger().info(
                f"[{name}] ARRIVAL target=({x:+.3f},{y:+.3f},{yaw_deg:+.1f}°) "
                f"actual=({ax:+.3f},{ay:+.3f},{math.degrees(ayaw):+.1f}°) "
                f"err=({err_x:+.3f},{err_y:+.3f},{err_yaw_deg:+.2f}°)")
        return True

    def execute_sequence(self, waypoints):
        """[(name, x, y, yaw_deg, reverse), ...] 리스트 순차 실행.
        reverse 진입 직전 자동 pickup_approach=True (drive_backward 내부 처리)."""
        for (name, x, y, yaw, reverse) in waypoints:
            if reverse:
                ok = self.drive_backward(name, x, y, yaw)
            else:
                ok = self.send(name, x, y, yaw)
            if not ok:
                self.get_logger().error(f"{name} 실패 — sequence 중단")
                return False
        return True


def main():
    parser = argparse.ArgumentParser(description="iw_hub_ROS chain (01/02 × plus/minus)")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        choices=["iw_hub_ROS_01", "iw_hub_ROS_02"])
    parser.add_argument("--mode", default="plus", choices=["plus", "minus"])
    parser.add_argument("--start", default=None, help="시작 waypoint 이름 (생략 시 처음부터)")
    parser.add_argument("--end", default=None, help="종료 waypoint 이름 (생략 시 끝까지)")
    args = parser.parse_args()

    waypoints = ROUTES[(args.robot, args.mode)]
    start_idx, end_idx = 0, len(waypoints)
    if args.start:
        for i, w in enumerate(waypoints):
            if w[0] == args.start:
                start_idx = i; break
    if args.end:
        for i, w in enumerate(waypoints):
            if w[0] == args.end:
                end_idx = i + 1; break
    sliced = waypoints[start_idx:end_idx]

    rclpy.init()
    node = ChainGoalSender(robot_ns=f"/{args.robot}")
    node.get_logger().info(
        f"실행: {args.robot} {args.mode} {[w[0] for w in sliced]}")
    try:
        node.execute_sequence(sliced)
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
