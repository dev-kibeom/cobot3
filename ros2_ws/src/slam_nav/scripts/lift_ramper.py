#!/usr/bin/env python3
"""iw_hub_ROS lift ramper daemon — 상위 target을 받아 0.008 m/s 속도로 부드럽게 ramp.

토픽 인터페이스:
  - Subscribe: /<robot>/lift_target (std_msgs/Float64)
      목표 z (0.0 ~ 0.04). 들어올림=0.04, 내림=0.0, 중간값도 가능.
  - Publish:   /<robot>/lift_command (sensor_msgs/JointState)
      lift_joint 명령. position[0]을 매 tick 증분 publish하여 ramp 구현.
      ramp 속도: 0.008 m/s (5초간 0 → 0.04 완주)

사용:
  python3 lift_ramper.py --robot iw_hub_ROS_01

PC-D에서 target 보내기 (예시):
  ros2 topic pub --once /iw_hub_ROS_01/lift_target std_msgs/Float64 "data: 0.04"
  ros2 topic pub --once /iw_hub_ROS_01/lift_target std_msgs/Float64 "data: 0.0"

설계:
  - 노드 내부 state: _current (마지막 publish한 position).
  - target 변경 시: _current → target 으로 매 tick 0.008*dt 만큼 이동.
  - target=current 도달 시 publish 중지 (idle).
  - 초기 _current = 0.0 (Isaac Play 직후 기본 상태 가정).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


JOINT_NAME = "lift_joint"
RAMP_RATE = 0.01           # m/s — 4초간 0 → 0.04 ramp
PUBLISH_FREQ_HZ = 50.0     # 50Hz tick → step = 0.008 * 0.02 = 0.00016 per tick
POS_MIN = 0.0
POS_MAX = 0.04
EPSILON = 1e-5             # current == target 판정


class LiftRamper(Node):
    def __init__(self, robot_ns: str):
        bare = robot_ns.lstrip("/")
        self._target_topic = f"/{bare}/lift_target"
        self._command_topic = f"/{bare}/lift_command"

        super().__init__(f"{bare}_lift_ramper")

        self._current = 0.0    # 마지막 publish 위치
        self._target = 0.0     # 목표 위치
        self._ramping = False

        self.create_subscription(Float64, self._target_topic, self._on_target, 10)
        self._cmd_pub = self.create_publisher(JointState, self._command_topic, 10)

        self._dt = 1.0 / PUBLISH_FREQ_HZ
        self._step = RAMP_RATE * self._dt
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"lift_ramper 시작 (robot={bare})")
        self.get_logger().info(
            f"  subscribe: {self._target_topic} (std_msgs/Float64, 목표 z)")
        self.get_logger().info(
            f"  publish:   {self._command_topic} (sensor_msgs/JointState)")
        self.get_logger().info(
            f"  ramp rate: {RAMP_RATE} m/s, step/tick: {self._step:.5f} m, "
            f"freq: {PUBLISH_FREQ_HZ} Hz")
        self.get_logger().info(
            f"  range: [{POS_MIN}, {POS_MAX}], 초기 current=0.0")

    def _on_target(self, msg: Float64):
        new_target = max(POS_MIN, min(POS_MAX, msg.data))
        if abs(new_target - self._target) > EPSILON:
            self._target = new_target
            self._ramping = True
            self.get_logger().info(
                f"target 변경: {self._target:.4f} (current={self._current:.4f})")

    def _tick(self):
        if not self._ramping:
            return
        diff = self._target - self._current
        if abs(diff) <= self._step:
            # 마지막 step — 정확히 target에 맞춤
            self._current = self._target
            self._ramping = False
            self.get_logger().info(f"ramp 완료 — 위치={self._current:.4f}")
        else:
            self._current += self._step if diff > 0 else -self._step

        msg = JointState()
        msg.name = [JOINT_NAME]
        msg.position = [self._current]
        self._cmd_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="iw_hub_ROS lift ramper daemon")
    parser.add_argument("--robot", default="iw_hub_ROS_01",
                        help="robot bare name (iw_hub_ROS_01 or iw_hub_ROS_02)")
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = LiftRamper(robot_ns=args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
