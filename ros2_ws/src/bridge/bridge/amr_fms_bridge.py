import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
import requests
from math import sin, cos


class FMSBridgeNode(Node):
    def __init__(self):
        super().__init__(
            "fms_bridge_node",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )

        # 실행 시 파라미터로 로봇 ID를 받습니다.
        self.declare_parameter("robot_id", "AMR-001")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        # 서버 IP도 유연하게 변경할 수 있도록 파라미터화
        self.declare_parameter("server_url", "http://127.0.0.1:8001/api/robots/amr")
        self.server_url = (
            self.get_parameter("server_url").get_parameter_value().string_value
        )

        self.state = "IDLE"
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.register_to_server()

        self.poll_timer = self.create_timer(2.0, self.poll_server)
        self.get_logger().info(
            f"FMS Bridge Node Started. Polling server for [{self.robot_id}] tasks..."
        )

    def register_to_server(self):
        """서버가 켜져있지 않을 수도 있으니 예외 처리를 해줍니다."""
        try:
            self.get_logger().info(f"Registering {self.robot_id} to FMS Server...")
            response = requests.post(
                f"{self.server_url}/{self.robot_id}/register", timeout=2.0
            )
            if response.status_code == 200:
                self.get_logger().info(f"✅ {response.json()['message']}")
            else:
                self.get_logger().warn(
                    f"⚠️ Failed to register. Server returned: {response.status_code}"
                )
        except Exception as e:
            self.get_logger().error(
                f"❌ FMS Server is unreachable. Please check the server. ({e})"
            )
            
    def poll_server(self):
        if self.state != "IDLE":
            return

        try:
            response = requests.get(
                f"{self.server_url}/{self.robot_id}/task", timeout=1.0
            )
            if response.status_code == 200:
                data = response.json()
                if data.get("has_task"):
                    goal_data = data["goal"]
                    self.get_logger().info(f"New task received! Goal: {goal_data}")
                    self.send_goal_to_nav2(goal_data)
        except Exception:
            pass

    def send_goal_to_nav2(self, goal_data):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready yet! Retrying...")
            self.state = "IDLE"
            return

        self.state = "MOVING"
        self.update_server_status("MOVING")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal_data["x"]
        goal_msg.pose.pose.position.y = goal_data["y"]

        yaw = goal_data["yaw"]
        goal_msg.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2.0)

        self.get_logger().info("Sending goal to Nav2...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.state = "IDLE"
            self.update_server_status("IDLE")
            return

        self.get_logger().info("Goal accepted, navigating...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info("Goal Reached Successfully!")
            self.update_server_status("ARRIVED")
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
            self.update_server_status("IDLE")

        self.state = "IDLE"

    def update_server_status(self, new_status):
        try:
            requests.post(
                f"{self.server_url}/{self.robot_id}/status",
                json={"status": new_status},
                timeout=1.0,
            )
            self.get_logger().info(f"Reported status [{new_status}] to FMS Server.")
        except Exception as e:
            self.get_logger().warn(f"Failed to update status to FMS Server: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FMSBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
