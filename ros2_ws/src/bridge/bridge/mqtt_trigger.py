import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MqttTriggerBridge(Node):
    def __init__(self):
        super().__init__("mqtt_trigger_bridge")

        # ROS 2 파라미터 (MQTT 서버 설정)
        self.declare_parameter("mqtt_broker", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_topic", "smart_factory/amr/arrival")

        self.broker = (
            self.get_parameter("mqtt_broker").get_parameter_value().string_value
        )
        self.port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.topic = self.get_parameter("mqtt_topic").get_parameter_value().string_value

        # MQTT 클라이언트 연결
        self.mqtt_client = mqtt.Client()
        try:
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.get_logger().info(
                f"✅ MQTT 브로커 연결 성공 ({self.broker}:{self.port})"
            )
            self.mqtt_client.loop_start()  # 백그라운드 스레드 시작
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 브로커 연결 실패: {e}")

        # ROS 2 구독 설정 (C++ 관제탑의 알림 대기)
        self.subscription = self.create_subscription(
            String, "/amr/arrival_trigger", self.arrival_callback, 10
        )
        self.get_logger().info("📡 ROS 2 도착 알림 수신 대기 중...")

    def arrival_callback(self, msg):
        self.get_logger().info(f"📥 C++ 관제탑으로부터 도착 알림 수신: {msg.data}")

        # ROS 2로 받은 JSON(문자열)을 그대로 MQTT로 발행 (로봇팔 호출)
        try:
            self.mqtt_client.publish(self.topic, msg.data)
            self.get_logger().info(
                f"🚀 MQTT 토픽({self.topic})으로 성공적으로 토스했습니다!"
            )
        except Exception as e:
            self.get_logger().error(f"MQTT 발행 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MqttTriggerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
