import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MqttReceiverBridge(Node):
    def __init__(self):
        super().__init__("mqtt_receiver_bridge")

        # 파라미터 선언
        self.declare_parameter("mqtt_broker", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_topic", "fms/trigger/arm")

        self.broker = (
            self.get_parameter("mqtt_broker").get_parameter_value().string_value
        )
        self.port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.topic = self.get_parameter("mqtt_topic").get_parameter_value().string_value

        # MQTT에서 받은 신호를 로컬 ROS 2 토픽으로 변환해서 쏴줄 퍼블리셔
        self.ros_pub = self.create_publisher(String, "amr_arrival_trigger", 10)

        # MQTT 클라이언트 연결 및 구독
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message_callback
        try:
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.subscribe(self.topic)
            self.get_logger().info(
                f"✅ MQTT 수신 브릿지 가동! [{self.topic}] 구독 중..."
            )
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 브로커 연결 실패: {e}")

    def on_message_callback(self, client, userdata, msg):
        """FMS 서버로부터 MQTT 메시지가 도착하면 실행됨"""
        if msg.retain:
            self.get_logger().info(
                "👻 이전 테스트의 도착 알림(Retained Message)은 무시합니다."
            )
            return
        
        payload = msg.payload.decode("utf-8")
        self.get_logger().info(f"📥 FMS 트리거 수신 완료: {payload}")

        # 로봇팔 제어기(m0609_controller)가 들을 수 있도록 ROS 2 String 타입으로 변환하여 토스
        ros_msg = String()
        ros_msg.data = payload
        self.ros_pub.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MqttReceiverBridge()
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
