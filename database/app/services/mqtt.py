import paho.mqtt.client as mqtt
import json

from app.core import config

class MQTTHandler:
    def __init__(self):
        # v2.0 API 규격 적용
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, "fms_central_server"
        )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("✅ [MQTT] Broker 연결 성공! (로봇팔 트리거 대기 및 송출 준비 완료)")
        else:
            print(f"❌ [MQTT] Broker 연결 실패 (에러 코드: {reason_code})")

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        print("⚠️ [MQTT] Broker 연결이 끊어졌습니다.")

    def start(self):
        """서버 기동 시 백그라운드에서 MQTT 루프를 시작합니다."""
        try:
            self.client.connect(config.MQTT_BROKER_IP, config.MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"❌ [MQTT] 브로커에 접속할 수 없습니다: {e}")

    def publish_trigger(self, topic: str, payload: dict):
        """
        FastAPI 라우터에서 쉽게 호출할 수 있는 공용 트리거 송출 함수
        """
        try:
            msg_str = json.dumps(payload)
            self.client.publish(topic, msg_str)
            print(f"📡 [MQTT PUBLISH] {topic} -> {msg_str}")
        except Exception as e:
            print(f"❌ [MQTT PUBLISH ERROR] {e}")


# 싱글톤 인스턴스 생성
mqtt_service = MQTTHandler()
