import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sqlite3
import os


class DBSyncNode(Node):
    def __init__(self):
        super().__init__("db_sync_node")

        # 1. 젯봇의 속도 명령(/cmd_vel)을 구독합니다.
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.db_updated = False  # 테스트 중 DB가 무한히 업데이트되는 것을 막는 잠금장치

        # DB 파일 경로 지정 (스크립트가 실행되는 현재 폴더의 smart_factory.db)
        self.db_path = os.path.join(os.getcwd(), "smart_factory.db")
        self.get_logger().info(
            "🤖 DB 연동 노드 가동 완료! 젯봇을 앞으로(I키) 전진시켜보세요."
        )

    def cmd_vel_callback(self, msg):
        # 2. 젯봇이 앞으로 전진하는 명령(linear.x > 0)을 받았고, 아직 업데이트를 안 했다면!
        if msg.linear.x > 0.0 and not self.db_updated:
            self.get_logger().info(
                "🚀 젯봇의 전진 이동이 감지되었습니다! 빈 작업대로 배달을 시작합니다..."
            )
            self.update_database()
            self.db_updated = True  # 한 번만 작동하도록 락(Lock) 걸기

    def update_database(self):
        try:
            # 3. DB 연결
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            # =========================================================
            # SQL UPDATE 
            # 현재 테스트 방식: workbenches 표에서 station_id가 2인 줄을 찾아서,
            # is_empty를 0으로, current_item을 'Jetbot_Delivery'로 바꿔라!
            # =========================================================
            cursor.execute("""
                UPDATE workbenches 
                SET is_empty = 0, current_item = 'Jetbot_Delivery' 
                WHERE station_id = 2
            """)
            conn.commit()  # 변경사항 저장
            self.get_logger().info(
                "✅ DB 업데이트 완료: 2번 작업대에 물건이 채워졌습니다!"
            )

            # 4. 다시 검색(SELECT), 출력해서 확인
            cursor.execute("SELECT * FROM workbenches WHERE station_id = 2")
            result = cursor.fetchone()
            self.get_logger().info(f"📊 [현재 2번 작업대 상태] : {result}")

            conn.close()
        except Exception as e:
            self.get_logger().error(f"DB 에러 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DBSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
