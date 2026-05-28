import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import urllib.request
import urllib.error


# =============================================================================
# 설정
# =============================================================================

R_SERVER  = "http://127.0.0.1:8765"
ROBOT_ID  = 1
PART_CODE = 1
PROD_QTY  = 1
MATERIALS = [
    {"mat_code": 1, "used_qty": 1.0},
]

LINEAR_THRESHOLD  = 0.05
ANGULAR_THRESHOLD = 0.05
FAST_THRESHOLD    = 0.3

# =============================================================================
# R 서버 헬퍼
# =============================================================================

def r_post(endpoint, payload):
    url  = f"{R_SERVER}{endpoint}"
    body = json.dumps(payload).encode("utf-8")
    req  = urllib.request.Request(
        url, data=body,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            raw = json.loads(r.read())
            return raw[0] if isinstance(raw, list) else raw
    except urllib.error.HTTPError as e:
        return {"ok": False, "approved": False,
                "reason": f"HTTP {e.code}: {e.read().decode()}"}
    except Exception as e:
        return {"ok": False, "approved": False, "reason": str(e)}

def r_get(endpoint):
    try:
        with urllib.request.urlopen(f"{R_SERVER}{endpoint}", timeout=5) as r:
            raw = json.loads(r.read())
            return raw[0] if isinstance(raw, list) else raw
    except Exception as e:
        return {"ok": False, "reason": str(e)}

def unbox(v):
    return v[0] if isinstance(v, list) and len(v) == 1 else v

# =============================================================================
# 동작 분류
# =============================================================================

def classify_motion(lx, ly, az):
    moving   = abs(lx) > LINEAR_THRESHOLD or abs(ly) > LINEAR_THRESHOLD
    rotating = abs(az) > ANGULAR_THRESHOLD
    if not moving and not rotating:
        return "STOP", "⛔ 정지"
    parts = []
    if lx > LINEAR_THRESHOLD:
        parts.append(f"🔼 전진({'빠름' if abs(lx) > FAST_THRESHOLD else '느림'})")
    elif lx < -LINEAR_THRESHOLD:
        parts.append(f"🔽 후진({'빠름' if abs(lx) > FAST_THRESHOLD else '느림'})")
    if ly > LINEAR_THRESHOLD:
        parts.append("➡️  횡이동(우)")
    elif ly < -LINEAR_THRESHOLD:
        parts.append("⬅️  횡이동(좌)")
    if az > ANGULAR_THRESHOLD:
        parts.append(f"↪️  좌회전({'빠름' if abs(az) > FAST_THRESHOLD else '느림'})")
    elif az < -ANGULAR_THRESHOLD:
        parts.append(f"↩️  우회전({'빠름' if abs(az) > FAST_THRESHOLD else '느림'})")
    return "MOVE", " + ".join(parts)

# =============================================================================
# ROS2 노드
# =============================================================================

class DBSyncNode(Node):
    def __init__(self):
        super().__init__("db_sync_node")

        # 1. 속도 명령(/cmd_vel)을 구독합니다.
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.db_updated  = False  # 테스트 중 DB가 무한히 업데이트되는 것을 막는 잠금장치
        self._prev_state = None
        self._cmd_id     = None

        # R 서버 연결 확인
        res = r_get("/health")
        if unbox(res.get("status")) == "ok":
            self.get_logger().info(
                f"✅ R 서버 연결 성공 (mode={unbox(res.get('mode'))})"
            )
        else:
            self.get_logger().warn(
                f"⚠️  R 서버 응답 없음 — {res.get('reason', 'unknown')}"
            )

        self.get_logger().info(
            "🤖 DB 연동 노드 가동 완료! Carter를 앞으로(I키) 전진시켜보세요."
        )

    def cmd_vel_callback(self, msg):
        lx = msg.linear.x
        ly = msg.linear.y
        az = msg.angular.z

        state, label = classify_motion(lx, ly, az)

        # 2. 상태 변화가 있을 때만 출력
        if state != self._prev_state:
            self.get_logger().info(
                f"🚗 {label}  (lx={lx:.2f}, ly={ly:.2f}, az={az:.2f})"
            )
            self._prev_state = state

        # 3. 전진 감지 + 아직 업데이트 안 했다면
        if lx > LINEAR_THRESHOLD and not self.db_updated:
            self.get_logger().info(
                "🚀 전진 이동이 감지되었습니다! 제조 설비로 배달을 시작합니다..."
            )
            self.update_database()
            self.db_updated = True  # 한 번만 작동하도록 락(Lock) 걸기

    def update_database(self):
        # ── R 서버 통신 ───────────────────────────────────────────────────────
        self.get_logger().info("🔍 R 서버 /check_work 요청 중...")
        resp     = r_post("/check_work", {
            "robot_id":  ROBOT_ID,
            "part_code": PART_CODE,
            "prod_qty":  PROD_QTY,
        })
        approved = unbox(resp.get("approved"))
        reason   = unbox(resp.get("reason", ""))
        self._cmd_id = resp.get("cmd_id")

        if not approved:
            self.get_logger().warn(f"⛔ 작업 거부: {reason}")
            return

        self.get_logger().info(f"✅ 작업 허가! cmd_id={self._cmd_id}")

        self.get_logger().info("🔧 R 서버 /update_stock 요청 중...")
        resp2 = r_post("/update_stock", {
            "robot_id":  ROBOT_ID,
            "part_code": PART_CODE,
            "prod_qty":  PROD_QTY,
            "cmd_id":    self._cmd_id,
            "materials": MATERIALS,
        })
        ok = unbox(resp2.get("ok"))

        if ok:
            self.get_logger().info(
                f"✅ DB 업데이트 완료: metal_cube 소모, metal_part +{PROD_QTY} 생산"
            )
            self.get_logger().info(
                f"📊 [현재 상태] 소모 자재={resp2.get('updated_mat_codes')}  "
                f"생산 부품=part_{PART_CODE} +{PROD_QTY}"
            )
        else:
            self.get_logger().error(f"❌ DB 업데이트 실패: {resp2.get('reason')}")


def main(args=None):
    rclpy.init(args=args)
    node = DBSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
