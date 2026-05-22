"""
orchestrator.py — 스마트 팩토리 오케스트레이터
================================================
역할:
  1. Supabase robot_state 실시간 구독 → 로봇 상태 파악
  2. 생산 주문 수신 → 작업 할당 결정
  3. Supabase robot_command_carter → Carter에 Nav Goal 발행
  4. Supabase robot_command_arm    → M0609에 작업 지시

실행: python3 orchestrator.py
필요: pip install supabase
"""

import time
import threading
from datetime import datetime
from supabase import create_client, Client

# =============================================================================
# 설정
# =============================================================================

SUPABASE_URL = "https://xxxx.supabase.co"   # 여기에 입력
SUPABASE_KEY = "eyJhbGciOiJIUzI1..."        # 여기에 입력

# 로봇 ID 정의
CARTER_IDS = [1, 2]
ARM_IDS    = [3, 4, 5, 6, 7, 8]

# robot_status 코드
STATUS_IDLE    = 1
STATUS_WORKING = 2
STATUS_ERROR   = 3
STATUS_STOPPED = 4

# Carter command 코드
CMD_CARTER_MOVE    = 1
CMD_CARTER_PICKUP  = 2
CMD_CARTER_DELIVER = 3
CMD_CARTER_STANDBY = 4
CMD_CARTER_STOP    = 5

# ARM command 코드
CMD_ARM_PROCESS  = 1
CMD_ARM_STANDBY  = 2
CMD_ARM_STOP     = 3

# command status 코드
CMD_PENDING  = 0
CMD_RUNNING  = 1
CMD_DONE     = 2
CMD_FAILED   = 3

# location_id (맵 완성 후 채울 것)
LOC_STORAGE  = 1   # 원자재 적재소
LOC_STATION  = 2   # 제조 설비 (ARM 위치)
LOC_ASSEMBLY = 3   # 조립대
LOC_STANDBY  = 4   # 대기 위치

# =============================================================================
# Supabase 클라이언트
# =============================================================================

supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)

def now_str():
    return datetime.now().strftime("%H:%M:%S")

def log(tag, msg, level="INFO"):
    icons = {"INFO": "🔵", "OK": "✅", "WARN": "⚠️ ", "ERR": "❌"}
    print(f"  {now_str()}  {icons.get(level,'  ')}  [{tag:20s}]  {msg}")

# =============================================================================
# 로봇 상태 캐시 (Supabase Realtime으로 갱신)
# =============================================================================

robot_states = {}   # robot_id → state dict
state_lock   = threading.Lock()

def get_idle_carter():
    """유휴 Carter 중 가장 먼저 찾은 ID 반환"""
    with state_lock:
        for cid in CARTER_IDS:
            st = robot_states.get(cid, {})
            if st.get("status", STATUS_IDLE) == STATUS_IDLE:
                return cid
    return None

def get_idle_arm():
    """유휴 ARM 중 가장 먼저 찾은 ID 반환"""
    with state_lock:
        for aid in ARM_IDS:
            st = robot_states.get(aid, {})
            if st.get("status", STATUS_IDLE) == STATUS_IDLE:
                return aid
    return None

def get_robot_pos(robot_id):
    with state_lock:
        st = robot_states.get(robot_id, {})
        return st.get("pos_x", 0), st.get("pos_y", 0), st.get("pos_z", 0)

# =============================================================================
# Supabase 커맨드 발행
# =============================================================================

def issue_carter_command(robot_id: int, command: int,
                          to_loc: int, from_loc: int = None,
                          cargo_type: int = 0, cargo_code: int = None,
                          quantity: float = 0.0,
                          issued_by: str = "orchestrator") -> int:
    """
    Carter에 이동 커맨드 발행
    Returns: 생성된 command id
    """
    payload = {
        "robot_id":   robot_id,
        "command":    command,
        "to_loc":     to_loc,
        "cargo_type": cargo_type,
        "quantity":   quantity,
        "status":     CMD_PENDING,
        "issued_by":  issued_by,
    }
    if from_loc  is not None: payload["from_loc"]   = from_loc
    if cargo_code is not None: payload["cargo_code"] = cargo_code

    res = supabase.table("robot_command_carter").insert(payload).execute()
    cmd_id = res.data[0]["id"]
    log("ORCHESTRATOR",
        f"Carter-{robot_id} cmd={command} to_loc={to_loc}  id={cmd_id}", "OK")
    return cmd_id

def issue_arm_command(robot_id: int, command: int,
                       part_code: int, prod_qty: int = 1,
                       carter_id: int = None,
                       issued_by: str = "orchestrator") -> int:
    """
    M0609에 작업 커맨드 발행
    Returns: 생성된 command id
    """
    payload = {
        "robot_id":  robot_id,
        "command":   command,
        "part_code": part_code,
        "prod_qty":  prod_qty,
        "status":    CMD_PENDING,
        "issued_by": issued_by,
    }
    if carter_id is not None: payload["carter_id"] = carter_id

    res = supabase.table("robot_command_arm").insert(payload).execute()
    cmd_id = res.data[0]["id"]
    log("ORCHESTRATOR",
        f"ARM-{robot_id} cmd={command} part={part_code} qty={prod_qty}  id={cmd_id}", "OK")
    return cmd_id

def wait_command_done(table: str, cmd_id: int,
                       timeout: float = 60.0) -> bool:
    """커맨드 완료(status=2) 또는 실패(status=3) 대기"""
    start = time.time()
    while time.time() - start < timeout:
        res = supabase.table(table)\
                      .select("status")\
                      .eq("id", cmd_id)\
                      .execute()
        if res.data:
            st = res.data[0]["status"]
            if st == CMD_DONE:   return True
            if st == CMD_FAILED: return False
        time.sleep(1)
    log("ORCHESTRATOR", f"timeout waiting cmd_id={cmd_id}", "WARN")
    return False

# =============================================================================
# 생산 공정 — 단일 사이클
# (원자재 픽업 → 제조 → 완제품 납품)
# =============================================================================

def run_production_cycle(part_code: int, prod_qty: int,
                          mat_code: int, mat_qty: float):
    """
    하나의 생산 사이클 실행
    맵 완성 후 location_id 실제 값으로 교체
    """
    log("ORCHESTRATOR",
        f"Production cycle start — part={part_code} qty={prod_qty}", "INFO")

    # ── 1. 유휴 Carter + ARM 할당 ──────────────────────────────────────────────
    carter_id = None
    arm_id    = None

    for _ in range(30):   # 최대 30초 대기
        carter_id = get_idle_carter()
        arm_id    = get_idle_arm()
        if carter_id and arm_id:
            break
        log("ORCHESTRATOR", "Waiting for idle robot...", "INFO")
        time.sleep(1)

    if not carter_id or not arm_id:
        log("ORCHESTRATOR", "No idle robot available — cycle aborted", "WARN")
        return False

    log("ORCHESTRATOR",
        f"Assigned — Carter-{carter_id}, ARM-{arm_id}", "OK")

    # ── 2. Carter: 자재 창고 → 제조 설비 이동 ────────────────────────────────
    cmd_id = issue_carter_command(
        robot_id   = carter_id,
        command    = CMD_CARTER_MOVE,
        from_loc   = LOC_STORAGE,
        to_loc     = LOC_STATION,
        cargo_type = 1,             # 1 = material
        cargo_code = mat_code,
        quantity   = mat_qty,
        issued_by  = f"orchestrator/cycle"
    )

    log("ORCHESTRATOR",
        f"Carter-{carter_id} → 자재 창고 픽업 후 제조 설비 이동", "INFO")

    if not wait_command_done("robot_command_carter", cmd_id):
        log("ORCHESTRATOR", f"Carter-{carter_id} move failed", "ERR")
        return False

    # ── 3. ARM: 가공 시작 ─────────────────────────────────────────────────────
    arm_cmd_id = issue_arm_command(
        robot_id  = arm_id,
        command   = CMD_ARM_PROCESS,
        part_code = part_code,
        prod_qty  = prod_qty,
        carter_id = carter_id,
        issued_by = f"orchestrator/cycle"
    )

    log("ORCHESTRATOR", f"ARM-{arm_id} → 가공 시작", "INFO")

    if not wait_command_done("robot_command_arm", arm_cmd_id, timeout=120):
        log("ORCHESTRATOR", f"ARM-{arm_id} process failed", "ERR")
        return False

    # ── 4. Carter: 완제품 픽업 → 조립대 납품 ────────────────────────────────
    deliver_cmd_id = issue_carter_command(
        robot_id   = carter_id,
        command    = CMD_CARTER_DELIVER,
        from_loc   = LOC_STATION,
        to_loc     = LOC_ASSEMBLY,
        cargo_type = 2,             # 2 = part
        cargo_code = part_code,
        quantity   = float(prod_qty),
        issued_by  = f"orchestrator/cycle"
    )

    log("ORCHESTRATOR",
        f"Carter-{carter_id} → 완제품 픽업 후 조립대 납품", "INFO")

    if not wait_command_done("robot_command_carter", deliver_cmd_id):
        log("ORCHESTRATOR", f"Carter-{carter_id} deliver failed", "ERR")
        return False

    log("ORCHESTRATOR",
        f"Production cycle complete — part={part_code} x{prod_qty}", "OK")
    return True

# =============================================================================
# robot_state polling (sync 클라이언트 — Realtime 미지원으로 polling 대체)
# =============================================================================

STATE_POLL_INTERVAL = 1.0   # 초

def start_state_polling():
    """별도 스레드에서 주기적으로 robot_state를 Supabase에서 읽어 캐시 갱신"""
    def _poll():
        while True:
            try:
                res = supabase.table("robot_state").select("*").execute()
                with state_lock:
                    for row in res.data:
                        prev = robot_states.get(row["robot_id"], {})
                        robot_states[row["robot_id"]] = row
                        # 상태 변화 시 로그
                        if prev.get("status") != row["status"]:
                            log("STATE POLL",
                                f"robot_id={row['robot_id']} "
                                f"status={prev.get('status','?')} → {row['status']} "
                                f"pos=({row.get('pos_x',0):.2f},"
                                f"{row.get('pos_y',0):.2f})", "INFO")
            except Exception as e:
                log("STATE POLL", f"poll error: {e}", "WARN")
            time.sleep(STATE_POLL_INTERVAL)

    t = threading.Thread(target=_poll, daemon=True)
    t.start()
    log("STATE POLL", f"Started (interval={STATE_POLL_INTERVAL}s)", "OK")

# =============================================================================
# 초기 상태 로드
# =============================================================================

def load_initial_states():
    res = supabase.table("robot_state").select("*").execute()
    with state_lock:
        for row in res.data:
            robot_states[row["robot_id"]] = row
    log("ORCHESTRATOR",
        f"Loaded {len(res.data)} robot states from Supabase", "OK")

# =============================================================================
# 메인
# =============================================================================

def main():
    print("\n" + "="*56)
    print("  🏭  SMART FACTORY ORCHESTRATOR")
    print("="*56)
    print(f"  Carter IDs : {CARTER_IDS}")
    print(f"  ARM IDs    : {ARM_IDS}")
    print("="*56 + "\n")

    # 초기 상태 로드
    load_initial_states()

    # robot_state polling 시작
    start_state_polling()

    # ── 테스트용 단일 사이클 실행 ──────────────────────────────────────────────
    # 실전에서는 Nav DB 주문 큐를 polling하거나 구독하는 루프로 교체
    log("ORCHESTRATOR", "Starting test production cycle...", "INFO")

    success = run_production_cycle(
        part_code = 1,      # metal_part
        prod_qty  = 5,
        mat_code  = 1,      # metal_cube
        mat_qty   = 1.0,
    )

    if success:
        log("ORCHESTRATOR", "Test cycle completed successfully", "OK")
    else:
        log("ORCHESTRATOR", "Test cycle failed", "ERR")

    # 유지 (Ctrl+C 종료)
    try:
        log("ORCHESTRATOR", "Running... (Ctrl+C to quit)", "INFO")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log("ORCHESTRATOR", "Shutting down...", "INFO")


if __name__ == "__main__":
    main()
