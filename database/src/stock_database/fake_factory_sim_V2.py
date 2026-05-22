"""
fake_factory_sim_V2.py — 스마트 팩토리 공정 시뮬레이션 V2
============================================================
구성:
  두산 M0609 로봇팔 x6  : 제조 설비에서 metal_cube 가공 → metal_part 5개 생산
  Nova Carter (젯봇) x2 : 자재 창고 → 제조 설비 (원자재 공급)
                          제조 설비 → 조립대 (완제품 운반)

공정 플로우 (Isaac Sim 실제 흐름 재현):
  Carter  : 자재 창고에서 metal_cube 1개 픽업
         → 제조 설비 이동
         → 도착 시 POST /check_work  ← Isaac Sim 트리거 포인트
         → approved → ARM에 작업 시작 신호
         → ARM 가공 완료 대기
         → 완제품 픽업
         → 조립대 이동 + 납품
         → 복귀

  ARM     : Carter 도착 신호 대기
         → 가공 시작 (8~14초)
         → 완료 시 POST /update_stock
         → Carter에 픽업 신호

  조립대  : 10초마다 자동으로 비워짐

실행: python3 fake_factory_sim_V2.py
R 서버 먼저 실행: Rscript factory_server_V2.R
"""

import json
import time
import random
import threading
import queue
import urllib.request
import urllib.error
from datetime import datetime

# =============================================================================
# 설정
# =============================================================================

R_SERVER  = "http://127.0.0.1:8765"
SIM_SPEED = 1.0     # 속도 배수

N_ARMS    = 6       # 두산 M0609 수
N_CARTERS = 2       # Jetbot / Carter 수

PART_CODE = 1       # metal_part
MAT_CODE  = 1       # metal_cube
PROD_QTY  = 5       # ARM 1회 생산량
MAT_USED  = 1       # ARM 1회 소모량

ARM_CYCLE_TIME   = (8, 14)   # 가공 소요 시간 (초)
CARTER_TRAVEL_TO = (3, 6)    # 자재창고→제조설비 이동 시간
CARTER_TRAVEL_FM = (3, 6)    # 제조설비→조립대 이동 시간
CARTER_RETURN    = (2, 4)    # 복귀 시간
ASSEMBLY_CLEAR   = 10        # 조립대 비워지는 주기 (초)

# =============================================================================
# 공유 상태
# =============================================================================

sim_state = {
    "running":        True,
    "total_produced": 0,
    "total_denied":   0,
    "total_delivered":0,
    "arm_cycles":     {i: 0 for i in range(1, N_ARMS+1)},
    "carter_trips":   {i: 0 for i in range(1, N_CARTERS+1)},
    "assembly_stock": 0,
    # 로봇 상태: 1=유휴 2=이동중 3=작업중 4=대기
    "arm_status":     {i: 1 for i in range(1, N_ARMS+1)},
    "carter_status":  {i: 1 for i in range(1, N_CARTERS+1)},
}

LOG_LOCK    = threading.Lock()

# Carter → ARM: 작업 시작 이벤트 (carter_id, cmd_id, done_event)
arm_work_queue = queue.Queue()

# ARM → Carter: 픽업 준비 완료 이벤트 per arm
arm_done_events = {i: threading.Event() for i in range(1, N_ARMS+1)}

# Carter 대기 큐 (사용 가능한 carter_id)
carter_idle = queue.Queue()
for i in range(1, N_CARTERS+1):
    carter_idle.put(i)

# =============================================================================
# HTTP 헬퍼
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
        with urllib.request.urlopen(req, timeout=10) as r:
            raw = json.loads(r.read())
            return raw[0] if isinstance(raw, list) else raw
    except urllib.error.HTTPError as e:
        return {"ok": False, "approved": False, "reason": f"HTTP {e.code}"}
    except Exception as e:
        return {"ok": False, "approved": False, "reason": str(e)}

def unbox(v):
    return v[0] if isinstance(v, list) and len(v) == 1 else v

# =============================================================================
# 로그
# =============================================================================

def log(tag, msg, level="INFO"):
    ts = datetime.now().strftime("%H:%M:%S")
    icons = {"INFO":"🔵","OK":"✅","WARN":"⚠️ ","ERR":"❌",
             "MOVE":"🚚","ARM":"🦾","DONE":"🎉","SIM":"🏭"}
    icon = icons.get(level, "  ")
    with LOG_LOCK:
        print(f"  {ts}  {icon}  {tag:22s}  {msg}")

# =============================================================================
# 조립대
# =============================================================================

def assembly_loop():
    log("ASSEMBLY", "가동 시작 — 10초마다 자동 처리", "SIM")
    while sim_state["running"]:
        time.sleep(ASSEMBLY_CLEAR / SIM_SPEED)
        cleared = sim_state["assembly_stock"]
        if cleared > 0:
            sim_state["assembly_stock"]  = 0
            sim_state["total_delivered"] += cleared
            log("ASSEMBLY",
                f"처리 완료 — {cleared}개 소진  "
                f"(총 납품: {sim_state['total_delivered']})", "DONE")

# =============================================================================
# ARM (두산 M0609)
# =============================================================================

def arm_loop(arm_id):
    log(f"ARM-{arm_id}", "준비 완료 — Carter 원자재 수령 대기", "ARM")

    while sim_state["running"]:
        # Carter가 원자재를 가져올 때까지 대기
        sim_state["arm_status"][arm_id] = 4  # 대기
        try:
            task = arm_work_queue.get(timeout=1)
        except queue.Empty:
            continue

        carter_id = task["carter_id"]
        cmd_id    = task["cmd_id"]
        arm_work_queue.task_done()

        # 가공 시작
        sim_state["arm_status"][arm_id] = 3  # 작업중
        cycle_sec = random.uniform(*ARM_CYCLE_TIME) / SIM_SPEED
        log(f"ARM-{arm_id}",
            f"metal_cube 가공 시작 (CARTER-{carter_id} 공급)  "
            f"가공 시간: {cycle_sec:.1f}s", "ARM")
        time.sleep(cycle_sec)

        # 재고 차감 + 생산 기록 → R 서버
        resp = r_post("/update_stock", {
            "robot_id":  arm_id,
            "part_code": PART_CODE,
            "prod_qty":  PROD_QTY,
            "cmd_id":    cmd_id,
            "materials": [{"mat_code": MAT_CODE, "used_qty": MAT_USED}],
        })
        ok = unbox(resp.get("ok"))

        if ok:
            sim_state["total_produced"]     += PROD_QTY
            sim_state["arm_cycles"][arm_id] += 1
            log(f"ARM-{arm_id}",
                f"가공 완료 — metal_part x{PROD_QTY} 생산  "
                f"(총 생산: {sim_state['total_produced']}개)", "DONE")
        else:
            log(f"ARM-{arm_id}",
                f"재고 업데이트 실패 — {resp.get('reason')}", "ERR")

        # Carter에 픽업 준비 완료 신호
        sim_state["arm_status"][arm_id] = 1  # 유휴
        arm_done_events[arm_id].set()

# =============================================================================
# Carter (Nova Carter / Jetbot)
# Isaac Sim 흐름 재현:
#   자재 창고 픽업 → 제조 설비 이동 → 도착(/check_work 트리거)
#   → ARM 가공 대기 → 완제품 픽업 → 조립대 이동 → 납품 → 복귀
# =============================================================================

def carter_loop(carter_id):
    log(f"CARTER-{carter_id}", "준비 완료 — 자재 창고 대기", "MOVE")

    # ARM 순서 할당 (Carter 1 → 홀수 ARM, Carter 2 → 짝수 ARM 순환)
    arm_ids = list(range(carter_id, N_ARMS+1, N_CARTERS))
    arm_idx = 0

    while sim_state["running"]:
        sim_state["carter_status"][carter_id] = 1  # 유휴
        arm_id = arm_ids[arm_idx % len(arm_ids)]
        arm_idx += 1

        # ── 1. 자재 창고: metal_cube 픽업 ─────────────────────────────────────
        sim_state["carter_status"][carter_id] = 2  # 이동중
        log(f"CARTER-{carter_id}",
            f"자재 창고 → metal_cube 픽업 후 제조 설비(ARM-{arm_id})로 이동 중...",
            "MOVE")
        time.sleep(random.uniform(*CARTER_TRAVEL_TO) / SIM_SPEED)

        # ── 2. 제조 설비 도착 → POST /check_work  ← Isaac Sim 트리거 포인트 ──
        log(f"CARTER-{carter_id}",
            f"제조 설비 도착 (ARM-{arm_id}) — R 서버에 작업 허가 요청", "MOVE")

        resp     = r_post("/check_work", {
            "robot_id":  carter_id + 100,  # Carter는 101, 102로 구분
            "part_code": PART_CODE,
            "prod_qty":  PROD_QTY,
        })
        approved = unbox(resp.get("approved"))
        reason   = unbox(resp.get("reason", ""))
        cmd_id   = resp.get("cmd_id", -1)

        if not approved:
            log(f"CARTER-{carter_id}",
                f"작업 거부 — {reason}  잠시 대기 후 재시도", "WARN")
            sim_state["total_denied"] += 1
            time.sleep(5 / SIM_SPEED)
            arm_idx -= 1  # 같은 ARM 재시도
            continue

        log(f"CARTER-{carter_id}",
            f"작업 허가 — cmd_id={cmd_id}  ARM-{arm_id}에 작업 시작 신호 전달", "OK")

        # ── 3. ARM에 작업 시작 신호 ───────────────────────────────────────────
        arm_done_events[arm_id].clear()
        arm_work_queue.put({"carter_id": carter_id, "arm_id": arm_id, "cmd_id": cmd_id})

        # ── 4. ARM 가공 완료 대기 ─────────────────────────────────────────────
        log(f"CARTER-{carter_id}", f"ARM-{arm_id} 가공 완료 대기 중...", "MOVE")
        sim_state["carter_status"][carter_id] = 4  # 대기
        arm_done_events[arm_id].wait()

        # ── 5. 완제품 픽업 후 조립대로 이동 ──────────────────────────────────
        sim_state["carter_status"][carter_id] = 2  # 이동중
        travel_sec = random.uniform(*CARTER_TRAVEL_FM) / SIM_SPEED
        log(f"CARTER-{carter_id}",
            f"metal_part x{PROD_QTY} 픽업 — 조립대로 이동 ({travel_sec:.1f}s)", "MOVE")
        time.sleep(travel_sec)

        # ── 6. 조립대 납품 ────────────────────────────────────────────────────
        sim_state["assembly_stock"]            += PROD_QTY
        sim_state["carter_trips"][carter_id]   += 1
        log(f"CARTER-{carter_id}",
            f"조립대 납품 완료 — metal_part x{PROD_QTY}  "
            f"(총 {sim_state['carter_trips'][carter_id]}회 운반)", "OK")

        # ── 7. 복귀 ───────────────────────────────────────────────────────────
        time.sleep(random.uniform(*CARTER_RETURN) / SIM_SPEED)
        log(f"CARTER-{carter_id}", "자재 창고로 복귀 완료", "MOVE")

# =============================================================================
# 상태 요약
# =============================================================================

def status_loop(interval=20.0):
    STATUS = {1:"IDLE", 2:"MOVE", 3:"WORK", 4:"WAIT"}
    while sim_state["running"]:
        time.sleep(interval / SIM_SPEED)
        with LOG_LOCK:
            print()
            print("  " + "=" * 58)
            print(f"  {'[ STATUS SUMMARY ]':^58}")
            print("  " + "=" * 58)
            print(f"  총 metal_part 생산  : {sim_state['total_produced']:>5}개")
            print(f"  총 조립대 납품      : {sim_state['total_delivered']:>5}개")
            print(f"  조립대 대기 중      : {sim_state['assembly_stock']:>5}개")
            print(f"  작업 거부 횟수      : {sim_state['total_denied']:>5}회")
            print(f"  {'─'*42}")
            for i in range(1, N_ARMS+1):
                st = STATUS.get(sim_state["arm_status"][i], "?")
                print(f"  ARM-{i}  [{st:4s}]  사이클: {sim_state['arm_cycles'][i]:>4}회")
            for i in range(1, N_CARTERS+1):
                st = STATUS.get(sim_state["carter_status"][i], "?")
                print(f"  CARTER-{i}  [{st:4s}]  운반: {sim_state['carter_trips'][i]:>4}회")
            print("  " + "=" * 58)
            print()

# =============================================================================
# 메인
# =============================================================================

def main():
    try:
        with urllib.request.urlopen(f"{R_SERVER}/health", timeout=3) as r:
            health = json.loads(r.read())
            health = health[0] if isinstance(health, list) else health
            if unbox(health.get("status")) != "ok":
                print("❌ R 서버 응답 이상"); return
    except Exception as e:
        print(f"❌ R 서버에 연결할 수 없습니다: {e}")
        print("   먼저 'Rscript factory_server_V2.R' 을 실행하세요.")
        return

    print()
    print("  " + "=" * 58)
    print("  🏭  SMART FACTORY SIMULATION V2")
    print("  " + "=" * 58)
    print(f"  두산 M0609 로봇팔  : {N_ARMS}대")
    print(f"  Nova Carter        : {N_CARTERS}대")
    print(f"  공정               : metal_cube x{MAT_USED} → metal_part x{PROD_QTY}")
    print(f"  ARM 가공 시간      : {ARM_CYCLE_TIME[0]}~{ARM_CYCLE_TIME[1]}초")
    print(f"  Carter 이동 시간   : {CARTER_TRAVEL_TO[0]}~{CARTER_TRAVEL_TO[1]}초")
    print(f"  조립대 처리 주기   : {ASSEMBLY_CLEAR}초")
    print(f"  시뮬레이션 속도    : x{SIM_SPEED}")
    print("  " + "=" * 58)
    print()
    print("  [Isaac Sim 트리거 포인트]")
    print("  Carter 제조 설비 도착 → POST /check_work")
    print("  ARM 가공 완료         → POST /update_stock")
    print("  " + "=" * 58)
    print()

    threads = []

    t = threading.Thread(target=assembly_loop, daemon=True)
    t.start(); threads.append(t)

    for i in range(1, N_CARTERS+1):
        t = threading.Thread(target=carter_loop, args=(i,), daemon=True)
        t.start(); threads.append(t)

    for i in range(1, N_ARMS+1):
        t = threading.Thread(target=arm_loop, args=(i,), daemon=True)
        t.start(); threads.append(t)

    t = threading.Thread(target=status_loop, args=(20,), daemon=True)
    t.start(); threads.append(t)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim_state["running"] = False
        print()
        print("  " + "=" * 58)
        print("  🏁  SIMULATION STOPPED  —  FINAL SUMMARY")
        print("  " + "=" * 58)
        print(f"  총 metal_part 생산  : {sim_state['total_produced']}개")
        print(f"  총 조립대 납품      : {sim_state['total_delivered']}개")
        print(f"  작업 거부 횟수      : {sim_state['total_denied']}회")
        for i in range(1, N_ARMS+1):
            print(f"  ARM-{i} 사이클       : {sim_state['arm_cycles'][i]}회")
        for i in range(1, N_CARTERS+1):
            print(f"  CARTER-{i} 운반      : {sim_state['carter_trips'][i]}회")
        print("  " + "=" * 58)

if __name__ == "__main__":
    main()
