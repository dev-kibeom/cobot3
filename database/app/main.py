import json
from typing import Optional
from fastapi import FastAPI, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy.orm import Session

from app.services.mqtt import mqtt_service
from app.models.database import get_db, init_db, TransportRobot, Workstation, RawMaterialStorage, VisionCamera
from app.services.scheduler import fms_scheduler

# ==========================================
# 1. FastAPI 애플리케이션 초기화
# ==========================================
app = FastAPI(title="Smart Factory FMS Central Server")

@app.on_event("startup")
def startup_event():
    """서버가 켜질 때 DB 테이블을 만들고 MQTT 통신망을 엽니다."""
    init_db()
    fms_scheduler.start()
    mqtt_service.start()

    db = next(get_db())
    
    # 고정 좌표 데이터 시딩 (Seeding)
    try:
        with open("seed_data.json", "r") as f:
            seed = json.load(f)

            # 작업대 로드
            for ws in seed.get("workstations", []):
                if not db.query(Workstation).filter(Workstation.id == ws["id"]).first():
                    new_ws = Workstation(
                        id=ws["id"],
                        name=ws["name"],
                        location_x=ws["x"],
                        location_y=ws["y"],
                        location_yaw=ws["yaw"],
                    )
                    db.add(new_ws)

            # 보관대 로드
            for st in seed.get("storages", []):
                if (
                    not db.query(RawMaterialStorage)
                    .filter(RawMaterialStorage.id == st["id"])
                    .first()
                ):
                    new_st = RawMaterialStorage(
                        id=st["id"],
                        material_type=st["material"],
                        location_x=st["x"],
                        location_y=st["y"],
                        location_yaw=st["yaw"],
                    )
                    db.add(new_st)
        db.commit()
        print("✅ 초기 맵 좌표 데이터 시딩 완료!")
    except FileNotFoundError:
        print("⚠️ seed_data.json 파일이 없어 좌표를 로드하지 못했습니다.")

# ==========================================
# 2. Pydantic 스키마 정의 (클라이언트 요청/응답 검증)
# ==========================================
class PoseGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float

class NodeGoalRequest(BaseModel):
    target_node_id: str  # 예: "STATION-001" 또는 "STORAGE-001"
    
class StatusUpdate(BaseModel):
    status: str
    completed_task: Optional[str] = None  

class OrderRequest(BaseModel):
    material_type: str  # 예: Iron_Cube, Iron_Plate
    station_name: str  # 예: Press, CNC


# ==========================================
# 3. API 엔드포인트 (REST 라우터)
# ==========================================
@app.get("/")
def read_root():
    return {"message": "FMS Central Server is Running"}

# --- [자동 등록] 로봇이 부팅될 때 서버에 자신을 신고하는 곳 ---
@app.post("/api/robots/amr/{robot_id}/register")
def register_amr(robot_id: str, db: Session = Depends(get_db)):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()

    # DB에 로봇이 없다면 새로 만들어줍니다.
    if not robot:
        new_robot = TransportRobot(id=robot_id, model_type="nova_carter", status="IDLE")
        db.add(new_robot)
        db.commit()
        return {"message": f"Successfully registered NEW robot: {robot_id}"}

    # 이미 등록되어 있다면 상태만 IDLE로 초기화해줍니다.
    robot.status = "IDLE"
    db.commit()
    return {"message": f"Robot {robot_id} re-connected and initialized to IDLE."}

# --- [관리자 테스트용] 특정 작업대에 자재 공급 강제 요청하기 ---
@app.post("/api/test/trigger_supply/{station_id}")
def test_trigger_supply(station_id: str, db: Session = Depends(get_db)):
    station = db.query(Workstation).filter(Workstation.id == station_id).first()
    if not station:
        raise HTTPException(status_code=404, detail="Station not found")

    # 깃발을 번쩍 듭니다! (스케줄러가 이걸 보고 로봇을 보냅니다)
    station.needs_supply = True
    db.commit()

    return {"message": f"🚩 {station_id}에 자재 공급 요청이 발령되었습니다!"}

# --- 스마트 팩토리 실시간 주문 접수 API ---
@app.post("/api/orders")
def create_order(request: OrderRequest, db: Session = Depends(get_db)):
    # 1. 요청받은 공정(Press/CNC)을 수행할 수 있는 비어있는(AVAILABLE) 작업대 선점
    workstation = (
        db.query(Workstation)
        .filter(
            Workstation.name == request.station_name, Workstation.status == "AVAILABLE"
        )
        .first()
    )

    if not workstation:
        raise HTTPException(
            status_code=400,
            detail=f"현재 가동 가능한 {request.station_name} 장비(작업대)가 없습니다.",
        )

    # =========================================================
    # 창고 분산 알고리즘 (로봇 점유까지 확인)
    # =========================================================

    # 조건 A: 아직 스케줄러가 가져가지 않은 대기 중인 창고 ID
    pending_storage_ids = [
        ws.allocated_storage_id
        for ws in db.query(Workstation)
        .filter(Workstation.allocated_storage_id != None)
        .all()
    ]

    # 조건 B: 이미 로봇들이 배정받아 이동 중인 물리적 창고 '좌표'들
    active_robots = (
        db.query(TransportRobot).filter(TransportRobot.storage_x != None).all()
    )
    active_storage_coords = [(r.storage_x, r.storage_y) for r in active_robots]

    # STOCKED 상태의 창고를 모두 가져와서 필터링 시작
    all_available_storages = (
        db.query(RawMaterialStorage)
        .filter(
            RawMaterialStorage.material_type == request.material_type,
            RawMaterialStorage.status == "STOCKED",
            ~RawMaterialStorage.id.in_(pending_storage_ids),  # 조건 A 필터링
        )
        .all()
    )

    storage = None
    for st in all_available_storages:
        # 조건 B 필터링: 로봇이 이미 향하고 있는 좌표와 겹치지 않는 첫 번째 창고 선택!
        if (st.location_x, st.location_y) not in active_storage_coords:
            storage = st
            break

    if not storage:
        raise HTTPException(
            status_code=400, detail=f"{request.material_type} 자재의 재고가 부족합니다."
        )

    # 3. 매핑 연동 및 공급 트리거 발동
    workstation.status = "OCCUPIED"  # 다른 주문이 채가지 못하도록 즉시 잠금
    workstation.needs_supply = True
    workstation.allocated_storage_id = storage.id

    db.commit()

    print(
        f"🛒 [주문 접수] 제품 공정: {request.station_name} | 필요 자재: {request.material_type}"
    )
    print(f"   ➔ 매핑 결과: {workstation.id}번 작업대에 {storage.id}번 자재 입고 지시")

    return {
        "status": "주문 완료",
        "message": f"성공적으로 주문이 접수되어 {workstation.id} 장비에 {storage.id} 자재가 배정되었습니다.",
        "assigned_resources": {
            "workstation_id": workstation.id,
            "storage_id": storage.id,
        },
    }

# --- [명령 하달] 관리자가 운반 로봇(AMR)에게 목표를 지정 ---
@app.post("/api/robots/amr/{robot_id}/goal")
def set_amr_goal_by_pose(robot_id: str, goal: PoseGoalRequest, db: Session = Depends(get_db)):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="AMR not found in DB")

    robot.goal_x = goal.x
    robot.goal_y = goal.y
    robot.goal_yaw = goal.yaw
    robot.status = "IDLE"  # 새 명령을 받았으므로 대기 상태로 전환
    db.commit()

    return {"message": f"Goal assigned to {robot_id}"}

# --- [명령 하달] 좌표 대신 '작업대/보관대 ID'로 이동 명령 ---
@app.post("/api/robots/amr/{robot_id}/goal_by_node")
def set_amr_goal_by_node(
    robot_id: str, request: NodeGoalRequest, db: Session = Depends(get_db)
):
    # 1. 대상 로봇이 존재하는지 확인
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="AMR not found in DB")

    # 2. 목적지(작업대)가 Workstation 테이블에 있는지 검색
    target_node = (
        db.query(Workstation).filter(Workstation.id == request.target_node_id).first()
    )

    # 3. 없으면 RawMaterialStorage 테이블에서 검색
    if not target_node:
        target_node = (
            db.query(RawMaterialStorage)
            .filter(RawMaterialStorage.id == request.target_node_id)
            .first()
        )

    # 4. 둘 다 없으면 에러 반환
    if not target_node:
        raise HTTPException(
            status_code=404,
            detail=f"Target node '{request.target_node_id}' not found in DB",
        )

    # 5. 좌표를 찾았으면 로봇의 목적지로 업데이트
    robot.goal_x = target_node.location_x
    robot.goal_y = target_node.location_y
    robot.goal_yaw = target_node.location_yaw
    robot.status = "IDLE"
    db.commit()

    return {
        "message": f"Goal successfully assigned to {robot_id}",
        "target": request.target_node_id,
        "coordinates": {
            "x": target_node.location_x,
            "y": target_node.location_y,
            "yaw": target_node.location_yaw,
        },
    }

# --- C++ 관제탑 하달용 Task API 확장 ---
@app.get("/api/robots/amr/{robot_id}/task")
def get_amr_task(robot_id: str, db: Session = Depends(get_db)):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()

    if robot and robot.status == "IDLE" and robot.current_task_id is not None:
        # C++ Behavior Tree가 두 좌표를 모두 가져갈 수 있도록 확장된 데이터 구조 전송
        return {
            "has_task": True,
            "task_id": robot.current_task_id,
            "task_type": robot.current_task_type,
            "storage": {
                "x": robot.storage_x,
                "y": robot.storage_y,
                "yaw": robot.storage_yaw,
            },
            "goal": {"x": robot.goal_x, "y": robot.goal_y, "yaw": robot.goal_yaw},
        }
    return {"has_task": False}

# --- [전체 조회] DB에 있는 모든 운반 로봇 상태 보기 ---
@app.get("/api/dashboard/robots")
def get_all_robots(db: Session = Depends(get_db)):
    return db.query(TransportRobot).all()

@app.post("/api/robots/amr/{robot_id}/status")
def update_amr_status(
    robot_id: str, update: StatusUpdate, db: Session = Depends(get_db)
):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="AMR not found")

    robot.status = update.status

    if update.status == "ARRIVED":
        trigger_payload = {
            "event": "AMR_ARRIVED",
            "robot_id": robot_id,
            "station_id": robot.current_task_target_node,  # (DB 스키마에 목적지 ID 속성이 있다고 가정)
        }
        
        mqtt_service.publish_trigger("fms/trigger/arm", trigger_payload)

    # C++ 관제탑이 'ReportTaskCompleteToDB' 노드로 IDLE 상태를 보내면 작업 초기화
    elif update.status == "IDLE" and update.completed_task:
        print(
            f"✅ {robot_id}가 임무({update.completed_task})를 완전히 종료하고 대기 상태로 복귀했습니다."
        )
        robot.current_task_id = None
        robot.current_task_type = None

        robot.storage_x = None
        robot.storage_y = None
        robot.storage_yaw = None

    db.commit()
    return {"message": f"Status updated to {update.status}"}