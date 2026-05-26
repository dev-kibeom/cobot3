import json
from fastapi import FastAPI, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy.orm import Session

from app.services.mqtt import mqtt_service
from app.models.database import get_db, init_db, TransportRobot, Workstation, RawMaterialStorage, VisionCamera

# ==========================================
# 1. FastAPI 애플리케이션 초기화
# ==========================================
app = FastAPI(title="Smart Factory FMS Central Server")

@app.on_event("startup")
def startup_event():
    """서버가 켜질 때 DB 테이블을 만들고 MQTT 통신망을 엽니다."""
    init_db()
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

# --- [상태 폴링] 카터 로봇(브릿지 노드)이 2초마다 일거리를 묻는 곳 ---
@app.get("/api/robots/amr/{robot_id}/task")
def get_amr_task(robot_id: str, db: Session = Depends(get_db)):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()

    # 상태가 IDLE이고 목표 좌표가 존재할 때만 작업을 하달
    if robot and robot.status == "IDLE" and robot.goal_x is not None:
        return {
            "has_task": True,
            "goal": {"x": robot.goal_x, "y": robot.goal_y, "yaw": robot.goal_yaw},
        }
    return {"has_task": False}

# --- [전체 조회] DB에 있는 모든 운반 로봇 상태 보기 ---
@app.get("/api/dashboard/robots")
def get_all_robots(db: Session = Depends(get_db)):
    return db.query(TransportRobot).all()

# --- [결과 보고 & 트리거 발동] 카터가 목적지 도착을 알리는 곳 ---
@app.post("/api/robots/amr/{robot_id}/status")
def update_amr_status(
    robot_id: str, update: StatusUpdate, db: Session = Depends(get_db)
):
    robot = db.query(TransportRobot).filter(TransportRobot.id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="AMR not found")

    robot.status = update.status

    # 🚀 핵심: 로봇이 도착(ARRIVED)하면 DB를 정리하고 로봇팔을 깨우는 MQTT 송출!
    if update.status == "ARRIVED":
        robot.goal_x = None
        robot.goal_y = None
        robot.goal_yaw = None

        trigger_payload = {
            "event": "AMR_ARRIVED",
            "robot_id": robot_id,
            "action_required": "START_VISION_GRASPING",
        }
        # 분리해 둔 mqtt_handler의 함수를 호출하여 브로커로 방송
        mqtt_service.publish_trigger("fms/trigger/arm", trigger_payload)

    db.commit()
    return {"message": f"Status updated to {update.status}"}