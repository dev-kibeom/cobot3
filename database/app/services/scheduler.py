import uuid
from apscheduler.schedulers.background import BackgroundScheduler
from sqlalchemy.orm import Session
from app.models.database import (
    SessionLocal,
    TransportRobot,
    Workstation,
    RawMaterialStorage,
)


def auto_dispatch_tasks():
    db: Session = SessionLocal()
    try:
        # 1. 잉여 로봇 찾기
        idle_robot = (
            db.query(TransportRobot)
            .filter(
                TransportRobot.status == "IDLE", TransportRobot.current_task_id == None
            )
            .first()
        )

        if not idle_robot:
            return

        # 2. 자재 공급이 필요한 작업대 찾기
        needy_station = (
            db.query(Workstation).filter(Workstation.needs_supply == True).first()
        )
        if needy_station:
            # 🚀 작업대에 할당된 원자재 창고의 위치 정보 조회
            storage = (
                db.query(RawMaterialStorage)
                .filter(RawMaterialStorage.id == needy_station.allocated_storage_id)
                .first()
            )

            task_id = f"TASK-SUPPLY-{uuid.uuid4().hex[:6].upper()}"
            idle_robot.current_task_id = task_id
            idle_robot.current_task_type = "supply"

            # 🚀 1차 경유지(창고) 좌표 주입
            if storage:
                idle_robot.storage_x = storage.location_x
                idle_robot.storage_y = storage.location_y
                idle_robot.storage_yaw = storage.location_yaw
            else:
                # 방어 코드 (창고 매핑이 누락된 경우 즉시 작업대로 가도록 설정)
                idle_robot.storage_x = needy_station.location_x
                idle_robot.storage_y = needy_station.location_y
                idle_robot.storage_yaw = needy_station.location_yaw

            # 🚀 2차 목적지(작업대) 좌표 주입
            idle_robot.goal_x = needy_station.location_x
            idle_robot.goal_y = needy_station.location_y
            idle_robot.goal_yaw = needy_station.location_yaw

            # 플래그 초기화
            needy_station.needs_supply = False
            needy_station.allocated_storage_id = None
            db.commit()

            print(f"⚙️ [Auto Dispatcher] {idle_robot.id}에게 스마트 라우팅 명령 하달!")
            print(f"   ➔ [1차 경유] 창고: {storage.id if storage else 'None'}")
            print(f"   ➔ [2차 목적] 작업대: {needy_station.id} ({needy_station.name})")
            return

        # 3. 제품 수거가 필요한 작업대 처리 (기존 로직 유지)
        ready_station = (
            db.query(Workstation).filter(Workstation.product_ready == True).first()
        )
        if ready_station:
            task_id = f"TASK-COLLECT-{uuid.uuid4().hex[:6].upper()}"
            idle_robot.current_task_id = task_id
            idle_robot.current_task_type = "collect"
            idle_robot.goal_x = ready_station.location_x
            idle_robot.goal_y = ready_station.location_y
            idle_robot.goal_yaw = ready_station.location_yaw

            # 수거 시에는 창고를 들르지 않으므로 None 처리
            idle_robot.storage_x = None
            idle_robot.storage_y = None
            idle_robot.storage_yaw = None

            ready_station.product_ready = False
            db.commit()
            print(
                f"📦 [Auto Dispatcher] {idle_robot.id}에게 제품 수거 명령({task_id}) 하달! -> {ready_station.name}"
            )
            return

    finally:
        db.close()


# 스케줄러 객체 생성 및 3초 간격 실행 설정
fms_scheduler = BackgroundScheduler()
fms_scheduler.add_job(auto_dispatch_tasks, "interval", seconds=3)
