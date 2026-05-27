import uuid
from apscheduler.schedulers.background import BackgroundScheduler
from sqlalchemy.orm import Session
from app.models.database import SessionLocal, TransportRobot, Workstation


def auto_dispatch_tasks():
    # 백그라운드 스레드이므로 별도의 DB 세션을 열어줍니다.
    db: Session = SessionLocal()
    try:
        # 1. 잉여 로봇(IDLE 상태이면서 할당된 작업이 없는 로봇) 찾기
        idle_robot = (
            db.query(TransportRobot)
            .filter(
                TransportRobot.status == "IDLE", TransportRobot.current_task_id == None
            )
            .first()
        )

        if not idle_robot:
            return  # 놀고 있는 로봇이 없으면 스케줄링 패스

        # 2. [우선순위 1] 자재가 부족한 작업대(needs_supply == True) 찾기
        needy_station = (
            db.query(Workstation).filter(Workstation.needs_supply == True).first()
        )
        if needy_station:
            task_id = f"TASK-SUPPLY-{uuid.uuid4().hex[:6].upper()}"
            idle_robot.current_task_id = task_id
            idle_robot.current_task_type = "supply"
            idle_robot.goal_x = needy_station.location_x
            idle_robot.goal_y = needy_station.location_y
            idle_robot.goal_yaw = needy_station.location_yaw

            # 중복 파견을 막기 위해 깃발을 내립니다.
            needy_station.needs_supply = False
            db.commit()
            print(
                f"⚙️ [Auto Dispatcher] {idle_robot.id}에게 자재 공급 명령({task_id}) 하달! -> {needy_station.name}"
            )
            return

        # 3. [우선순위 2] 제품 수거가 필요한 작업대(product_ready == True) 찾기
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
