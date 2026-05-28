from sqlalchemy import create_engine, Column, String, Float, ForeignKey, Boolean
from sqlalchemy.orm import declarative_base, sessionmaker

from app.core import config

# ==========================================
# 1. 데이터베이스 엔진 및 세션 설정
# ==========================================
engine = create_engine(config.DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# ==========================================
# 2. 데이터베이스 스키마 정의 (테이블 구조)
# ==========================================

# 🚚 [테이블 1] 운반 로봇 (AMR / AGV)
class TransportRobot(Base):
    __tablename__ = "transport_robots"

    id = Column(String, primary_key=True, index=True)  # 예: AMR-001 (기기 고유 ID)
    model_type = Column(String, nullable=False)  # 예: nova_carter, mir250
    status = Column(String, default="IDLE")  # IDLE, MOVING, ARRIVED, ERROR, CHARGING
    battery_level = Column(Float, default=100.0)  # 배터리 잔량 (%)

    current_task_id = Column(String, nullable=True)  # 예: TASK-1234
    current_task_type = Column(String, nullable=True)  # 예: supply, collect, idle

    # 내비게이션 목적지
    goal_x = Column(Float, nullable=True)
    goal_y = Column(Float, nullable=True)
    goal_yaw = Column(Float, nullable=True)

    storage_x = Column(Float, nullable=True)
    storage_y = Column(Float, nullable=True)
    storage_yaw = Column(Float, nullable=True)


# 🦾 [테이블 2] 로봇팔 (Manipulator)
class ManipulatorArm(Base):
    __tablename__ = "manipulator_arms"

    id = Column(String, primary_key=True, index=True)  # 예: ARM-001
    model_type = Column(String, nullable=False)  # 예: doosan_m0609, ur10e
    status = Column(String, default="IDLE")  # IDLE, GRASPING, WAITING, ERROR

    # 현재 수행 중인 작업 정보
    current_task = Column(String, nullable=True)  # 예: PICK_PART_A, DROP_TO_AMR
    base_location = Column(String, nullable=True)  # 설치된 위치 (예: station_1)


# 👁️ [테이블 3] 비전 카메라 (Vision Sensor)
class VisionCamera(Base):
    __tablename__ = "vision_cameras"

    id = Column(String, primary_key=True, index=True)  # 예: CAM-001
    model_type = Column(String, nullable=False)  # 예: realsense_d455
    mount_type = Column(
        String, default="FIXED"
    )  # FIXED(천장/벽면 고정), ARM_MOUNTED(로봇팔 부착)

    status = Column(String, default="ACTIVE")  # ACTIVE, OFFLINE, ERROR
    ip_address = Column(String, nullable=True)  # 엣지 단말기 스트리밍/통신용 IP

    # 만약 로봇팔 끝단에 달려있는 카메라라면, 어떤 로봇팔에 속해있는지 명시 (외래 키 활용)
    attached_arm_id = Column(String, ForeignKey("manipulator_arms.id"), nullable=True)


# 🏭 [테이블 4] 작업대 (Workstation / Station)
class Workstation(Base):
    __tablename__ = "workstations"

    id = Column(String, primary_key=True, index=True)  # 예: STATION-001
    name = Column(String, nullable=False)  # 예: Inspection_Desk, Loading_Dock
    status = Column(String, default="AVAILABLE")  # AVAILABLE, OCCUPIED, MAINTENANCE

    # 작업대의 물리적 좌표 (카터가 찾아가야 할 목표 위치)
    location_x = Column(Float, nullable=False)
    location_y = Column(Float, nullable=False)
    location_yaw = Column(Float, nullable=False)

    # 현재 이 작업대를 차지하고 있는 운반 로봇의 ID (외래 키)
    # 카터가 도착하면 이 값이 'AMR-001'로 채워지고, 떠나면 Null로 비워집니다.
    current_amr_id = Column(String, ForeignKey("transport_robots.id"), nullable=True)

    needs_supply = Column(Boolean, default=False)  # True면 자재 공급 로봇 호출
    product_ready = Column(Boolean, default=False)  # True면 완제품 수거 로봇 호출

    allocated_storage_id = Column(String, nullable=True)


# 📦 [테이블 5] 원자재 보관대 (Raw Material Storage)
class RawMaterialStorage(Base):
    __tablename__ = "raw_material_storage"

    id = Column(String, primary_key=True, index=True)  # 예: STORAGE-001
    material_type = Column(String, nullable=False)  # 예: Iron_Panel, Iron_Cube
    status = Column(String, default="STOCKED")  # STOCKED(재고있음), EMPTY(비어있음)
    quantity = Column(Float, default=10.0)  # 남은 수량

    # 보관대의 물리적 좌표 (카터가 자재를 받으러 갈 위치)
    location_x = Column(Float, nullable=False)
    location_y = Column(Float, nullable=False)
    location_yaw = Column(Float, nullable=False)

# ==========================================
# 3. DB 초기화 및 의존성 주입
# ==========================================
def init_db():
    Base.metadata.create_all(bind=engine)


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
