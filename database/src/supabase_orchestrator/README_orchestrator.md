# 스마트 팩토리 — 오케스트레이터 시스템

Isaac Sim 로봇(Carter, M0609)의 작업을 할당하고 Supabase를 통해 실시간으로 제어하는 시스템입니다.

---

## 시스템 구성

```
Isaac Sim (ROS2)
    │  Odometry 토픽 publish
    ▼
ros2_supabase_bridge.py     ← ROS2 → Supabase 브릿지
    │  robot_state UPSERT
    ▼
Supabase (robot_state)
    │  polling (1초)
    ▼
orchestrator.py             ← 작업 할당 오케스트레이터
    ├── robot_command_carter INSERT  →  nav_node.py → NavigateToPose
    └── robot_command_arm    INSERT  →  Isaac Sim M0609 가공
                                               │
                                        POST /update_stock
                                               │
                                        R 서버 → MySQL + Machbase
```

---

## 사전 요구사항

| 항목 | 버전 |
|---|---|
| Python | 3.10 이상 |
| ROS2 | Humble |
| Nav2 | Humble |
| Supabase 계정 | - |

---

## 설치

```bash
pip install supabase
```

---

## Supabase 테이블 초기화

**Supabase Dashboard → SQL Editor** 에서 실행합니다.

### 테이블 생성

```sql
CREATE OR REPLACE FUNCTION update_timestamp()
RETURNS TRIGGER AS $$
BEGIN NEW.updated_at = NOW(); RETURN NEW; END;
$$ LANGUAGE plpgsql;

-- Carter 이동 커맨드
CREATE TABLE IF NOT EXISTS robot_command_carter (
    id          BIGSERIAL    PRIMARY KEY,
    robot_id    SMALLINT     NOT NULL,
    command     SMALLINT     NOT NULL,
    from_loc    SMALLINT,
    to_loc      SMALLINT     NOT NULL,
    cargo_type  SMALLINT     NOT NULL DEFAULT 0,
    cargo_code  SMALLINT,
    quantity    NUMERIC(12,3) NOT NULL DEFAULT 0,
    status      SMALLINT     NOT NULL DEFAULT 0,
    issued_by   TEXT,
    created_at  TIMESTAMPTZ  NOT NULL DEFAULT NOW(),
    updated_at  TIMESTAMPTZ  NOT NULL DEFAULT NOW()
);

CREATE OR REPLACE TRIGGER trg_carter_updated_at
    BEFORE UPDATE ON robot_command_carter
    FOR EACH ROW EXECUTE FUNCTION update_timestamp();

-- M0609 작업 커맨드
CREATE TABLE IF NOT EXISTS robot_command_arm (
    id          BIGSERIAL    PRIMARY KEY,
    robot_id    SMALLINT     NOT NULL,
    command     SMALLINT     NOT NULL,
    part_code   SMALLINT     NOT NULL,
    prod_qty    INTEGER      NOT NULL DEFAULT 1,
    carter_id   SMALLINT,
    status      SMALLINT     NOT NULL DEFAULT 0,
    issued_by   TEXT,
    created_at  TIMESTAMPTZ  NOT NULL DEFAULT NOW(),
    updated_at  TIMESTAMPTZ  NOT NULL DEFAULT NOW()
);

CREATE OR REPLACE TRIGGER trg_arm_updated_at
    BEFORE UPDATE ON robot_command_arm
    FOR EACH ROW EXECUTE FUNCTION update_timestamp();

-- 로봇 실시간 상태
CREATE TABLE IF NOT EXISTS robot_state (
    robot_id    SMALLINT     PRIMARY KEY,
    robot_type  SMALLINT     NOT NULL,
    pos_x       DOUBLE PRECISION NOT NULL DEFAULT 0,
    pos_y       DOUBLE PRECISION NOT NULL DEFAULT 0,
    pos_z       DOUBLE PRECISION NOT NULL DEFAULT 0,
    rot_w       DOUBLE PRECISION NOT NULL DEFAULT 1,
    status      SMALLINT     NOT NULL DEFAULT 1,
    holding     SMALLINT     NOT NULL DEFAULT 0,
    updated_at  TIMESTAMPTZ  NOT NULL DEFAULT NOW()
);

-- 위치 코드 (맵 완성 후 채울 것)
CREATE TABLE IF NOT EXISTS location (
    location_id   SMALLINT     PRIMARY KEY,
    location_type SMALLINT     NOT NULL,
    pos_x         DOUBLE PRECISION NOT NULL DEFAULT 0,
    pos_y         DOUBLE PRECISION NOT NULL DEFAULT 0,
    pos_z         DOUBLE PRECISION NOT NULL DEFAULT 0
);

-- 초기 로봇 상태
INSERT INTO robot_state (robot_id, robot_type, status) VALUES
    (1, 1, 1), (2, 1, 1),
    (3, 2, 1), (4, 2, 1), (5, 2, 1),
    (6, 2, 1), (7, 2, 1), (8, 2, 1)
ON CONFLICT (robot_id) DO NOTHING;
```

### RLS 비활성화 (테스트 단계)

```sql
ALTER TABLE robot_state          DISABLE ROW LEVEL SECURITY;
ALTER TABLE robot_command_carter DISABLE ROW LEVEL SECURITY;
ALTER TABLE robot_command_arm    DISABLE ROW LEVEL SECURITY;
ALTER TABLE location             DISABLE ROW LEVEL SECURITY;
```

### Realtime 활성화

**Database → Publications → supabase_realtime** 에서 아래 테이블 토글 ON, 또는 SQL Editor에서:

```sql
ALTER PUBLICATION supabase_realtime
  ADD TABLE robot_command_carter, robot_command_arm, robot_state;
```

---

## 설정값 변경

각 파일 상단의 `SUPABASE_URL`, `SUPABASE_KEY`를 입력합니다.

```python
SUPABASE_URL = "https://xxxx.supabase.co"   # Project URL
SUPABASE_KEY = "eyJhbGci..."                # anon (public) key
```

> ⚠️ `service_role` key는 절대 클라이언트 코드에 넣지 마세요.

---

## 실행 순서

### 1. ROS2 → Supabase 브릿지

```bash
# ROS2 환경 소스 후 실행
source /opt/ros/humble/setup.bash
python3 ros2_supabase_bridge.py
```

Isaac Sim에서 Odometry 토픽이 publish되면 Supabase `robot_state`가 0.5초 간격으로 업데이트됩니다.

### 2. 오케스트레이터

```bash
python3 orchestrator.py
```

정상 기동 시 출력:

```
========================================================
  🏭  SMART FACTORY ORCHESTRATOR
========================================================
  Carter IDs : [1, 2]
  ARM IDs    : [3, 4, 5, 6, 7, 8]
========================================================
  ✅  Loaded 8 robot states from Supabase
  ✅  State polling started (interval=1.0s)
  🔵  Starting test production cycle...
```

### 3. Nav 노드 (Carter별로 실행)

```bash
# Carter 1
MY_ROBOT_ID=1 python3 nav_node.py

# Carter 2 (별도 터미널)
MY_ROBOT_ID=2 python3 nav_node.py
```

---

## 코드 수정 포인트 (맵 완성 후)

### 토픽명 수정 (`ros2_supabase_bridge.py`)

```python
ROBOT_TOPIC_MAP = {
    "/carter_1/odom": {"robot_id": 1, "robot_type": 1},  # 실제 토픽명으로 교체
    "/carter_2/odom": {"robot_id": 2, "robot_type": 1},
    "/arm_1/odom":    {"robot_id": 3, "robot_type": 2},
    # ...
}
```

### 위치 좌표 수정 (`nav_node.py`)

```python
LOCATION_POSES = {
    1: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},  # 자재 창고 실제 좌표
    2: {"x": 5.0, "y": 0.0, "z": 0.0, "w": 1.0},  # 제조 설비 실제 좌표
    3: {"x": 10.0,"y": 0.0, "z": 0.0, "w": 1.0},  # 조립대 실제 좌표
    4: {"x": 0.0, "y": 5.0, "z": 0.0, "w": 1.0},  # 대기 위치 실제 좌표
}
```

### 주문 루프 교체 (`orchestrator.py`)

```python
# 현재: 테스트용 단일 사이클
run_production_cycle(part_code=1, prod_qty=5, mat_code=1, mat_qty=1.0)

# 실전: Nav DB 주문 큐 polling으로 교체
while True:
    order = poll_nav_db()
    if order:
        run_production_cycle(**order)
```

---

## 코드 구조 및 역할

| 코드 구성 요소 | 역할 |
|---|---|
| `update_robot_state()` | Supabase robot_state UPSERT |
| `get_idle_carter()` | 유휴 Carter 탐색 (status=1) |
| `get_idle_arm()` | 유휴 ARM 탐색 (status=1) |
| `issue_carter_command()` | robot_command_carter INSERT |
| `issue_arm_command()` | robot_command_arm INSERT |
| `wait_command_done()` | 커맨드 완료(status=2) polling 대기 |
| `run_production_cycle()` | 단일 생산 사이클 실행 |
| `start_state_polling()` | 1초 주기 robot_state 캐시 갱신 |

---

## 커맨드/상태 코드

### status 코드 (공통)

| 코드 | 의미 |
|---|---|
| 0 | pending (Isaac Sim 수신 대기) |
| 1 | running (수행 중) |
| 2 | done (완료) |
| 3 | failed (실패) |

### Carter command 코드

| 코드 | 의미 |
|---|---|
| 1 | move (이동) |
| 2 | pickup (픽업) |
| 3 | deliver (납품) |
| 4 | standby (대기) |
| 5 | stop (정지) |

### ARM command 코드

| 코드 | 의미 |
|---|---|
| 1 | process (가공 시작) |
| 2 | standby (대기) |
| 3 | stop (정지) |

### robot_type 코드

| 코드 | 의미 |
|---|---|
| 1 | Nova Carter |
| 2 | M0609 (로봇팔) |

---

## 파일 구조

```
src/supabase_orchestrator/
├── ros2_supabase_bridge.py  # ROS2 Odometry → Supabase robot_state
├── orchestrator.py          # 생산 주문 → 작업 할당
└── nav_node.py              # Supabase 커맨드 → Nav2 실행
```

---

## 주의사항

- Supabase `anon key` 사용 (service_role key 사용 금지)
- `robot_state` polling은 sync 클라이언트 한계로 Realtime 구독 대신 사용
- `wait_command_done()` 기본 timeout은 60초 (ARM은 120초)
- 맵 좌표 미확정 상태에서는 `LOCATION_POSES` 더미값으로 동작
