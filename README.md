# Cobot3 Smart Factory FMS

ROS 2 기반 스마트 팩토리 관제 시스템입니다. AMR 이동, Doosan M0609 매니퓰레이터 제어, 비전 인식, MQTT 트리거, FastAPI 중앙 서버와 SQLite DB를 함께 사용합니다.

## 주요 구성

- `database/`: FastAPI 기반 FMS 중앙 서버, SQLite DB, 작업 스케줄러, MQTT publish
- `ros2_ws/`: ROS 2 워크스페이스
  - `bringup`: Nav2, RViz, AMR 관제, M0609/비전 런치 파일
  - `bridge`: MQTT, 이미지, LiDAR 브릿지 노드
  - `controller`: Doosan M0609 및 그리퍼 컨트롤러
  - `manager`: BehaviorTree.CPP 기반 AMR 작업 관제 노드
  - `vision`: YOLO 객체 인식 및 pixel-to-world 변환 노드
  - `interfaces`: 커스텀 인터페이스 패키지용 자리
- `isaac_envs/`: Isaac Sim 환경 및 테스트 스크립트
- `admin_dashboard/`: 관리자 대시보드 테스트 코드

## 사전 준비

### 시스템 의존성

- Ubuntu + ROS 2 환경
- Nav2
- Python 3.10 이상 권장
- MQTT Broker, 예: Mosquitto
- Isaac Sim, 시뮬레이션을 사용할 경우

ROS 2 패키지 빌드에는 다음 의존성이 필요합니다.

- `rclcpp`
- `nav2_msgs`
- `rclcpp_action`
- `behaviortree_cpp`
- `cpr`
- `nlohmann_json`
- `ament_index_cpp`

## Isaac Sim assets 준비

본 레포지토리를 clone한 후, 반드시 별도 드라이브에 있는 `assets.zip` 파일을 다운로드하여 아래 경로에 압축을 풀어주세요.

```bash
isaac_envs/assets/
```

이 assets가 없으면 Isaac Sim 환경이 정상적으로 로드되지 않습니다.

추가 주의사항:

- Isaac Sim script는 import 순서가 중요합니다.
- assembler가 정상 동작하지 않을 수 있으므로 직접 assemble 후 `.usd` 파일로 만들어 import하는 방식을 권장합니다.
- VS Code integrated extension을 켠 상태에서 실행하는 흐름을 기준으로 작성되어 있습니다.

## FMS 서버 실행

FastAPI 서버는 `database/` 디렉터리 기준으로 실행합니다. 서버 시작 시 DB 테이블을 만들고 `seed_data.json`의 작업대/보관대 좌표를 시딩합니다.

```bash
cd database
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python run.py
```

기본 설정:

- 서버 주소: `0.0.0.0:8001`
- API 문서: `http://127.0.0.1:8001/docs`
- DB 파일: `database/data/smart_factory.db`
- MQTT Broker 기본값: `127.0.0.1:1883`

필요하면 `database/.env` 파일로 설정을 바꿀 수 있습니다.

```env
SERVER_HOST=0.0.0.0
SERVER_PORT=8001
MQTT_BROKER_IP=127.0.0.1
MQTT_PORT=1883
```

## ROS 2 워크스페이스 빌드

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

새 터미널을 열 때마다 `source install/setup.bash`를 다시 실행해야 합니다.

## 실행 방법

### 1. FMS 중앙 서버

```bash
cd database
source .venv/bin/activate
python run.py
```

### 2. AMR Nav2 + BT 관제 실행

다른 터미널에서 실행합니다.

```bash
cd ros2_ws
source install/setup.bash
ros2 launch bringup iw_hub_nav2.launch.py
```

이 런치는 다음 구성을 함께 실행합니다.

- RViz2
- Nav2 bringup
- `bridge/mqtt_trigger`
- `manager/bt_scheduler`

`bt_scheduler`는 시작 시 `IW_HUB_01` 로봇을 FMS 서버에 자동 등록하고, DB에서 작업을 조회해 Nav2 goal을 보냅니다.

### 3. M0609 + pixel-to-world 비전 실행

```bash
cd ros2_ws
source install/setup.bash
ros2 launch bringup m0609_pnp.launch.py
```

### 4. M0609 + YOLO 비전 실행

```bash
cd ros2_ws
source install/setup.bash
ros2 launch bringup m0609_yolo.launch.py
```

## API 예시

서버 상태 확인:

```bash
curl http://127.0.0.1:8001/
```

AMR 등록:

```bash
curl -X POST http://127.0.0.1:8001/api/robots/amr/IW_HUB_01/register
```

작업대 자재 공급 요청:

```bash
curl -X POST http://127.0.0.1:8001/api/test/trigger_supply/STATION-001
```

좌표 기반 AMR 목표 지정:

```bash
curl -X POST http://127.0.0.1:8001/api/robots/amr/IW_HUB_01/goal \
  -H "Content-Type: application/json" \
  -d '{"x": 5.5, "y": -2.0, "yaw": 1.57}'
```

노드 ID 기반 AMR 목표 지정:

```bash
curl -X POST http://127.0.0.1:8001/api/robots/amr/IW_HUB_01/goal_by_node \
  -H "Content-Type: application/json" \
  -d '{"target_node_id": "STATION-001"}'
```

AMR 상태 조회:

```bash
curl http://127.0.0.1:8001/api/dashboard/robots
```

## 작업 흐름

1. FastAPI 서버가 DB와 MQTT 서비스를 시작합니다.
2. AMR 관제 노드가 서버에 로봇을 등록합니다.
3. 작업대에 `needs_supply` 또는 `product_ready` 플래그가 생기면 스케줄러가 IDLE 로봇에 작업을 할당합니다.
4. Behavior Tree 노드가 `/task` API에서 작업을 가져옵니다.
5. Nav2 goal을 전송하고 목적지 도착을 서버와 MQTT로 알립니다.
6. 작업 완료 후 로봇 상태를 다시 `IDLE`로 갱신합니다.

## 주요 파일

- `database/app/main.py`: FastAPI 엔드포인트
- `database/app/models/database.py`: SQLAlchemy DB 모델
- `database/app/services/scheduler.py`: 자동 작업 할당 스케줄러
- `database/app/services/mqtt.py`: MQTT publish 서비스
- `ros2_ws/src/bringup/launch/iw_hub_nav2.launch.py`: AMR/Nav2 통합 런치
- `ros2_ws/src/bringup/launch/m0609_pnp.launch.py`: M0609 + pixel-to-world 런치
- `ros2_ws/src/bringup/launch/m0609_yolo.launch.py`: M0609 + YOLO 런치
- `ros2_ws/src/manager/config/task_tree.xml`: Behavior Tree 정의
- `ros2_ws/src/manager/src/bt_main.cpp`: BT 관제 실행 노드

## 트러블슈팅

- `seed_data.json`을 찾지 못하면 `database/` 디렉터리에서 서버를 실행했는지 확인하세요.
- MQTT 연결 실패 시 broker가 실행 중인지, `.env`의 `MQTT_BROKER_IP`, `MQTT_PORT`가 맞는지 확인하세요.
- ROS 2 패키지를 찾지 못하면 `source ros2_ws/install/setup.bash`를 실행했는지 확인하세요.
- Nav2가 map 또는 params 파일을 찾지 못하면 `colcon build --symlink-install` 후 다시 source 하세요.
- Isaac Sim 환경이 비어 있거나 로드되지 않으면 `isaac_envs/assets/`에 assets 압축이 풀려 있는지 확인하세요.

## 라이선스

현재 패키지 메타데이터의 라이선스는 `TODO` 상태입니다. 배포 전 라이선스 정책을 확정해 주세요.
