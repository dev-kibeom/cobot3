# slam_nav — Smart Factory 자율주행 패키지

두 대의 iw_hub_ROS (idealworks AMR)를 Isaac Sim 안에서 자율주행·dolly 운반시키는 ROS 2 Humble 패키지. SLAM+Nav2 기반 절대맵·상대맵 통합, dolly 픽업/drop chain, lift joint 제어, multi-PC 분산 운영 지원.

---

## 🚀 명령어 레퍼런스 (Quick Start)

### 환경 셋업 (모든 PC 공통)

```bash
source /opt/ros/humble/setup.bash
source ~/smart_factory_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101
```

### Nav 호스트 (PC-C) — Nav2 + chain + lift

```bash
# 한 번에 spawn: Nav2 + helper + chain_waypoint_server + lift_ramper × 2 robot + RViz
ros2 launch slam_nav multi_robot_slam.launch.py

# RViz 없이
ros2 launch slam_nav multi_robot_slam.launch.py rviz:=false
```

### Lift 명령 (dolly 들어올림/내려놓음, 4초 ramp)

```bash
# UP — 0 → 0.04 4초간 ramp
ros2 topic pub --once /iw_hub_ROS_01/lift_target std_msgs/Float64 "data: 0.04"

# DOWN — 0.04 → 0 4초간 ramp
ros2 topic pub --once /iw_hub_ROS_01/lift_target std_msgs/Float64 "data: 0.0"

# 임의 위치 (0~0.04 사이)
ros2 topic pub --once /iw_hub_ROS_02/lift_target std_msgs/Float64 "data: 0.02"
```


### 빌드

```bash
cd ~/smart_factory_project/ros2_ws
colcon build --packages-select slam_nav
source install/setup.bash
```

---

## 📝 시행착오 기록 (주요 문제·해결)

이 프로젝트를 진행하면서 마주친 핵심 문제들과 해결 과정. 새로 합류하는 사람이 같은 함정에 빠지지 않게 정리.

### A. USD payload OmniGraph namespace 분할 (Phase 0)

- **현상**: 두 iw_hub_ROS 모두 같은 payload (`Collected_iw_hub_ROS/iw_hub_ROS.usd`) 참조 → `/cmd_vel`, `/tf`, `/scan` 같은 토픽 충돌
- **원인**: payload 안 OmniGraph의 `node_namespace` 가 default라 둘 다 같은 토픽 발행
- **방법 모색**: payload 직접 편집 (불가 — omniverse:// 원격) vs `over` 블록으로 prim 안에서 default override
- **적용**: 각 robot prim 안에 `differential_drive`, `ros_lidars`, `transform_tree_odometry`, `front_hawk` 각각의 `over` 블록 작성 + 자체 `ros2_context` (domain_id=101) 추가
- **결과**: USD 라인 1559 → 3379, 두 robot 토픽 완전 분리 (각 47개)
- **함정**: `world_pose_publisher` OmniGraph가 다른 graph의 ros2_context를 cross-graph 참조 시 런타임에 도메인 0으로 누수 → 각 그래프 자체 `ros2_context` 필수

### B. 회전 시 arc 그림 (직선 spin 안 됨)

- **현상**: DWB가 sampling-based라 `vx>0 + omega>0` trajectory 선택 → 호 그리며 회전
- **원인**: DWB sampling 기본 동작
- **적용**: `nav2_rotation_shim_controller::RotationShimController`로 DWB wrap, `angular_dist_threshold: 0.349 rad (20°)` — 이상이면 spin
- **결과**: cmd_vel 96% spin_only, arc 거의 사라짐

### C. 후진 정밀도 문제 (chain Dolly 픽업)

- **현상**: B(0,-12.5)에서 C(0,-15.25)로 reverse 시 x 오차 0.4m (실제 -0.40, target 0.0)
- **원인 1**: B yaw align 후 잔여 yaw 오차 3.8° → 후진 중 -X drift
- **원인 2**: 후진 명령 속도 캡 (Isaac diff drive plugin이 명령의 ~10%만 실속도로 전환)
- **방법 모색**: YAW_TOL 강화 → 효과 미미. `REVERSE_YAW_KP` 증가 → 효과 미미. → cross-track 보정 도입
- **적용**:
  - `YAW_TOL_RAD 4° → 1°` (시작 yaw 정밀도 ↑)
  - `REVERSE_SPEED 0.15 → 1.0` (10배 증폭으로 cap 회피 → 실속도 ~0.10 m/s)
  - **Cross-track 보정**: target line lateral 편차 측정 → yaw target 동적 조정 (`yaw_target = hold_yaw - K_cross * lateral`, K_cross=1.5)
- **결과**: C 도착 (0.02, -15.20) — x 오차 0.02m, y 0.05m. 후진 내내 lateral ±0.022m 안정. 사용자 허용 (±0.05/±0.10) 내 완전 안착.

### D. cmd_vel jam (chain 평균 속도 1/7)

- **현상**: chain이 `/cmd_vel`에 직접 publish → REVERSE timeout (60s에 1.1m). forward도 4.5m에 132s
- **원인**: velocity_smoother가 같은 `/cmd_vel`에 input timeout 1s zero 발행 → alternating
- **방법 모색**: publisher count 측정 → 7개 (chain + smoother + behavior_server×4 + arrival_stopper)
- **적용**: chain의 cmd_vel publish target을 `cmd_vel` → `cmd_vel_nav` (velocity_smoother input topic)로 변경
- **결과**: 정상 속도 회복. cmd_vel 패턴 정상화.

### E. 회전 속도 튜닝 (사용자 피드백 반복)

- **현상**: 회전이 너무 느림/빠름 (주관)
- **시행착오 사이클**:
  - 1차 2배 (`max_vel_theta 5.0`) → 너무 빠름
  - 2차 1.5배 (3.75) → 약간 느림
  - 최종 1.75배에서 -25% = **약 1.3배** (3.3)
- **적용**: `max_velocity` angular 3.3, `rotate_to_heading_angular_vel` 1.95, `max_vel_theta` 3.3, `acc_lim_theta` 7.9. velocity_smoother도 동기화.
- **함정 발견**: robot2_nav.yaml에 이 튜닝이 미반영 (단일 PC 검증 시 robot1만 사용해서 모름) → 통합 작업 시 발견 + 동기화

### F. 코드 통합/최적화 (helper 8→4, params/RViz 단일 템플릿)

- **현상**: robot1_*/robot2_* helper 4쌍 (8 파일) — namespace만 다른데 매번 양쪽 수정해야 함. params/rviz도 2개씩.
- **원인**: Phase 3 robot2 복제 시 sed 치환만 해놓고 통합 안 함
- **방법 모색**: namespace 인자(`--robot`) 받는 단일 helper 파일 4개로 통합 + nav.yaml/slam.rviz는 `<ROBOT>` placeholder 템플릿 + launch에서 `/tmp/slam_nav_<bare>_*` 로 치환
- **적용**:
  - scripts: 8 → 4 (`world_tf_bridge.py`, `footprint_pub.py`, `lidar_self_filter.py`, `arrival_stopper.py`)
  - launch: 3 → 1 (multi_robot_slam.launch.py + `make_robot_group(N)` factory)
  - params/rviz: 2 → 1 (단일 템플릿)
- **결과**: 빌드 + 두 robot 병렬 chain B→D 검증 통과. C reverse 오차 robot1/robot2 동일 정밀도 (x ±0.02m, y ±0.04m). robot2 누락이던 회전 튜닝 자동 적용.

### G. wheel odom drift 진단 (Isaac vs ROS)

- **현상**: chain 보고 `actual y=27.71`, Isaac UI ground truth `y=28.25` → **+0.54m drift** (3m 후진 거리 대비 18% 오차)
- **원인 분석**:
  - 가설 1: world_tf_bridge 캡쳐 stale → 검증: 캡쳐된 static TF (0, 20, 0.1) = Isaac 현재 world_pose live값과 일치. 캡쳐 자체는 정확.
  - 가설 2: Isaac world_pose_publisher가 live가 아님 → 검증: USD OmniGraph의 `IsaacReadWorldPose` v2가 spawn pose만 cache, 매 tick 재계산 안 함. monitor 2분간 변화 0건.
  - **가설 3 (확정): Isaac diff drive plugin의 `wheelRadius` 와 실제 wheel 메시 크기 불일치** — 명령 cmd_vel/0.08=12.5 rad/s → 실제 mesh 반지름 ≈ 0.0856 × 12.5 rad/s = 1.07 m/s → robot 7% 더 이동
- **방법 모색**:
  - 옵션 A: USD wheel param 보정 (정도) — 사용자 결정
  - 옵션 B: chain_goal.py에 보정 계수 (임시방편)
- **적용**: USD에 `over "differential_controller_01" { custom double inputs:wheelRadius = 0.0856 }` (robot01·02 모두)
- **결과 (부분)**: chain log y는 27.71 그대로 (예상대로 wheel encoder 기반). Isaac y는 큰 변화 없음 (28.239) — drift 본질적 원인은 더 깊은 곳일 수도. 향후 검증 필요.

### H. ReadPrimAttribute caching — live world pose 추적 실패

- **현상**: `world_pose_publisher` OmniGraph가 spawn pose만 발행, robot 이동에 따라 안 갱신됨
- **원인**: `IsaacReadWorldPose` v2 (C++)가 `BaseResetNode` 상속 X → OmniGraph evaluator가 "pure compute" 노드로 취급, 첫 compute 후 outputs 캐시. 매 tick 재계산 안 함.
- **방법 모색**:
  - 옵션 1: `execIn` 입력 추가해 강제 재계산 → 시도, 효과 없음 (노드가 execIn 무시)
  - 옵션 2: `IsaacComputeOdometry` 대체 — 단 outputs:position이 starting-pose-relative라 절대 world 좌표 못 얻음
  - 옵션 3: Python script_node — 복잡
- **결론**: 순수 OmniGraph로 절대 world pose live 발행 어려움. 향후 lift_joint 같은 다른 영역에 영향: WritePrimAttribute → drive target도 마찬가지 캐싱 이슈
- **함정**: 우리가 작성한 USD `over "world_pose_publisher"` 안 read_world_pose에 `inputs:execIn` 추가 → 무시됨 (노드 스펙에 execIn 없음). 롤백.

### I. lift_joint 제어 — xform vs joint, attribute namespace, OmniGraph 안정성

- **현상 1**: lift xform 변경 (0 → 0.04 표시만) — 실제 dolly 안 들어올려짐
- **원인**: `lift`는 단순 Xform이 아니라 **PhysicsPrismaticJoint(`lift_joint`)** 으로 제어 — physics 엔진을 거쳐야 함. xform 값 변경은 USD 저작값만 바꿈.
- **현상 2**: `drive:linear:physics:targetPosition` 으로 명명한 attribute 작성 — 실제 USD의 attribute는 `drive:linear:targetPosition` (`physics:` namespace 없음)
- **원인**: payload USD에서 이미 정의된 attribute 이름을 추적 못함 (속성 namespace 짐작 실패)
- **적용**: attribute 이름을 `drive:linear:targetPosition` 으로 수정
- **현상 3**: 순수 OmniGraph로 ramp 구현 시도 → 1프레임 만에 0.04로 점프 또는 0.00013에서 멈춤
- **원인**: omni.graph.nodes 노드 (Multiply, Clamp 등)는 USD에서 `custom token` 타입(polymorphic)으로 선언해야 하는데 `custom double` 사용 → type resolution 깨짐. + ReadPrimAttribute caching → 매 tick 같은 0 반환
- **방법 모색**:
  - v3 `IsaacArticulationController` (즉시 점프, 작동 확인) 발견 — physics tensor API 통해 joint 직접 명령
  - 그래프 안 ramping → 캐싱 문제로 실패
  - drive damping/stiffness 튜닝 → 너무 높으면 안 움직임, 낮으면 빠름 (튜닝 까다로움)
  - **결론**: 그래프는 immediate (ArticulationController) + ramping은 PC-C 데몬 (lift_ramper.py)
- **적용**:
  - USD `lift_joint_01` (robot01) + `lift_joint_02` (robot02) — 각각 4 노드 (on_tick + ros2_context + subscribe_lift_cmd + articulation_controller)
  - `scripts/lift_ramper.py` 데몬 — `/iw_hub_ROS_0N/lift_target` (Float64) 구독, `/iw_hub_ROS_0N/lift_command` (JointState) 50Hz publish, 0.01 m/s ramp
  - multi_robot_slam.launch.py에 lift_ramper 자동 spawn (사용자 별도 실행 불필요)
- **결과**: `ros2 topic pub --once /iw_hub_ROS_0N/lift_target std_msgs/Float64 "data: 0.04"` 한 줄로 4초 부드러운 lift up. dolly 정상 들어올림.
- **함정**: USD OmniGraph 일반 노드 (`omni.graph.nodes.*`) 와 Isaac/ROS bridge 노드 (`isaacsim.ros2.bridge.*`) 의 USD 타입 선언이 다름. 전자는 polymorphic `token`, 후자는 typed (uint64, string 등). 섞으면 안 됨.

### J. USD 파일 reload + 캐시 / 다중 main_work 파일 혼동

- **현상**: USD 수정 후 Isaac에서 Reload해도 효과 없는 경우 발생
- **원인**: Isaac이 `main_work.usda` 가 아닌 다른 파일(`main_work1.usda`)을 열고 있던 시점이 있었음 (사용자가 "save as"로 만든 사본)
- **방법 모색**: 타이틀 바 + lsof로 어떤 파일 열려있는지 추적
- **적용**: 정리 작업으로 `main_work1.usda`, `postion_coll.usda`, `basic1.usda`, `basic2.usda` 모두 제거. `main_work.usda` 단일 canonical 유지.
- **결과**: 파일 헷갈림 방지. 공유 시에도 단순화.

### K. Multi-PC 아키텍처 결정

- **요구**: 4PC (A: Isaac, B: 로봇팔, C: Nav, D: DB) 분산 운영 (실제는 3PC — B 제외)
- **방법 모색**: ROS 2 DDS multicast 디스커버리 — 같은 `ROS_DOMAIN_ID=101` + 같은 LAN + 같은 RMW면 자동 발견
- **적용**:
  - PC-A: Isaac Sim + USD canonical 보유. ROS 노드 추가 실행 불필요 (USD OmniGraph가 ROS 토픽 직접 발행)
  - PC-C: multi_robot_slam.launch.py 한 번에 모든 노드 spawn (Nav2 + helper + chain_server + lift_ramper)
  - PC-D: dispatcher (DB 폴링 + PoseArray + Float64 publish) — 가장 가벼움, 라즈베리파이도 OK
- **검증 단계**: `ros2 topic list | grep iw_hub_ROS` 로 디스커버리 확인. PC-D `ros2 topic pub --once` 로 수동 명령 흐름 검증 가능.
- **함정**: 시간 동기화 (PC-A의 /clock 사용 — use_sim_time:=true가 기본). 방화벽 (UDP 7400-7500). RMW 통일 (3PC 모두 fastrtps_cpp).

---

## 📋 Claude 작업 가이드 (이 README 갱신 규칙)

> **Claude에게 — 매 세션 시작 시 반드시 이 파일을 처음부터 끝까지 읽고 시작할 것.**
>
> 절차:
> 1. **세션 시작**: README 전체 읽기 → §3 현재 상태, §4 다음 즉시 할 일을 가장 먼저 파악
> 2. **작업 시작 전**: §4의 "다음 즉시 할 일"을 갱신 (무엇을, 왜)
> 3. **작업 진행 중**: 한 단계 끝낼 때마다 §5 진행 로그에 한 줄 추가
> 4. **문제 발생**: §6 형식(현상→원인→방법 모색→적용→결과/성과)으로 새 절을 만들거나 같은 종류 문제 재발 시 기존 절 아래에 `(재발/후속)` 소절 누적
> 5. **작업 종료/일시정지**: §3 현재 상태 + §4 다음 즉시 할 일 반드시 갱신 (다음 세션이 이걸 보고 바로 재개)

---

## 1. 프로젝트 목적 + 시스템 아키텍처

### 1-1. 목적

두 대의 **iw_hub_ROS** (idealworks AMR)가 SLAM 기반 실시간 navigation으로 공장 내
**재료 조달** 작업을 자율 수행. 각 로봇은 **독립**이며 DB에서 받은 서로 다른 명령을 처리.

### 1-2. 로봇 작업 시나리오 (1 사이클)

```
[로봇팔]                      [DB]                      [iw_hub_ROS]
   │                            │                            │
   │ ① 재료 요청                │                            │
   │───────────────────────────▶│                            │
   │                            │ ② 대기 중 로봇 검색         │
   │                            │ ③ Dolly 위치로 이동 명령     │
   │                            │────────────────────────────▶│
   │                            │                            │ ④ DB 지정 경로로 Dolly로 이동
   │                            │                            │ ⑤ Dolly 픽업
   │                            │ ⑥ 다음 명령 요청           │
   │                            │◀───────────────────────────│
   │                            │ ⑦ 재료 수급 위치 명령       │
   │                            │────────────────────────────▶│
   │                            │                            │ ⑧ 수급 위치로 이동·재료 수급
   │                            │ ⑨ 로봇팔 위치 명령          │
   │                            │────────────────────────────▶│
   │                            │                            │ ⑩ 로봇팔 위치로 이동
   │ ⑪ 재료 도착                │                            │
   │◀───────────────────────────────────────────────────────│
   │                            │                            │ ⑫ Dolly 내려놓음
   │                            │ ⑬ 명령 완수 보고            │
   │                            │◀───────────────────────────│
   │                            │ ⑭ 대기 위치로 이동 명령     │
   │                            │────────────────────────────▶│
   │                            │                            │ ⑮ 대기 위치 복귀
```

**Dolly**: 운반용 달구지. iw_hub_ROS가 들고 옮기는 주된 객체.

### 1-3. PC 분담 (A·B·C·D)

| PC | 역할 | 상태 |
|---|---|---|
| **A** | Isaac Sim 구동, 시각화 환경 제공 | 현재 PC가 임시 담당 |
| **B** | 로봇팔 제어 | 현재 PC가 임시 담당 |
| **C** | iw_hub_ROS navigation (SLAM + Nav2) | **이 패키지 = C** |
| **D** | DB — 모든 로봇·좌표·경로 정보, 명령 발급 | 현재 PC가 임시 담당 |

현재는 4PC 통신 전 단계 → 이 PC 하나에서 A·D 역할도 임시 수행. 분리 시점에 ROS_DOMAIN_ID
+ DDS multicast로 통신.

### 1-4. SLAM + Navigation 로직 (이 패키지 핵심)

```
[절대 좌표]                          [상대 좌표]
basic1.png + basic1.yaml             LiDAR scan 실시간 SLAM
(사전 warehouse map)                 (iw_hub_ROS_NN/map)
       │                                    │
       └─────────── 좌표 통합 ──────────────┘
                      │
                      ▼
              Nav2 navigation
                      │
                      ▼
              직선 + spin 경로
              (arc 회피, 90°·45° 회전은 제자리 spin)
```

- **절대 좌표 (master)**: `maps/basic2.{png,yaml}` — 공장 전역 사전맵. world frame.
  origin `(-23.975, -35.975, 0)`, resolution 0.05, 사이즈 960×1553 = 48.0×77.65m.
  (이전: basic1.{png,yaml} — 영역 확장으로 2026-05-26 교체)
- **상대 좌표 (live)**: LiDAR scan → SLAM → `iw_hub_ROS_NN/map` topic. 새 장애물 실시간 추가.
- **통합**: 상대맵의 occupancy를 절대맵 좌표계에 누적 → Nav2 costmap에서 둘 다 회피 대상.
- **경로 패턴**: arc 금지, **직선 segment + 제자리 spin 회전**. `RotationShimController`로 wrap.

---

## 2. 환경 + 워크스페이스

### 2-1. 경로

- **워크스페이스 루트**: `~/smart_factory_project/ros2_ws`
- **패키지 루트**: `~/smart_factory_project/ros2_ws/src/slam_nav` (= 이 디렉토리)
- **빌드**: `cd ~/smart_factory_project/ros2_ws && colcon build --packages-select slam_nav`

### 2-2. 폴더 구조

```
slam_nav/
├── README.md              # 이 파일 — 매 세션 가장 먼저 읽기
├── package.xml            # ament_cmake
├── CMakeLists.txt         # share/ + lib/ 설치 규칙
├── launch/
│   ├── multi_robot_slam.launch.py    # robot1+2 통합 단일 launch (2026-05-27~)
│   └── yield_coordinator.launch.py
├── params/
│   └── nav.yaml           # 단일 템플릿 — <ROBOT> placeholder를 launch에서 치환
├── scripts/               # --robot 인자 받는 통합 helper (2026-05-27~)
│   ├── world_tf_bridge.py
│   ├── footprint_pub.py
│   ├── lidar_self_filter.py
│   ├── arrival_stopper.py
│   ├── chain_goal.py
│   ├── chain_waypoint_server.py
│   └── robot_yield_coordinator.py
├── behavior_trees/        # Nav2 BT XML
├── rviz/
│   └── slam.rviz          # 단일 템플릿 — <ROBOT> placeholder (2026-05-27~)
├── maps/                  # ← basic2.png + basic2.yaml (절대 좌표 사전맵, 2026-05-26~)
├── usd/                   # ← main_work.usda (Isaac Sim scene canonical, 2026-05-27~)
├── config/                # 일반 설정
└── env-hooks/             # ROS_DOMAIN_ID 등 환경 hook
```

### 2-3. ROS 환경

- **배포**: Humble
- **`ROS_DOMAIN_ID`**: 101 (이전 패키지에서 확정. 4PC 통신 시 동일 도메인 필수)
- **소싱**:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/smart_factory_project/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=101
  ```

### 2-4. 기본 입력 파일 위치

| 파일 | 경로 | 역할 |
|---|---|---|
| `basic2.png` | `maps/basic2.png` | 절대 좌표 occupancy grid (960×1553, 48.0×77.65m) — 2026-05-26 갱신 |
| `basic2.yaml` | `maps/basic2.yaml` | nav2 map_server용 메타데이터 (origin (-23.975, -35.975), res 0.05) |
| `main_work.usda` | `usd/main_work.usda` | Isaac Sim scene — 두 iw_hub_ROS, lift_joint_01/02 그래프, 공장 환경 (2026-05-27 canonical) |
| `basic1.{png,yaml}` | `maps/basic1.*` | 이전 절대맵 (479×776, 23.95×38.8m) — 보존 |

---

## 3. 현재 상태 (마지막 갱신: 2026-05-27 — 코드 통합/최적화 단계 진입)

### 3-1. 진행 단계

**Phase 0 — 패키지 초기 셋업** ✅ 완료:
- 패키지 신규 생성 + `basic1.{png,yaml,usda}` 재배치
- `basic1.usda` namespace 분할 (ROS_Clock + 두 prim의 OmniGraph over 통합 블록)
- USD `Duplicate prim` 오류 fix (기존 `over` 블록과 신규 namespace `over` 통합)
- `omni:scripting:scripts` 상대→절대 경로 수정

**Phase 1 — 단일 로봇 (robot1) launch** ✅ 완료:
- `launch/robot1_slam.launch.py` — map_to_odom static + world_tf_bridge + basic1 map_server + Nav2 stack + RViz
- `params/robot1_nav.yaml` — RotationShimController(20°) + DWB(vx10/vθ30) + inflation 0.75 + obs_persist 0.5
- 보조 노드 5종: `world_tf_bridge` / `footprint_pub` (초록 화살표 1.064×0.728×0.330m) / `lidar_self_filter` (back `[-115°,-90°]` ≤0.6m) / `arrival_stopper` (cmd_vel 정지 + marker DELETEALL)
- `rviz/robot1_slam.rviz` — TF text 끔 + footprint marker + goal marker display
- 검증: Nav2 lifecycle active(3), TF chain `world→base_link` 완전, LiDAR 1.5Hz, spin 96% 패턴

**Phase 2 — 일체화 + 단일 navigation 검증** ✅ 완료:
- `scripts/robot1_send_random_goal.py` — basic1 free cell sampling(209,859) → NavigateToPose 발행
- 검증: world 좌표 (-2.80, +1.75) goal → 차체 정확 추적 + 100% straight (arc 0%)
- 절대맵(basic1) ↔ 상대맵(odom) TF chain으로 좌표 일치 확인

**Phase 3 — 두 번째 로봇 (robot2) 복제** ✅ 완료:
- `launch/robot2_slam.launch.py` + `params/robot2_nav.yaml` + `rviz/robot2_slam.rviz`
- `scripts/robot2_*.py` 5종 (sed로 namespace 치환)
- 두 launch 동시 실행 → 두 Nav2 active(3), 토픽 분리 (각 47개), 두 RViz 독립 가동

**Phase 4 — 회피(양보) 코디네이터 A안** ✅ 검증 통과:
- `scripts/robot_yield_coordinator.py` + `launch/yield_coordinator.launch.py`
- 두 robot TF로 거리 계산 → `<2m` 시 robot_id 큰 쪽 `cmd_vel jam(25Hz)` → `>3m` 시 해제 (hysteresis)
- 검증 시나리오 (swap goal): 두 robot 정면 마주 (d=6→0.81m) → t=24.3s 양보 발동(robot02 정지) → robot01이 LiDAR로 robot02 우회 → 옆 통과 → 충돌 0건

**Phase 5 — Dolly 픽업/drop chain + cmd_vel/속도 최적화** ✅ 검증 통과 (2026-05-26):
- `scripts/chain_goal.py` (이전 `chain_goal_robot1.py` 일반화) — robot 01/02 × plus/minus 4 경로 통합. `ChainGoalSender(robot_ns)` 클래스 + argparse `--robot --mode --start --end`.
- 4개 경로 정의 (`ROUTES` dict): A~J waypoint, forward/reverse, 고정/DB 출처 분리. 후진은 `drive_backward()` (cmd_vel 직접) — yaw drift 보정 + **cross-track 보정** (target line lateral 편차를 yaw target 동적 조정).
- **cmd_vel jam 해결**: chain이 `/iw_hub_ROS_0N/cmd_vel`에 직접 publish 시 velocity_smoother와 alternating → 평균 속도 1/7. `/cmd_vel_nav` (smoother input)로 변경 → 정상 속도.
- **회전 속도 1.3배**: max_vel_theta 2.5→3.3, rotate_to_heading_angular_vel 1.5→1.95, YAW_MAX_OMEGA 1.2→1.6 (1.5/2배 시도 후 -25% 절충)
- **YAW tolerance 4°→1°**: 후진 시작 정밀도 강화
- **REVERSE_SPEED 0.15→1.0**: Isaac diff drive plugin이 명령의 ~10%로 실속도 변환 → cap 회피 위해 명령 10배 증폭. 실속도 ~0.10 m/s 안정.
- **pickup_approach 자동 toggle**: B/F 진입 직전 lidar self-filter pickup_approach=True (±120°·4m dead zone), 후진 완료 후 False. drive_backward 내부 try/finally.
- **lidar self-filter pickup_approach 영역**: ±60°·2m → ±120°·4m로 확대 (robot1/2 모두 통일).
- **검증 정확도** (C/D/G/H 후진 4개 평균): x 오차 ±0.05m, y 오차 ±0.09m — 사용자 허용 (x±0.08, y±0.10) 안에 들어옴.

**Phase 6 — Multi-robot 통합 launch + DB topic 인터페이스** ✅ 통합 완료 (2026-05-26):
- `launch/multi_robot_slam.launch.py` — robot1+robot2 helper 8종 + Nav2 2 stack + chain_waypoint_server 2개 + RViz 2개 한 번에 spawn (rviz:=false 옵션). 기존 `robot{1,2}_slam.launch.py` 대체.
- `scripts/chain_waypoint_server.py` — DB(PC-D)가 `/iw_hub_ROS_0N/chain_waypoints` (PoseArray)로 sequence 보내면 자동 실행. **PoseArray 인코딩 (2026-05-27 확장)**: position.x/y=좌표, orientation=yaw, **position.z = 0/1/2/3** (0=forward, 1=reverse, 2=reverse+lift_up+5초, 3=reverse+lift_down+5초). 선택: `header.frame_id="task_id:<n>"`로 작업 ID 전달 시 완료 시 `/<robot>/chain_done` (String) 으로 `task_done:<n>` 발행. robot01/02 독립 처리 (각자 server instance).
- 두 robot 병렬 chain 검증: robot1 (plus→minus) + robot2 (plus→minus) 동시 실행 → 모두 정상 완주. 작업 영역 분리 (robot1: -Y, robot2: +Y).

### 3-2. 시스템 아키텍처 — Phase 별 진행도

| Phase | 내용 | 상태 |
|---|---|---|
| **0** | 패키지 셋업 + USD 분할 | ✅ |
| **1** | 단일 로봇 navigation | ✅ |
| **2** | 절대↔상대 좌표 일체화 검증 | ✅ |
| **3** | 멀티 로봇 동시 운영 | ✅ |
| **4** | 충돌 회피 + 양보 (A안) | ✅ (A안 — D안 확장 가능) |
| **5** | Dolly 픽업/drop chain + 후진 정밀도 | ✅ (cross-track 보정, 후진 오차 ±0.05m) |
| **6** | Multi-robot 통합 launch + DB topic 인터페이스 | ✅ (chain_waypoint_server, multi_robot_slam.launch) |
| **7** | Dolly lift_joint 제어 (ROS articulation) | ⏳ Isaac action graph 설정 필요 |
| **8** | SLAM 통합 (mapping 실시간) | ⏳ 현재는 obstacle_layer raw LiDAR만 |
| **9** | 4PC 분산 운영 (PC-A 로봇팔 + PC-D DB + PC-C nav + Isaac) | ⏳ |

### 3-3. 알려진 이슈 (현재 상태)

| # | 이슈 | 영향 | 우선순위 | 노트 |
|---|---|---|---|---|
| 1 | **A안 단순 정지 한계** — `cmd_vel jam`만 — Nav2 `navigate_to_pose`는 계속 active 상태라 progress_checker stuck 인식 가능 | 양보 길어지면 controller ABORT 발생 가능 | 중간 | D안(NavigateToPose cancel + 메모리 goal 재발행)으로 업그레이드 검토 |
| 2 | **yaw 정렬 stuck** — 도착지 근처 장애물 있으면 yaw 정렬 시도 시 진동, SUCCEEDED 못 받음 | 단일 goal로는 인식 어려움. waypoint 모드에서 자주 발생 | 중간 | `yaw_align_watchdog.py` 미작성 — 이전 패키지에서 검증된 패턴 이식 필요 |
| 3 | **LiDAR 1.5Hz** — RTX render product 제약 | Nav2 reactive 응답 느림 | 시뮬 한계 (해결 불가) | 실제 로봇 이식 시엔 자연 해결 |
| 4 | **AMCL Phase 2 미적용** — wheel odom yaw drift 누적 시 LiDAR scan world 변환 어긋남 가능 | 장시간 운영 시 obstacle marking 부정확 | 중간 (장기) | nav.yaml의 `amcl:` 섹션은 placeholder로 작성됨. 적용 여부는 검토 필요 |
| 5 | **`<exec_depend>slam_toolbox</exec_depend>` 잔존** — `package.xml`에 미사용 의존성 | 실행 영향 0 (선언만) | 낮음 | 정확성 차원에서 제거 가능, 사용자 결정 보류 |
| 6 | **robot1/robot2 params 튜닝 불일치** — 2026-05-26 회전속도/yaw tolerance 튜닝이 robot1_nav.yaml에만 적용되고 robot2_nav.yaml에는 미반영 | robot2가 robot1보다 회전 느림 | **2026-05-27 통합 작업에서 해결 예정** | nav.yaml 단일 템플릿화 시 robot1 값으로 동기화 |

**해결/폐기된 이전 항목**:
- ~~robot01 좁은 통로 stuck~~ → 통로 자체를 안 생기게 환경 보정으로 해결 (2026-05-27)
- ~~iwhub_grab_kinematic.py dead reference~~ → 폐기. lift_joint 제어 자체 불용 (2026-05-27)
- ~~양보 해제 미검증~~ → 적용 여부 자체 미정으로 보류 (2026-05-27)

---

## 4. 다음 즉시 할 일

| 순번 | 작업 | 상태 |
|---|---|---|
| 1 | **helper script 통합 (오늘 메인 작업)** — robot1_*/robot2_* helper 4쌍 → `--robot` 인자 받는 단일 파일 4개. params/nav.yaml도 단일 템플릿 + RewrittenYaml로 namespace 치환. legacy `robot{1,2}_slam.launch.py`·`robot{1,2}_nav.yaml` 제거. multi_robot_slam.launch.py가 신규 통합 스크립트 호출하도록 갱신. robot2 params 누락 튜닝(회전속도/yaw tolerance)도 같이 동기화 | **현재 (2026-05-27)** |
| 2 | **forward 도착 정밀도 개선** — NavigateToPose의 NEAR-GOAL HOLD 8s cancel로 오차 0.2~1.0m. 후진(±0.05m)처럼 정밀하게 하려면 NEAR-GOAL 후 추가 정렬 단계 또는 NEAR_GOAL_DIST 축소 | 대기 |
| 3 | **DB 명령 dispatcher (검토 필요)** — `chain_waypoint_server`가 단일 sequence만 받음. 큐잉(plus→minus 연쇄) + 명령 ID 추적 + 완료 보고 토픽. 실제 적용 시점/범위 검토 | 검토 |
| 4 | **AMCL Phase 2 (검토 필요)** — wheel odom yaw drift 보정. `nav.yaml`의 amcl 섹션 활성화 + initial_pose 자동 설정. 적용 여부 자체 검토 | 검토 |

**폐기/보류 항목 (이전 §4 목록에서 정리)**:
- ~~lift_joint 제어~~ → 불용 (2026-05-27)
- ~~iwhub_grab_kinematic.py 작성~~ → 폐기 (2026-05-27)
- ~~양보 해제 검증~~ → 적용 여부 미정으로 보류 (2026-05-27)
- ~~좁은 통로 stuck~~ → 통로 자체 안 생기게 환경 보정 완료 (2026-05-27)

---

## 5. 진행 로그 (시간순, 최신이 아래)

### 2026-05-24 — Phase 0: 패키지 신규 생성

- `~/smart_factory_project/ros2_ws/src/slam_nav/` ament_cmake 패키지 생성 — `package.xml`,
  `CMakeLists.txt`, 표준 폴더 (launch/params/scripts/behavior_trees/rviz/maps/usd/config/env-hooks).
- `basic1.{png,yaml}` → `maps/`, `basic1.usda` → `usd/` 재배치. `basic1.yaml`의 `image: basic1.png`은
  동일 디렉토리 기준이라 무수정.
- 이전 패키지 `slam_navigation`은 **`slam_navigation_backups/2026-05-24_before-slam_nav-new-pkg.tar.gz`**
  (7.5MB, 로컬 보존)에 백업 — git에는 안 올림. 복원 절차는 같은 폴더의 `_NOTE.md` 참조.

### 2026-05-24 — Phase 0: basic1.usda namespace 분할

**현상** — basic1.usda의 두 iw_hub_ROS 모두 같은 payload (`Collected_iw_hub_ROS/iw_hub_ROS.usd`)
참조. 내부 OmniGraph의 `node_namespace`·`ros2_context`·`transform_tree_odometry`가 default 값이라
두 로봇이 같은 토픽 (`/cmd_vel`, `/tf`, `/scan` 등) 발행 → ROS 측 충돌.

**모색 옵션**:
1. payload 자체 편집 → 두 로봇이 서로 다른 payload 사용. **불가**: payload가 omniverse:// 원격이라
   편집 시 다른 사용 사례 깨짐 + 상대경로 참조 깨질 위험.
2. 두 prim 안에 OmniGraph 노드별 `over` 블록을 얹어 payload 안 default를 override. **채택**:
   기존 `slam_navigation/practice/sample5.usda`가 같은 패턴 사용해 검증됨.

**적용** — basic1.usda 3가지 변경 (총 +368 라인):

1. **`/World/ROS_Clock` OmniGraph 신규** (World prim 안, HighFriction Material 다음).
   - `on_playback_tick` + `ros2_context (domain_id=101)` + `isaac_read_simulation_time` + `ros2_publish_clock`.
   - 다른 OmniGraph가 cross-graph로 참조하던 sample5의 `_01` 단일 입력 형태는 제거 (단순화).

2. **`iw_hub_ROS_01` prim 안에 4종 over + world_pose_publisher** (lift_joint 다음 ~ prim 닫는 brace 직전):
   - `over "differential_drive"`: `node_namespace.inputs:value = "iw_hub_ROS_01"`, `ros2_context.domain_id = 101`
   - `over "ros_lidars"`: 동일 패턴
   - `over "transform_tree_odometry"`: 동일 + `ros2_publish_raw_transform_tree.parentFrameId = "iw_hub_ROS_01/odom"`
   - `over "front_hawk"`: `ros2_context.domain_id = 101` + `left/right_camera_namespace = "/iw_hub_ROS_01/front_stereo_camera/{left,right}"`
   - `def OmniGraph "world_pose_publisher"` 신규 — 자체 ros2_context 보유 (cross-graph 도메인 0 누수 회피).
     publish: `world → iw_hub_ROS_01/world_pose`.

3. **`iw_hub_ROS_02` prim에 동일 패턴** — NS만 `iw_hub_ROS_02`로 치환.

**적용 결과 / 성과**:
- USD 파일 라인 1559 → **3379** (+1820). brace 414/414 정합.
- node_namespace `inputs:value`: `iw_hub_ROS_01` 3개 + `iw_hub_ROS_02` 3개 (각 robot의 3개 ROS 그래프).
- `ros2_context.domain_id = 101`: 총 11곳 (ROS_Clock 1 + 두 robot 5개씩).
- world_pose_publisher OmniGraph: 2개 (각 로봇별).
- `colcon build --packages-select slam_nav` 성공.
- install 산출물에 `share/slam_nav/usd/basic1.usda` (140KB) 동기화.

**자주 발생하는 함정 (§6에도 등재)**:
- payload 안 default를 override할 때 `over "<원본_노드명>"` 정확한 이름이 핵심. payload 내부의
  실제 OmniGraph 노드명을 모르면 동작 안 함. 이번엔 sample5 동일 payload라 동일 노드명 가정.
- ROS_Clock 안의 `ros2_publish_clock`이 `isaac_read_simulation_time` 노드를 같은 그래프 내에서
  참조 — cross-graph 참조 금지 (런타임에 도메인 0 누수 버그 발생).
- 두 로봇의 world_pose_publisher가 *각자* ros2_context를 보유. 한 곳에서 공유 X.

**검증 (사용자 Isaac Sim 액션 필요)**:
1. Isaac Sim에서 `usd/basic1.usda` Stop → Reload → Play.
2. ROS_DOMAIN_ID=101 환경에서 `ros2 topic list | grep iw_hub_ROS_01` → cmd_vel, tf, scan 등 보여야 함.
3. `ros2 topic list | grep iw_hub_ROS_02` → 동일.
4. `ros2 topic echo /iw_hub_ROS_01/tf --once` → `world → iw_hub_ROS_01/world_pose` 메시지 보여야 함.

### 2026-05-24 — Phase 1: 첫 launch + Nav2 stack 작성

**목적** — basic1 절대맵 위에서 iw_hub_ROS_01이 NavigateToPose로 단독 자율주행 가능한 상태.

**작성한 4종 파일**:

1. **`launch/robot1_slam.launch.py`** — 14 entities. 구성:
   - `iw_hub_ROS_01_map_to_odom_static` — tf2_ros identity static TF (`map → odom`).
   - `world_to_iw_hub_ROS_01_map_bridge` — Isaac Sim spawn pose 캡쳐 → static TF `world → iw_hub_ROS_01/map`.
   - `iw_hub_ROS_01_warehouse_map_server` + `lifecycle_manager` — basic1.yaml을 `/iw_hub_ROS_01/warehouse_map` 토픽으로 publish (frame=world).
   - Nav2 stack 8개: controller_server / smoother_server / planner_server / behavior_server / bt_navigator / waypoint_follower / velocity_smoother / lifecycle_manager_navigation.
   - RViz (TimerAction 8초 지연 — map_server activate 대기).

2. **`params/robot1_nav.yaml`** — Nav2 전체 파라미터. 핵심:
   - `controller_server.FollowPath.plugin` = `nav2_rotation_shim_controller::RotationShimController` (primary: DWB)
   - `angular_dist_threshold: 0.349` (20° spin threshold), `vx_samples: 10, vtheta_samples: 30, PathAlign.scale: 64`
   - `global_costmap.global_frame: world`, `static_layer.map_topic: /iw_hub_ROS_01/warehouse_map`
   - `inflation_radius: 0.75, footprint_padding: 0.25, observation_persistence: 0.5`
   - `goal_checker_plugins: [stopped_goal_checker]` (복수형!), `yaw_goal_tolerance: 0.3`
   - amcl 섹션은 placeholder (Phase 2에서 활성화).

3. **`scripts/robot1_world_tf_bridge.py`** — `/iw_hub_ROS_01/tf` 구독해 `world → iw_hub_ROS_01/world_pose` 첫 메시지 캡쳐 → `world → iw_hub_ROS_01/map` static TF로 변환 후 `/iw_hub_ROS_01/tf_static`에 발행. 2초마다 재발행으로 늦은 subscriber 보장.

4. **`rviz/robot1_slam.rviz`** — Displays:
   - Grid (world frame), TF tree
   - Warehouse Map (`/iw_hub_ROS_01/warehouse_map`, TRANSIENT_LOCAL+RELIABLE)
   - Front/Back 2D LiDAR (BEST_EFFORT)
   - Global Plan (녹색 굵은 선) / Local Plan (주황)
   - Global Costmap (inflation 시각화)
   - Tools: SetInitialPose, 2D GoalTool
   - TopDownOrtho view, Fixed Frame=world.

**기존 시도와의 차이 (이전 슬램_navigation 대비)**:
- 맵: `warehouse_navigation.{yaml,png}` → `basic1.{yaml,png}` (이 패키지 사양)
- 패키지명·executable 모두 `slam_navigation` → `slam_nav`
- 처음부터 RotationShim 활성화 (이전엔 DWB-only로 시작 → 나중에 wrap)
- 처음부터 inflation 0.75 적용 (이전엔 0.5 → 0.75 → 1.1 → 0.75 다양한 시행착오)
- 보조 노드 4종(arrival_stopper/lidar_self_filter/yaw_align_watchdog/footprint_pub)은 일단 제외 — Phase 1 검증 후 단계적 추가

**결과 / 성과**:
- `colcon build --packages-select slam_nav` 성공.
- `ros2 pkg executables slam_nav` → `robot1_world_tf_bridge.py` 등록.
- Python launch description load 성공 (`14 entities`).
- install yaml 검증: RotationShim·angular_dist_threshold·inflation·map_topic·global_frame 모두 의도대로.

**다음 단계 (검증 절차, 사용자 액션 필요)**:
```bash
# 터미널 1: Isaac Sim 실행 후 basic1.usda 열고 Play
# 터미널 2: launch 실행
source /opt/ros/humble/setup.bash
source ~/smart_factory_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101
ros2 launch slam_nav robot1_slam.launch.py
```

### 2026-05-24 — Phase 1: 첫 launch 검증 (TF chain + Nav2 lifecycle)

**문제** — 첫 launch 실행 시 USD reload 미실시로 `/iw_hub_ROS_01/tf` publisher 0개 발견. 이후 USD `Duplicate prim 'differential_drive'` syntax 오류로 Isaac Sim에서 USD 열기 실패.

**원인** — USD namespace 분할 시 기존 `over "differential_drive"`(joint_name_array용)에 합치지 않고 별도 블록으로 만들어 같은 prim에 over가 2번 정의됨. 또한 `omni:scripting:scripts`의 상대경로(`@../../isaac_envs/...@`)가 패키지 이동 후 깨짐.

**해결**:
- 기존 `over "differential_drive"` 블록 안에 namespace + ros2_context over를 합침 (USD 규약).
- `omni:scripting:scripts` 절대 경로로 수정 (`@file:/home/rokey/.../iwhub_grab_kinematic.py@`).
- iw_hub_ROS_01·02 둘 다 동일 fix 적용.

**결과** — USD Reload·Play 후:
- `/iw_hub_ROS_01/tf` Publisher count: 0 → 4 ✓
- TF chain `world → iw_hub_ROS_01/map → odom → base_link → 9 sensors` 완성 ✓
- Nav2 lifecycle bt_navigator·global_costmap **active(3)** ✓
- LiDAR rate 1.5Hz (RTX render 제약, 정상)

### 2026-05-24 — Phase 2: 보조 노드 추가 (초록 화살표 + back 꼬리 필터 + arrival_stopper)

**기능 추가 3종**:

1. **`scripts/robot1_footprint_pub.py`** — chassis frame_locked ARROW marker.
   - 크기 = iw.hub 차체 (L 1.064 × W 0.728 × H 0.330m)
   - 색 초록 `(0.10, 0.85, 0.30)`, +X 방향 = 카메라 시선
   - `/iw_hub_ROS_01/footprint_marker` TRANSIENT_LOCAL latched

2. **`scripts/robot1_lidar_self_filter.py`** — back_2d_lidar 차체 꼬리 무시.
   - 실측 (-101° 부근 0.41~0.46m에서 6빔 검출 = 좌측 후방 차체 일부).
   - 필터 영역 `[-115°, -90°]` + 거리 ≤0.6m → `inf`로 변환.
   - 발행: `/iw_hub_ROS_01/back_2d_lidar/scan_filtered`.
   - nav.yaml의 back obstacle_layer가 `_filtered` topic 구독.

3. **`scripts/robot1_arrival_stopper.py`** — 도착 처리.
   - `navigate_to_pose/_action/status` 구독.
   - SUCCEEDED: cmd_vel Twist(0) 10회 burst + 1초 20Hz 안전망 + goal_marker DELETEALL.
   - ABORTED/CANCELED: cmd_vel 정지만 (marker 유지, 사용자 실패 지점 확인).

**RViz 갱신**:
- TF display 끔 (Show Names/Axes/Arrows = false) → 좌표축·텍스트 안 보임.
- Footprint Arrow MarkerArray display 추가.
- Goal Marker MarkerArray display 추가 (다음 절에서 random goal sender 발행).
- Back 2D LiDAR topic을 `_filtered`로 변경.

**검증** — launch 재기동 후:
- self-filter raw 6빔 → filtered **0빔** ✓ (차체 꼬리 완전 무시)
- footprint marker publisher 1 + RViz subscription 1 ✓
- Nav2 lifecycle active 유지 ✓

### 2026-05-24 — Phase 2: random goal sender + 일체화 검증 (절대↔상대 좌표)

**작성** — `scripts/robot1_send_random_goal.py`:
- basic1.png/yaml 로드 → free cell 209,859개 추출 → 무작위 1개 sampling.
- NavigateToPose goal 발행 + goal_marker (주황 SPHERE + heading ARROW + "Goal (cam X°)" TEXT) 발행.
- CLI: `ros2 run slam_nav robot1_send_random_goal.py [yaw_deg]`.

**일체화 검증**:
- 시작: `(-3.00, -7.49)` (basic1.usda spawn pose, world 절대 좌표)
- goal: `(-2.80, +1.75)` (basic1 free cell에서 random sampling)
- 추적 15초: (-2.62,-3.30) → (-2.57,-1.22) — y축 +4m 진행, 목적지까지 거리 5.06→2.98m
- cmd_vel 패턴 100샘플: **100% straight (vx≠0, wz≈0), arc 0%, spin 0%** (차체가 이미 +Y 방향 정렬되어 spin 불필요)

→ **basic1(절대맵) ↔ odom(상대) 좌표 일체화 통과**. 절대 좌표로 발행한 goal을 차체가 정확히 추적.

### 2026-05-24 — Phase 3: robot2 전체 복제 + 두 launch 동시 실행

**작성** — sed 기반 namespace 일괄 치환:
- `launch/robot2_slam.launch.py` (`iw_hub_ROS_01` → `iw_hub_ROS_02`, `robot1_` → `robot2_`)
- `params/robot2_nav.yaml`, `rviz/robot2_slam.rviz`
- `scripts/robot2_{world_tf_bridge,footprint_pub,lidar_self_filter,send_random_goal,arrival_stopper}.py`
- CMakeLists에 10개 스크립트 모두 등록.

**검증** — 두 launch 동시 실행 + Isaac Sim play:
- 토픽 분리: `/iw_hub_ROS_01/*` 47개 + `/iw_hub_ROS_02/*` 47개
- 두 Nav2 lifecycle **active(3)**
- 두 RViz 독립 가동 (`iw_hub_ROS_{01,02}_rviz2`)
- 두 robot 동시 random goal 발행 가능
- robot02 SUCCEEDED + arrival_stopper marker DELETEALL ✓

**발견** — robot01이 좁은 통로(시작점 부근)에서 stuck. xy 1.43m에 진입했으나 yaw 정렬 단계 진동. 사용자 결정: **좁은 통로 문제는 후순위, 천천히 통과는 됨**. yaw_align_watchdog 미구현 (이전 패키지 패턴).

### 2026-05-24 — Phase 4: 회피(양보) 코디네이터 A안 + 시연 검증

**작성** — `scripts/robot_yield_coordinator.py` + `launch/yield_coordinator.launch.py`:
- 두 robot의 TF (`world → map` static + `odom → base_link` live) 합성 → world 거리 계산.
- 25Hz tick:
  - dist < **2.0m** → robot_id 큰 쪽(02)에 `Twist(0)` jam 발행 (velocity_smoother 20Hz보다 우세하여 정지).
  - dist > **3.0m** → 해제 (hysteresis로 임계값 근처 진동 방지).

**모색했던 4가지 옵션 (A/B/C/D)**:
- **A안 (단순 정지)** ← 채택. 이전 슬램_navigation 패키지에서 검증된 패턴, 즉시 작동.
- B안 (옆으로 비키기): NavigateToPose cancel + 임시 좌표 → 통과 후 원경로 재발행. 복잡, 좁은 통로엔 더 적합.
- C안 (costmap inflation 기반): Nav2가 자체 우회. deadlock 가능성 그대로.
- D안 (A안 + cancel + 메모리 goal): A안의 progress_checker stuck 위험 해결 + 시각 표시. **추천 다음 업그레이드**.

**검증 시나리오 (swap goal)**:
- 시작: robot01 `(-3.00, -7.50)`, robot02 `(+3.00, -7.50)`, 거리 6.00m. 같은 y선 정면 대치.
- swap goal: robot01 → 02 자리, robot02 → 01 자리. 마주 진행.
- 시계열:
  ```
  t= 5s  d=5.73m   (둘 다 이동)
  t=10s  d=4.71m
  t=15s  d=3.56m
  t=20s  d=2.83m
  t=24.3s ⚠️ 양보 발동 — dist=1.97m, iw_hub_ROS_02 정지 ✓
  t=30s  robot02 (+1.15,-6.55) 정지 / robot01 (+0.15,-6.86) 진행, d=0.90m
  t=40s  robot02 정지 유지 / robot01 (+0.67,-7.20), d=0.81m  ← 최근접
  t=45s  robot01 (+1.14,-7.56) — robot02 옆 통과 중
  t=50s  robot01 (+1.77,-7.76) — 통과 완료, d=1.36m
  t=60s  robot01 (+2.26,-7.77) — goal 1m 남음, robot02 여전히 정지
  ```

**결과 / 성과**:
- 양보 발동 임계값 정확히 작동 (dist=1.97m < 2.0m).
- robot02 35초 정지 유지 + robot01이 Nav2 LiDAR 우회로 옆 통과 (최근접 0.81m, **충돌 0건**).
- A안 단순 정지 + Nav2 native 우회의 시너지 — 별도 비키기 로직 없이도 통과 가능.

**미검증**:
- robot01이 goal 도달 후 dist > 3.0m 되면 양보 해제 + robot02 작업 재개 → 60초 안에 도달 못 함, 추가 모니터링 필요 (§4-1).

### 2026-05-24 — Phase 4+ 회피 검증 사이클 충돌 사고 + kill-before-relaunch 규칙

**사고** — 양보 시연 중 Isaac Sim Stop→Play로 reset했으나 RViz와 백그라운드 launch는 reset 안 함. 잔존 cmd_vel jam이 새 reset 상태에 영향 → 두 robot 충돌.

**원인**:
- ROS 노드(launch, coordinator)는 USD reset에 영향 없음 — 살아 있음.
- coordinator가 reset 전 양보 상태를 유지(robot02 jam) → reset 후 robot02 새 위치가 jam 받아 멈춤.
- 사용자가 RViz도 reset 안 해 시각상 어긋난 상태에서 새 시도.

**해결** — 사용자 지시: **재실행 시 모든 백그라운드 프로세스 kill 후 진행**.
- `pkill -INT -f "ros2 launch slam_nav"`, `pkill -INT -f "robot_yield_coordinator"` 등으로 정리.
- 정리 후 `pgrep -af "ros2 launch|robot_yield|slam_nav"` 0건 확인.
- 그 다음 사용자 Isaac Sim reset 신호 받은 후 launch 새로 시작.

**규칙 저장** — `~/.claude/projects/...memory/feedback_kill_before_relaunch.md` 에 기록 (다음 세션 자동 적용).

**재검증 결과** — kill 후 깨끗하게 다시 시작:
- 두 launch + coordinator 새 시작 → 두 Nav2 active.
- swap goal 재발행 → 양보 사이클 정상 동작 (위 절 결과 재현).

### 2026-05-26 — Phase 5: chain (A→J) + 후진 정밀도 최적화

**목표**: 사용자 정의 dolly 픽업/drop 시나리오 (10개 waypoint A→J, 후진 2개 구간) 자동 chain.

**구현** — `chain_goal_robot1.py` (이후 `chain_goal.py`로 일반화):
- WAYPOINTS dict `(name, x, y, yaw, reverse)`. reverse=True면 후진 모드.
- `send()`: NavigateToPose 발행 → NEAR-GOAL(0.5m, 8s hold) cancel → align_yaw (cmd_vel angular P-controller, 4°→1° tolerance).
- `drive_backward()`: cmd_vel 직접 발행 (action 없음). hold_yaw 유지 + 거리 추적 + REVERSE_DIST_TOL(0.05m) 도달 시 정지.

**문제 1 — cmd_vel jam (forward/reverse 모두 평균 속도 1/7)**:
- 증상: REVERSE timeout (3m을 60초에 1.1m만 후진). forward도 A→B 4.5m가 132s.
- 원인: chain이 `/cmd_vel`에 직접 publish + velocity_smoother가 같은 `/cmd_vel`에 publish (input timeout 1s zero 발행) → alternating.
- 측정: cmd_vel echo로 `z=0`과 `z=0.16` 교대 발행 확인. publisher count 7개 (chain + smoother + behavior_server×4 + arrival_stopper).
- 해결: chain의 cmd_vel publish target을 `cmd_vel` → `cmd_vel_nav` (velocity_smoother input topic)로 변경. smoother가 정상 처리.

**문제 2 — REVERSE_SPEED 0.15도 실제 0.02 m/s (~13%)**:
- 원인 가설: Isaac differential drive plugin이 max wheel velocity cap. forward Nav2 명령도 동일하게 느림.
- 해결: REVERSE_SPEED 0.15 → 1.0 (10배 증폭). 실속도 ~0.10 m/s. velocity_smoother min=-2.5라 cap 여유 있음.

**문제 3 — C 도착 x 오차 0.4m (target 0,-15.25 vs 실제 -0.40,-15.19)**:
- 원인: B yaw align 후 yaw 93.8° (target 90°, 오차 -3.83°) → 2.62m 후진 중 -X drift.
- 해결 단계:
  1. YAW_TOL_RAD 4°→1° (시작 yaw 정밀도 ↑)
  2. REVERSE_YAW_KP 1.0→3.0, MAX_OMEGA 0.45→1.0 (drift 적극 보정) — 효과 미미 (0.30m로만)
  3. **Cross-track 보정 추가** — target line에서 lateral 편차 측정 → yaw target 동적 조정 (`yaw_target = hold_yaw - K_cross * lateral`, K_cross=1.5, max ±0.3 rad)
- 결과: C 도착 **(0.02, -15.20)** — x 오차 0.02m, y 0.05m. 후진 내내 lateral ±0.022m 안정.

**문제 4 — dolly 바퀴 obstacle 인식**:
- 사용자 보고: 후진 중 dolly 바퀴가 Nav2 obstacle layer에 marking됨.
- 해결: `lidar_self_filter`의 `pickup_approach` mode 활용 — ±60°·2m 영역 inf 변환. 이후 ±120°·4m로 확대 (반원형 dead zone).
- chain에 자동 toggle 추가: B/F 진입 직전 `set_pickup_approach(True)`, 후진 완료 후 False (`drive_backward()` try/finally).

**회전 속도 조정** (2026-05-26 사용자 피드백 사이클):
- 1차: 2배 (5.0/3.0/2.4) — "너무 빠름"
- 2차: 1.5배 (3.75/2.25/1.8) — "약간 느림"
- 최종: 1.75배에서 -25% = **약 1.3배** (3.3/1.95/1.6). velocity_smoother angular cap도 동기화.

**부분 chain 실행** — main()에 argparse `--start --end` 추가. `python3 chain_goal.py --robot iw_hub_ROS_01 --mode plus --start B --end D`로 B→C→D만 검증 가능.

**결과 / 성과**:
- 4 reverse 구간 (01_plus C/G, 01_minus D/H 또는 02_plus C/G, 02_minus D/H) 모두 x±0.05m, y±0.09m 안에 도착.
- 사용자 허용 (x±0.08, y±0.10) 만족 ✓.
- chain 전체 A→J 약 9분 (forward 7 + reverse 2 + yaw align 9회).

### 2026-05-27 (저녁) — chain reverse_code 0/1/2/3 + 자동 lift post-action

**목표** — chain 시퀀스에서 dolly 픽업/drop을 자동화. 사용자 ros2 topic pub 한 줄로 5초 ramp + 후속 navigation 진행.

**구현**:
- `chain_goal.py` ROUTES `reverse: bool` → `reverse_code: int (0/1/2/3)` 인코딩 확장
  - 0=forward, 1=reverse only, 2=reverse+lift_up, 3=reverse+lift_down
  - 패턴: plus의 C(2)/G(3), minus의 D(2)/H(3) — 4개 route 일관
- `ChainGoalSender._lift_post_action(name, action)`: `/<robot>/lift_target` (Float64) publish + 5초 대기 (4초 lift_ramper ramp + 1초 여유)
  - `LIFT_POST_WAIT_SEC = 5.0`, `LIFT_UP_TARGET = 0.04`, `LIFT_DOWN_TARGET = 0.0`
- `execute_sequence(waypoints, task_id="")` — reverse_code 처리 + 완료 시 `/<robot>/chain_done` (String) 으로 `task_done:<id>` publish (task_id 비어있지 않을 때만)
- `chain_waypoint_server.py` — `position.z=int(round(z))` 파싱, `header.frame_id="task_id:<n>"` 인식 → 완료 시 chain_done 발행. 0~3 외 값은 0(forward)로 fallback.
- 구버전 호환: ROUTES의 bool reverse도 그대로 동작 (True→1, False→0 자동 변환).

**검증 (Isaac Sim + Nav2 launch)**:
- robot01 plus B→D: C(rev=2) 도착 → `[C] lift_up → lift_target=0.04, 5초 대기` 출력 + 5초 후 D 자동 진행 ✓
- robot01 plus E→J: G(rev=3) 도착 → `[G] lift_down → lift_target=0.00, 5초 대기` 출력 + 5초 후 H 자동 진행 ✓
- C arrival err: (+0.022, +0.046, +1.77°) — 사용자 허용 ±0.05/±0.10 안 ✓
- 전체 시퀀스 정상 완주 (A~J)

**다음 (PC-D 측 후속)**:
- Supabase 스키마 (`dolly_tasks`, `task_positions`)
- Dispatcher 노드 — pending 작업 polling + TEMPLATE+DB 합성 후 PoseArray publish
- chain_done 구독 → DB status='done' + task_positions delete

### 2026-05-27 — 코드 통합/최적화 (helper + params + RViz 단일화)

**목표** — robot1_*/robot2_*로 중복된 파일을 namespace 인자 받는 단일 파일로 통합. 두 robot 운영 시 유지보수 부담 절반.

**백업** — `slam_nav_backups/2026-05-27_before-helper-consolidation.tar.gz`.

**통합 결과**:

| 구분 | 변경 전 | 변경 후 |
|---|---|---|
| helper scripts | `robot{1,2}_world_tf_bridge.py`, `robot{1,2}_footprint_pub.py`, `robot{1,2}_lidar_self_filter.py`, `robot{1,2}_arrival_stopper.py` (8개) | `world_tf_bridge.py`, `footprint_pub.py`, `lidar_self_filter.py`, `arrival_stopper.py` (4개) — `argparse --robot iw_hub_ROS_0N` |
| launch | `robot1_slam.launch.py`, `robot2_slam.launch.py`, `multi_robot_slam.launch.py` (3개) | `multi_robot_slam.launch.py` (1개) — `make_robot_group(robot_num)` factory |
| params | `robot1_nav.yaml`, `robot2_nav.yaml` (2개) | `nav.yaml` 단일 템플릿 — `<ROBOT>` placeholder를 launch에서 `/tmp/slam_nav_<bare>_nav.yaml`로 치환 |
| rviz | `robot1_slam.rviz`, `robot2_slam.rviz` (2개) | `slam.rviz` 단일 템플릿 — 동일 치환 방식 |

**부수 효과 — robot2 params 누락 튜닝 동기화**:
- 통합 전 robot2_nav.yaml에는 2026-05-26 회전속도/yaw tolerance 튜닝(max_velocity angular 3.3, yaw_goal_tolerance 0.5, rotate_to_heading_angular_vel 1.95, max_vel_theta 3.3, acc_lim_theta 7.9)이 미반영. 통합 yaml에 robot1 값을 master로 적용 → 두 robot 모두 동일 회전 특성.

**구현 세부**:
- `multi_robot_slam.launch.py`의 `_materialize(template, bare, suffix)` 함수가 `<ROBOT>` → bare name 치환 후 `/tmp/slam_nav_<bare>_*.yaml`/`.rviz`에 저장. Nav2 노드는 이 파일을 parameters에 직접 사용.
- helper 스크립트 main은 `argparse.parse_known_args(sys.argv[1:])` — `--ros-args` 등 부가 인자 무시.
- CMakeLists install 규칙: 통합 scripts 4종 + 기존 (chain_goal, chain_waypoint_server, robot_yield_coordinator) 7개 PROGRAMS.

**제거된 legacy 파일** (8 scripts + 2 launch + 2 params + 2 rviz = 14개):
- scripts: `robot{1,2}_{world_tf_bridge,footprint_pub,lidar_self_filter,arrival_stopper}.py`
- launch: `robot{1,2}_slam.launch.py`
- params: `robot{1,2}_nav.yaml`
- rviz: `robot{1,2}_slam.rviz`

**검증 (자동 부분)**:
- `colcon build --packages-select slam_nav` 성공
- `ros2 pkg executables slam_nav` → 7개 (통합 4종 + chain 2종 + yield 1종)
- launch description load: 33 entities (2 robot × ~16 + 공통 2)
- 치환 결과 확인: `/tmp/slam_nav_iw_hub_ROS_0{1,2}_{nav.yaml,slam.rviz}` 4개 생성, `<ROBOT>` placeholder 0개 잔존, iw_hub_ROS_01 14곳·02 13곳 치환 완료

**다음 (사용자 액션 필요)** — Isaac Sim 실행 후 회귀 검증:
```bash
# 모든 백그라운드 프로세스 정리 (kill-before-relaunch 규칙)
pkill -INT -f "ros2 launch slam_nav" ; pkill -INT -f "chain_goal" ; sleep 1
# Isaac Sim에서 usd/basic1.usda Stop→Reload→Play
# 그 다음
source /opt/ros/humble/setup.bash
source ~/smart_factory_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101
ros2 launch slam_nav multi_robot_slam.launch.py
# 두 RViz 뜨면 → 두 robot Nav2 active(3) 확인 → chain test:
python3 ~/smart_factory_project/ros2_ws/src/slam_nav/scripts/chain_goal.py --robot iw_hub_ROS_01 --mode plus --start B --end D
python3 ~/smart_factory_project/ros2_ws/src/slam_nav/scripts/chain_goal.py --robot iw_hub_ROS_02 --mode plus --start B --end D
```

### 2026-05-26 — Phase 6: 4개 경로 정의 + multi-robot 통합 launch

**경로 정의** — 사용자 입력으로 4개 chain 확정:

| | plus | minus |
|---|---|---|
| **iw_hub_ROS_01** | 픽업(0,-15.25) → drop(-4.125,0) | 픽업(-4.125,0) → drop(0,-15.25) |
| **iw_hub_ROS_02** | 픽업(0,28.75) → drop(4.125,12) | 픽업(4.125,12) → drop(0,28.75) |

- A/E/I/J (plus) 또는 A/B/F/J (minus): 고정 좌표 (대기/도로 인프라)
- 그 외: DB에서 받는 좌표 (작업장 변동)
- 02 작업영역은 +Y쪽 (25~28.75)로 robot1과 분리 (초기 좌표 충돌 해결).
- 02_minus yaw 수정 사이클: 사용자 입력 오타 → 최종 B(-90)/C(180)/D(180,R)/E(90)/F(90) — C/D yaw 180 일치 → C→D 직선 후진 가능.

**chain_goal.py 일반화** — 기존 `chain_goal_robot1.py` 대체:
- `ChainGoalSender(robot_ns)` 클래스 (인스턴스 인자로 robot 선택)
- ROUTES dict `(robot, mode) → waypoints`
- argparse `--robot --mode --start --end`
- `execute_sequence(waypoints)` — 외부에서 임의 sequence 주입 가능

**chain_waypoint_server.py 신규** — DB topic 인터페이스:
- subscribe `/iw_hub_ROS_0N/chain_waypoints` (geometry_msgs/PoseArray)
- 인코딩 (2026-05-27 확장): position.x/y=좌표, orientation=yaw quaternion, **position.z = 0/1/2/3** (0=forward, 1=reverse, 2=reverse+lift_up, 3=reverse+lift_down)
- 선택: `header.frame_id="task_id:<n>"` 으로 작업 ID 전달 시 sequence 완료/실패 시 `/<robot>/chain_done` (String) 으로 `task_done:<n>` 또는 `task_failed:<n>` 발행 — PC-D dispatcher 가 구독해 DB 상태 갱신
- 받은 sequence를 `execute_sequence()`로 실행 (큐 1개, 실행 중 새 메시지 무시)
- robot01/02 각자 server instance — PC-D가 각 topic에 publish하면 독립 동시 처리

**multi_robot_slam.launch.py 신규** — robot1/robot2 통합:
- 기존 `robot1_slam.launch.py` + `robot2_slam.launch.py` 둘 다 띄울 필요 없이 단일 launch
- helper (world_tf_bridge, footprint_pub, lidar_self_filter, arrival_stopper, map_server) × 2 robot
- Nav2 stack × 2 robot (각자 namespace `iw_hub_ROS_0N`)
- chain_waypoint_server × 2 (각 robot 토픽)
- RViz × 2 (옵션 `rviz:=false`로 비활성 가능)
- TimerAction 8s 후 chain_server + RViz spawn (Nav2 active 대기)

**clean up**:
- `robot1_send_random_goal.py`, `robot2_send_random_goal.py` 삭제 (테스트용, launch 미사용)
- CMakeLists.txt에 chain_goal.py, chain_waypoint_server.py 등록

**검증 — 두 robot 동시 plus → minus**:
- robot1: plus → minus 순차 (총 ~20분, A~J × 2 사이클)
- robot2: plus → minus 순차 (병렬)
- 모두 정상 완주. 작업 영역 분리로 충돌 없음.
- ARRIVAL log 추가 — `target / actual / err` 한 줄. minus 사이클은 풀 데이터 수집 (plus는 코드 수정 전 실행이라 미수집).

**비교 표 (minus 도착 정확도)**:
- 후진 (C/D/G/H 4개): x±0.05m, y±0.09m — **허용 안에 들어옴** ✓
- forward (NavigateToPose, NEAR-GOAL cancel): x/y 오차 0.2~1.0m — NEAR-GOAL HOLD 8s 이후 cancel 특성
- yaw 도착: 대부분 ±1° (1° tolerance), 후진은 cross-track 보정 부수효과로 ±3° 가능

---

## 6. 자주 발생하는 문제 + 해결 카탈로그

> 이전 `slam_navigation` 패키지에서 이미 겪고 해결한 문제들. 새 패키지에서 같은 증상 보이면
> 여기 먼저 확인. 해결책 그대로 적용 가능.

### 6-1. ROS_DOMAIN_ID mismatch — Isaac Sim 토픽 안 보임

- **증상**: `ros2 topic list`에 `/iw_hub_ROS_NN/*` 안 보임. launch 띄워도 SLAM/Nav2가 LiDAR 못 받음.
- **원인**: USD OmniGraph의 `ros2_context.domain_id`와 launch 터미널의 `ROS_DOMAIN_ID` 불일치.
- **해결**: 양쪽 모두 **101**로 통일. launch 실행 전 `export ROS_DOMAIN_ID=101`. USD 내부
  모든 `ros2_context`의 `domain_id=101, useDomainIDEnvVar=0`.

### 6-2. `world` 프레임 누락 → warehouse map RViz에 안 뜸

- **증상**: RViz "no map received" 또는 map만 안 뜨고 로봇·LiDAR는 정상.
- **원인**: world_pose_publisher OmniGraph의 ROS2 publish 노드가 **다른 graph(`/World/ROS_Clock`)의
  ros2_context를 cross-graph 참조** → 런타임에 깨져 도메인 0으로 발행 → 도메인 101 브리지가 못 받음.
- **해결**: `world_pose_publisher` graph에 **자체 `ROS2Context` 노드(domain_id=101)** 추가하고
  publish 노드가 그것을 같은-graph로 참조. cross-graph 의존 제거.

### 6-3. SLAM map이 warehouse map과 어긋남 ('귀신 obstacle')

- **증상**: RViz에 SLAM 맵의 occupancy가 warehouse 사각형 *바깥*까지 회전·이동되어 그려짐.
  차체가 "아무것도 없는데 있다"고 판단해 우회 시도.
- **원인**: wheel odom yaw drift 누적 → base_link world pose 어긋남 → LiDAR scan world 변환 시
  회전된 위치에 obstacle marking + slam_toolbox mapping이 어긋난 base_link 기준 누적.
- **해결 (이전 적용)**:
  1. **slam_toolbox 비활성** + identity static `map→odom` TF — SLAM map publish 자체 차단.
  2. `observation_persistence: 0.5초` — 잔상 단축.
  3. **AMCL** 추가 (warehouse 기준 절대 localization) — 본질적 해결, 이 패키지에서 처음부터 적용 권장.

### 6-4. 차체가 도착 후 yaw 정렬 실패 (좌우 무한 진동)

- **증상**: xy 도달했으나 카메라 방향(yaw) 못 맞춰 SUCCEEDED 안 나옴. 차체 좌우 진동.
- **원인 (복합)**:
  1. **LiDAR self-detection** — 차체 일부가 LiDAR에 obstacle로 보임 → 회피 시도 + yaw 정렬 충돌.
  2. **도착지 근처 외부 장애물** — yaw 회전 시도 시 obstacle 회피 동작과 충돌.
- **해결**:
  1. `scripts/lidar_self_filter.py` — 차체 dead zone (`[-140°,-70°]` ≤1.3m) 빔을 `inf`로 변환.
  2. `scripts/yaw_align_watchdog.py` — xy<1.5m + 8초 timeout이면 cancel.
  3. `scripts/arrival_stopper.py` — CANCELED + dist<1.5m면 marker 노랑 ✓ "Arrived xy"로 인정.

### 6-5. Nav2 yaml 키 미스매치 (조용히 default로 동작)

- **증상**: yaml에서 바꾼 파라미터가 적용 안 됨. `ros2 param get`으로 확인하면 default 값.
- **원인**:
  1. **단수/복수 오타**: `goal_checker_plugin` (X) ↔ `goal_checker_plugins` (O, 배열).
  2. **노드명/yaml 첫 키 불일치**: launch에서 노드 이름에 prefix 주면 yaml 첫 키도 같이 변경 필수
     (예: launch `name="iw_hub_ROS_01_slam_toolbox"` → yaml 첫 키 `iw_hub_ROS_01_slam_toolbox:`).
- **진단**: `ros2 param list <node> | grep <plugin>`로 등록 여부 확인. 등록 안 됐으면 키 mismatch.

### 6-6. 회전 시 arc 그림 (직선 spin 안 됨)

- **증상**: DWB가 sampling-based라 v=양수+omega=양수 trajectory 선택 → 호 그리며 회전.
- **해결**: `nav2_rotation_shim_controller::RotationShimController`로 DWB wrap.
  `angular_dist_threshold: 0.349 (20°)` 이상 차이면 spin → DWB. 결과: cmd_vel 96% spin_only.

### 6-7. 두 로봇 마주 접근 시 충돌

- **증상**: 양쪽 모두 우회 시도해 deadlock 또는 충돌.
- **해결**: `scripts/yield_coordinator.py` — 두 로봇 TF로 거리 계산, `<2m`면 robot_id 큰 쪽 정지
  (cmd_vel 25Hz jam), `>3m`이면 해제 (hysteresis).

### 6-8. cmd_vel watchdog 부재 → 도착 후에도 잔존 속도

- **증상**: Nav2가 도착 후 cmd_vel 멈춰도 차체가 계속 움직임.
- **원인**: USD `differential_drive`에 cmd_vel watchdog 없음 → 마지막 명령 무한 적용.
- **해결**: `scripts/arrival_stopper.py` — SUCCEEDED/CANCELED/ABORTED 모두 처리, Twist(0) 10회
  burst + 1초간 20Hz 안전망.

### 6-9-pre. USD payload OmniGraph override 패턴

- **증상**: 같은 payload(`iw_hub_ROS.usd`) 참조하는 두 prim이 같은 토픽 발행 → ROS 충돌.
- **해결 패턴**:
  ```usda
  def "iw_hub_ROS_01" (
      prepend payload = @.../iw_hub_ROS.usd@
  ) {
      over "differential_drive" {
          over "node_namespace" { custom string inputs:value = "iw_hub_ROS_01" }
          over "ros2_context"   { custom uchar inputs:domain_id = 101
                                  custom bool  inputs:useDomainIDEnvVar = 0 }
      }
      over "ros_lidars" { ... 동일 ... }
      over "transform_tree_odometry" {
          over "node_namespace" { ... }
          over "ros2_context"   { ... }
          over "ros2_publish_raw_transform_tree" {
              custom string inputs:parentFrameId = "iw_hub_ROS_01/odom"
          }
      }
      over "front_hawk" {
          over "ros2_context" { ... }
          over "left_camera_namespace"  { custom string inputs:value = "/iw_hub_ROS_01/front_stereo_camera/left" }
          over "right_camera_namespace" { custom string inputs:value = "/iw_hub_ROS_01/front_stereo_camera/right" }
      }
      def OmniGraph "world_pose_publisher" { ... 자체 ros2_context 보유 ... }
  }
  ```
- **함정**: 자체 ros2_context를 안 두고 다른 그래프의 context를 cross-graph로 참조하면 런타임에 도메인 0으로 누수 (이전 슬램_navigation §6 "문제 4").
- **검증**: Isaac Sim Stop→Reload→Play 후 `ros2 topic list | grep iw_hub_ROS_NN` 으로 분리 확인.

### 6-9. 차체 외접 반경보다 좁은 inflation

- **증상**: 좁은 틈으로 path 그려져 통과 실패 → Spin 복구 반복.
- **해결**: `inflation_radius` ≥ 차체 외접 반경(iw.hub: 0.645m). 안전 값 0.75m.
  너무 크면(1.1m) 좁은 통로 못 지나가므로 현장 맞춰 조정.

---

## 7. 진단 명령 모음 (빠른 체크)

```bash
# 필수 환경
source /opt/ros/humble/setup.bash
source ~/smart_factory_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101

# 토픽 전체
ros2 topic list | sort

# Isaac Sim publish 여부 (둘 다 보여야 함)
ros2 topic list | grep -E "iw_hub_ROS_(01|02)"

# 두 로봇 토픽 분리 확인 (Phase 0 완료 후)
ros2 topic list | grep iw_hub_ROS_01 | wc -l  # ≥ 5 ~ 10
ros2 topic list | grep iw_hub_ROS_02 | wc -l  # 같은 수

# TF chain (parent->child 페어)
timeout 4 ros2 topic echo /iw_hub_ROS_01/tf 2>&1 > /tmp/tf.out
# parent + child_frame_id 추출 코드는 이전 slam_navigation/practice/README.md §9 참조

# tf_static 펼치기 (한 topic의 모든 메시지)
timeout 5 ros2 topic echo /iw_hub_ROS_01/tf_static

# Nav2 lifecycle 활성 확인
ros2 service call /iw_hub_ROS_01/bt_navigator/get_state lifecycle_msgs/srv/GetState

# 센서 메시지 frame_id
ros2 topic echo /iw_hub_ROS_01/front_2d_lidar/scan --once --field header
```

---

## 8. 결정 사항 / 알려진 제약

- **절대맵 master + 상대맵 누적**: navigation 권위는 basic1.png(world frame). LiDAR SLAM은
  새 장애물 발견용 보조 (mapping은 obstacle_layer가 raw scan으로 처리).
- **직선 + spin 경로 강제**: RotationShim threshold 0.349(20°). arc 거의 사라짐.
- **양보 우선순위**: robot_id 큰 쪽이 양보 (단순 규칙. idle/긴급도는 향후 확장).
- **단일 PC 임시 운영**: 4PC 통신 분리 전까지 A·B·C·D 역할 모두 이 PC에서. domain 101 통일.
- **USD canonical**: `usd/basic1.usda`. 이 패키지 작업은 항상 이 파일을 수정.

---

## 9. 참고 자료 (이전 패키지)

- **이전 패키지 README**: `~/smart_factory_project/ros2_ws/src/slam_navigation/practice/README.md`
  (1404+ 줄, 작업 과정 + 트러블슈팅 누적 기록).
- **이전 USD canonical**: `~/smart_factory_project/ros2_ws/src/slam_navigation/practice/sample5.usda`
  (16MB+, namespace 분할 + iw.hub 통합본 — 새 USD 분할 시 패턴 참조).
- **백업**: `~/smart_factory_project/ros2_ws/slam_navigation_backups/2026-05-24_before-slam_nav-new-pkg.tar.gz`
  (이전 패키지 마지막 상태).
