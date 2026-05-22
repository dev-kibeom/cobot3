# M0609 연동 메모

이 `vision` ROS2 패키지는 런타임 노드, Isaac Sim 도구, 학습/평가 도구를 함께 담당합니다.
기본 실행은 `ros2 run vision yolo`, `ros2 run vision brain`, 또는 `ros2 launch vision vision_pipeline.launch.py`입니다.

- `yolo11s_obb_eye_node.py`: 이미지 토픽 -> YOLO11s-OBB -> OBB 결과 payload
- `alignment_brain.py`: OBB 결과 + 카메라 모델 + 목표 프레임 -> pick/place goal 계산
- `alignment_brain_node.py`: OBB 토픽 -> pick/place goal 토픽
- `goal_gateway.py`: 비전 도메인의 pick/place goal만 모션 도메인으로 전달
- `vision_contracts.py`: 노드 사이에서 공유하는 payload 형식
- `tools/isaac/`: Isaac Sim에서 직접 돌려보는 smoke/evidence/dataset 도구
- `tools/training/`: YOLO 평가, 데이터셋 변환, 파인튜닝 도구
- `work/`: 생성 이미지, 학습 데이터, 테스트 결과물 저장 위치

`m0609_vision/`은 로봇 실행부 구현입니다. 이 폴더에서는 참고용으로만 읽습니다.

## M0609와 연결되는 방식

기존 M0609 pick-place controller는 이미 아래 형태의 입력을 받습니다.

```python
actions = controller.forward(
    picking_position=cube_position,
    placing_position=goal_position,
    current_joint_positions=current_joints,
    end_effector_offset=np.array([0.0, 0.0, 0.2]),
)
robot.apply_action(actions)
```

비전 brain은 같은 형태로 넘길 수 있는 값을 만들어야 합니다.

```python
goal = make_pick_place_goal(cell_name, obb, target_frame, camera_model)

picking_position = np.array([goal.pick.x, goal.pick.y, goal.pick.z])
placing_position = np.array([goal.place.x, goal.place.y, goal.place.z])
```

`yaw` 값은 `goal.pick.yaw`, `goal.place.yaw`에 계산되어 들어갑니다.
다만 현재 M0609 `PickPlaceController.forward()` 흐름은 우선 위치 중심입니다.
따라서 end-effector orientation 반영은 position 기반 pick-place가 안정화된 뒤 로봇 쪽에서 추가하는 편이 좋습니다.

## Vision PC 실행 흐름

우리 비전 컴퓨터는 `ROS_DOMAIN_ID=104`에서 YOLO와 brain을 실행합니다.

```bash
export ROS_DOMAIN_ID=104
```

YOLO 노드는 이미지 토픽을 받아 OBB payload를 발행합니다.

```bash
ros2 run vision yolo \
  --cells global \
  --image-topic-template "/{cell}/top_camera/image" \
  --obb-topic-template "/{cell}/vision/plate_obb" \
  --imgsz 640 \
  --max-hz 2 \
  --conf 0.35
```

brain 노드는 OBB payload를 calibrated pick/place goal로 바꿉니다.

```bash
ros2 run vision brain \
  --cell global \
  --robot-index 1 \
  --obb-topic /global/vision/plate_obb \
  --goal-topic /global/vision/pick_place_goal
```

디버깅할 때만 YOLO 노드에 `--publish-debug`를 붙이고 `/global/vision/yolo_debug_image`를 봅니다.
실전에서는 서버와 모션 컴퓨터로 debug image를 넘기지 않습니다.

## Top-View YOLO와 Wrist Visual Servo의 역할 차이

참고한 `m0609_vision/m0609_visual_tracking_spec.md`는 eye-in-hand 방식의 visual servo 설계입니다.

- wrist RealSense mesh/sensor를 `angle_bracket`에 부착
- HSV tracker가 wrist camera 화면 중심 근처의 target을 찾음
- visual servo controller가 물체가 화면 중앙에 오도록 end-effector XY를 보정
- 이후 기존 pick-place sequence 실행

현재 `vision` 패키지 설계는 top-view 기반 정렬 방식입니다.

- 고정 top camera가 작업 셀 전체를 봄
- ArUco가 목표 frame을 정의
- YOLO11s-OBB가 철판 OBB를 검출
- `alignment_brain.py`가 pick/place pose를 계산
- M0609가 기존 pick-place 흐름으로 실행

추천 MVP 순서는 다음과 같습니다.

1. top-view vision으로 coarse pick/place target을 먼저 생성합니다.
2. wrist visual servo는 나중에 pick point 근처 정밀 보정용으로 남겨둡니다.
3. YOLO, visual servo, robot control을 한 파일에 모두 넣지 않습니다.

## 6개 로봇 셀 구조

각 작업 셀마다 namespace를 하나씩 둡니다.

```text
/cell_01/top_camera/image      -> /cell_01/vision/plate_obb
/cell_02/top_camera/image      -> /cell_02/vision/plate_obb
...
/cell_06/top_camera/image      -> /cell_06/vision/plate_obb
```

`yolo` alias는 6개 셀을 한 번에 처리할 수 있습니다.

```bash
ros2 run vision yolo \
  --model vision/models/yolo11s_obb_metal_hard-v2_refinetune_best.pt \
  --cells cell_01,cell_02,cell_03,cell_04,cell_05,cell_06
```

성능을 생각하면 처음부터 6개 카메라를 모두 켜지 않는 편이 좋습니다.
먼저 1개 셀만 활성화해서 성공시키고, 이후 셀을 하나씩 추가하세요.

## Goal Gateway

모션 담당 컴퓨터가 `ROS_DOMAIN_ID=102`를 사용한다면, 이미지나 debug 토픽을 넘기지 말고 최종 goal만 넘깁니다.

```text
Vision domain
  /global/vision/pick_place_goal
        |
        v
goal_gateway.py
        |
        v
Motion domain 102
  /m0609_vision/pick_place_goal
```

실행 예시는 다음과 같습니다.

```bash
ros2 run vision goal_gateway \
  --source-domain 104 \
  --target-domain 102 \
  --input-topic /global/vision/pick_place_goal \
  --output-topic /m0609_vision/pick_place_goal
```

gateway는 `Float32MultiArray` goal payload 15개 값만 전달합니다.
invalid goal, 낮은 confidence, 중복 goal, 너무 빠른 반복 발행은 기본적으로 줄여서 서버와 네트워크 부담을 낮춥니다.
