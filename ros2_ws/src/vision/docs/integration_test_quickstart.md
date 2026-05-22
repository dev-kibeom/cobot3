# M0609 D455 통합테스트 빠른 안내서

이 문서는 실전 통합테스트 때 터미널에 바로 칠 명령만 빠르게 찾기 위한 안내서입니다.

## 기본 토픽

| 구분 | 토픽 |
| --- | --- |
| 입력 이미지 | `/camera/rgb/observer_01/compressed` |
| 추론 요청 | `/m0609/vision/capture_request` |
| YOLO OBB 결과 | `/m0609/vision/plate_obb` |
| YOLO debug 이미지 | `/m0609/vision/debug_image` |
| YOLO debug 압축 이미지 | `/m0609/vision/debug_image/compressed` |
| Brain pick/place goal | `/m0609/vision/pick_place_goal` |
| Motion domain goal | `/m0609_vision/pick_place_goal` |
| D455 카메라 월드 변환 | `/m0609/d455/t_world_camera` |

## 모든 터미널 공통 준비

통합테스트 터미널을 새로 열 때마다 먼저 실행합니다.

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
```

`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`는 아래 에러를 피하기 위한 설정입니다.

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port ... open_and_lock_file failed
```

YOLO/Brain launch 파일에는 이 설정이 기본으로 들어가 있지만, `rqt_image_view`, `ros2 topic`, Isaac Sim 쪽 터미널에도 같은 설정을 넣는 편이 안전합니다.

## 터미널 1: 이미지 토픽 확인

상대 컴퓨터나 Isaac Sim에서 `/camera/rgb/observer_01/compressed`를 보내고 있는지 먼저 확인합니다.

```bash
ros2 topic list
ros2 topic hz /camera/rgb/observer_01/compressed
```

이미지가 보이는지 rqt로 확인합니다.

```bash
ros2 run rqt_image_view rqt_image_view
```

rqt 창에서 `/camera/rgb/observer_01/compressed`를 선택합니다.

## 터미널 2: YOLO 실행

손목 D455 카메라 테스트라면 기본 입력 토픽은 이미 `/camera/image_rgb`입니다.

```bash
ros2 launch vision m0609_d455_yolo.launch.py
```

YOLO가 발행하는 debug 이미지는 rqt에서 `/m0609/vision/debug_image`를 선택해서 봅니다.

모델 파일을 직접 지정해야 하면 아래처럼 실행합니다.

```bash
ros2 launch vision m0609_d455_yolo.launch.py \
  model:=/absolute/path/to/best.pt
```

검출이 너무 안 되면 임시로 confidence를 낮춰 확인합니다.

```bash
ros2 launch vision m0609_d455_yolo.launch.py conf:=0.25
```

천장 top-view 카메라라면 아래 launch를 권장합니다. YOLO와 Brain을 한 파일로 실행하지만, 내부 노드는 분리되어 있습니다. 기본은 실시간 추론입니다.

```bash
ros2 launch vision m0609_topview_pipeline.launch.py
```

top-view launch는 `/camera/rgb/observer_01/compressed`를 compressed 이미지 입력으로 받고, 철판과 큐브 둘 다 후보로 봅니다. debug 이미지는 동일하게 `/m0609/vision/debug_image`입니다.

요청 기반 1회 추론 모드가 필요할 때만 trigger topic을 지정합니다.

```bash
ros2 launch vision m0609_topview_pipeline.launch.py trigger_topic:=/m0609/vision/capture_request
ros2 topic pub --once /m0609/vision/capture_request std_msgs/msg/Empty "{}"
```

철판만 작게 테스트:

```bash
ros2 launch vision m0609_topview_pipeline.launch.py allowed_class_ids:=0 conf:=0.05
```

큐브만 작게 테스트:

```bash
ros2 launch vision m0609_topview_pipeline.launch.py allowed_class_ids:=1 conf:=0.05
```

## 터미널 3: Brain 실행

YOLO 결과를 받아 pick/place goal을 계산합니다.

```bash
ros2 launch vision m0609_d455_brain.launch.py
```

천장 top-view 카메라에서는 위 wrist D455 brain launch 대신 `m0609_topview_pipeline.launch.py`를 쓰는 편이 안전합니다. wrist D455 brain launch는 `/m0609/d455/t_world_camera`를 기다립니다.

Brain이 제대로 출력하는지 확인합니다.

```bash
ros2 topic echo /m0609/vision/pick_place_goal
```

## 터미널 4: rqt debug 보기

```bash
ros2 run rqt_image_view rqt_image_view
```

확인 순서:

1. `/camera/rgb/observer_01/compressed`
2. `/m0609/vision/debug_image`
3. `/m0609/vision/debug_image/compressed`

원본 이미지는 보이는데 debug 이미지가 안 보이면 YOLO 터미널 로그를 먼저 확인합니다. 모델 경로 오류, CUDA 오류, 입력 토픽 미수신, class filter가 흔한 원인입니다.

debug topic 발행 여부만 확인하려면:

```bash
ros2 topic hz /m0609/vision/debug_image
ros2 topic hz /m0609/vision/debug_image/compressed
```

## 한 컴퓨터에서 전체 vision만 실행

천장 top-view에서 이미지 토픽이 이미 들어오고 있다면 아래를 켭니다.

```bash
ros2 launch vision m0609_topview_pipeline.launch.py
```

추론할 때마다 요청을 한 번 보냅니다.

```bash
ros2 topic pub --once /m0609/vision/capture_request std_msgs/msg/Empty "{}"
```

## 모션 컴퓨터와 ROS_DOMAIN_ID가 다를 때

같은 domain이면 `goal_gateway`는 필요 없습니다.

다른 domain을 쓸 때만 goal만 넘깁니다. 예를 들어 vision은 `105`, motion은 `103`이라면:

```bash
ros2 run vision goal_gateway \
  --source-domain 105 \
  --target-domain 103 \
  --input-topic /m0609/vision/pick_place_goal \
  --output-topic /m0609_vision/pick_place_goal
```

이미지와 debug 이미지는 gateway로 넘기지 않습니다. 모션 쪽에는 최종 goal만 보내는 구조가 안전합니다.

모션 컴퓨터에서 수신 확인:

```bash
export ROS_DOMAIN_ID=103
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic echo /m0609_vision/pick_place_goal
```

모션팀 executor가 다른 토픽을 구독한다면 gateway의 `--output-topic`을 그 토픽명으로 맞춥니다.

## 짧은 별칭

위의 공통 준비 명령을 `source`로 실행하면 아래 alias가 자동으로 등록됩니다. alias는 현재 터미널에만 적용되므로 새 터미널을 열면 다시 `source` 해야 합니다.

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
```

이후에는 아래처럼 실행합니다.

```bash
vyolo
vbrain
vtop
vtop_once
vtrigger
vgateway103
vmotiongoal
vrqt
vgoal
```

## 빠른 문제 확인표

| 증상 | 먼저 확인할 것 |
| --- | --- |
| `/camera/rgb/observer_01/compressed`가 안 보임 | 양쪽 PC의 `ROS_DOMAIN_ID`, 네트워크, 이미지 publisher 실행 여부 |
| FastDDS SHM 에러 | 모든 터미널에 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| 원본은 보이는데 debug가 안 보임 | YOLO launch 실행 여부, 모델 경로, YOLO 로그 |
| debug는 보이는데 검출이 없음 | `vtrigger` 요청 여부, confidence, 모델 weight, top-view 실제 샘플 데이터, ROI/class filter |
| Brain goal이 안 나옴 | top-view는 `m0609_topview_pipeline.launch.py`, wrist-view는 `/m0609/d455/t_world_camera` 수신 여부 |
| 모션 컴에서 goal이 안 보임 | gateway 실행 여부, `ROS_DOMAIN_ID=103`, `/m0609_vision/pick_place_goal` echo |
| 로봇이 안 움직임 | motion executor가 `/m0609_vision/pick_place_goal` 또는 gateway output topic을 구독하는지 확인 |
