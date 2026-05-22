# Vision 도구

이 폴더는 Isaac Sim 기반 검증 데이터 생성, YOLO 평가, fine-tuning dataset 변환, 추가 학습을 위한 도구 모음입니다. 생성되는 이미지, CSV, dataset, 학습 결과는 기본적으로 `work/` 아래에 저장합니다.

## hard-v2 검증 샘플 생성

철판, 철큐브, 배경-only 샘플을 랜덤 조명/재질/글레어/노이즈 조건에서 렌더링합니다. 배경-only 샘플은 배경과 물체를 구분하도록 추가 파인튜닝할 때 사용합니다.

```bash
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  tools/isaac/isaac_yolo_confusion_matrix.py \
  --count 400 \
  --difficulty hard \
  --background-ratio 0.25 \
  --seed 43 \
  --clean
```

## 모델 평가

생성된 `samples.csv`를 현재 YOLO 모델로 평가하고 confusion matrix, 실패 이미지, background false positive 비율을 저장합니다.

```bash
python3 tools/training/yolo_confusion_from_samples.py \
  --samples-csv work/confusion_matrix_hard_v2/samples.csv \
  --model vision/models/yolo11s_obb_metal_hard-v2_refinetune_best.pt \
  --save-images failures \
  --device auto
```

## fine-tuning dataset 생성

평가 결과인 `samples_evaluated.csv`를 YOLO OBB 학습 포맷으로 변환합니다. Background 샘플은 빈 label 파일로 저장됩니다.

```bash
python3 tools/training/samples_to_yolo_obb_dataset.py \
  --samples-csv work/confusion_matrix_hard_v2/samples_evaluated.csv \
  --output-dir work/datasets/metal_objects_hard_v2 \
  --cube-repeat 2 \
  --failure-repeat 2 \
  --clean
```

## 추가 파인튜닝

기본 설정은 fine-tuned 모델에서 이어서 학습하도록 맞춰져 있습니다.

```bash
python3 tools/training/train_yolo11s_obb.py
```

기본 학습값은 `epochs=50`, `imgsz=960`, `batch=8`, `lr0=0.00015`, `patience=10`, `freeze=10`입니다. 전체 학습은 CUDA가 가능한 환경에서 실행하는 것을 권장합니다.

## M0609 D455 wrist-view 큐브 보강 데이터

D455 손목 카메라에서는 그리퍼, 흰 라인, 반사광, 그림자가 같이 보이므로 top-camera 데이터만으로는 큐브를 놓치거나 오탐할 수 있습니다. 실제 smoke scene과 같은 시야에서 큐브와 background-negative 샘플을 생성합니다.

```bash
cd /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  tools/isaac/m0609_d455_yolo_rqt_smoke.py \
  --capture-dataset \
  --capture-count 600 \
  --capture-background-ratio 0.40 \
  --capture-clean
```

생성된 dataset yaml은 `work/datasets/m0609_d455_wrist_obb/m0609_d455_wrist_obb.yaml`입니다. 빠른 큐브 보강 fine-tuning은 아래처럼 실행합니다.

```bash
python3 tools/training/train_yolo11s_obb.py \
  --data work/datasets/m0609_d455_wrist_obb/m0609_d455_wrist_obb.yaml \
  --name yolo11s_obb_m0609_d455_wrist \
  --epochs 30 \
  --freeze 10
```

학습 후 `work/runs/metal_objects_obb/yolo11s_obb_m0609_d455_wrist/weights/best.pt`를 `ros2 launch vision m0609_d455_yolo.launch.py model:=...`로 지정해서 확인합니다.

## M0609 D455 + YOLO rqt 확인

실전 통합테스트용 짧은 명령은 [`../docs/integration_test_quickstart.md`](../docs/integration_test_quickstart.md)를 먼저 봅니다.

천장 top-view 카메라를 쓰는 통합테스트라면 wrist D455 전용 launch 대신 아래를 사용합니다.

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
vtop
vtrigger
```

Isaac Sim에서 M0609, RG2, RealSense D455, 철큐브를 띄우고 D455 color 이미지를 ROS2로 내보냅니다. ROS domain은 기본값 `105`입니다.

터미널 1: Isaac Sim 카메라 publisher와 goal executor

```bash
bash /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/isaac/run_m0609_d455_smoke.sh
```

터미널 1은 D455 color 이미지와 함께 `/m0609/d455/t_world_camera`도 계속 발행합니다. D455는 손목 카메라이므로 brain은 이 transform 토픽을 받아 픽 좌표를 계산합니다.

터미널 2: YOLO debug 이미지와 OBB 발행

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
ros2 launch vision m0609_d455_yolo.launch.py
```

터미널 3: brain 실행

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
ros2 launch vision m0609_d455_brain.launch.py
```

`--cell m0609`일 때 brain은 기본으로 `/m0609/d455/t_world_camera`를 구독합니다. 다른 토픽명을 쓰고 싶으면 `--camera-transform-topic`으로 지정합니다.

터미널 4: rqt 확인

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh
vrqt
```

`rqt_image_view`에서 통합테스트 원본은 `/camera/rgb/observer_01/compressed`, YOLO 결과는 `/m0609/vision/debug_image` 또는 `/m0609/vision/debug_image/compressed`를 선택합니다. Isaac smoke tool을 단독으로 돌리는 경우 원본은 `/m0609/d455/color/image`일 수 있습니다. depth 시각화가 필요하면 터미널 1 명령에 `--publish-depth-vis`를 추가하고 `/m0609/d455/depth/vis`를 확인합니다.

### goal_gateway는 언제 쓰나?

같은 ROS domain 105 안에서 테스트할 때는 `goal_gateway`를 돌리지 않습니다. `goal_gateway`는 vision 쪽 domain과 motion/controller 쪽 domain이 서로 다를 때, 작은 `Float32MultiArray` goal만 복사하는 브리지입니다. 로봇을 움직이는 컨트롤러가 아니므로 gateway만 켜도 M0609는 움직이지 않습니다.

예를 들어 vision은 domain 105, motion은 domain 103에서 따로 돌릴 때만 아래처럼 사용합니다.

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/install/setup.bash
ros2 run vision goal_gateway \
  --source-domain 105 \
  --target-domain 103 \
  --input-topic /m0609/vision/pick_place_goal \
  --output-topic /m0609_vision/pick_place_goal
```

RG2가 큐브를 실제로 집는지 보는 테스트는 기존 visual-servo 스크립트로 먼저 확인합니다. 이 스크립트는 빨간 큐브 tracker 기반이라 YOLO 파이프라인과는 별도 검증입니다.

```bash
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  /home/rokey/dev_ws/isaac_sim/src/m0609_vision/M0609/m0609_pick_place_visual.py
```
