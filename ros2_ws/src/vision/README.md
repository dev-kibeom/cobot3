# vision

ROS2 package for the OBB vision pipeline.

## M0609 통합테스트 빠른 안내

실전 통합테스트 때는 [docs/integration_test_quickstart.md](docs/integration_test_quickstart.md)를 먼저 봅니다.

핵심 기본값:

- 입력 이미지: `/camera/rgb/observer_01/compressed`
- YOLO debug 이미지: `/m0609/vision/debug_image`
- YOLO OBB 결과: `/m0609/vision/plate_obb`
- Brain goal: `/m0609/vision/pick_place_goal`
- FastDDS SHM 에러 회피: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`

기본 실행:

```bash
source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh

vyolo
vbrain
vrqt
```

천장 top-view 카메라에서는 wrist D455 전용 launch 대신 아래를 권장합니다.

```bash
vtop
vtrigger
```

## Short commands

```bash
ros2 run vision yolo
ros2 run vision brain
ros2 launch vision vision_pipeline.launch.py
```

Global pipeline defaults:

- YOLO: `/global/top_camera/image` -> `/global/vision/plate_obb`
- Brain: `/global/vision/plate_obb` -> `/global/vision/pick_place_goal`
- Model: `vision/models/yolo11s_obb_metal_hard-v2_refinetune_best.pt`

Useful launch overrides:

```bash
ros2 launch vision vision_pipeline.launch.py \
  cell:=global \
  image_topic:=/camera/image_rgb/compressed \
  image_transport:=compressed \
  publish_debug:=true
```

Isaac Sim tools are in `tools/isaac`, YOLO evaluation/training tools are in `tools/training`, and generated outputs go under `work/`.

For the current cube/panel USD files, start with the normal-light dataset recipe in `tools/README.md`. It uses `iron_cube.usd` and `iron_panel.usd`, mild camera-height randomization for distance robustness, and keeps the scene easy enough for rectangles to be learned first.

## M0609 D455 test

Use ROS domain 105 for the Isaac/YOLO/brain test:

```bash
export ROS_DOMAIN_ID=105
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

For the current M0609 D455 integration test, the incoming image topic is `/camera/image_rgb`. Run YOLO with:

```bash
ros2 launch vision m0609_d455_yolo.launch.py
```

For request-based capture, pass a trigger topic and publish one empty message when motion/brain asks for a fresh inference:

```bash
ros2 launch vision m0609_d455_yolo.launch.py trigger_topic:=/m0609/vision/capture
ros2 topic pub -1 /m0609/vision/capture std_msgs/msg/Empty "{}"
```

Then select `/m0609/vision/debug_image` in `rqt_image_view`.

The brain launch subscribes `/m0609/d455/t_world_camera` by default for the wrist-mounted D455:

```bash
ros2 launch vision m0609_d455_brain.launch.py
```

For a ceiling-mounted top-view camera, use:

```bash
ros2 launch vision m0609_topview_pipeline.launch.py
```

`goal_gateway` is only a cross-domain goal bridge. It is not a robot controller and is not needed when every test process uses domain 105.

If the motion computer expects `/isaac/goal_pos_topic` as `geometry_msgs/msg/PoseStamped`, run `goal_gateway` with `--pose-output-topic /isaac/goal_pos_topic --pose-source pick --min-confidence 0.70`.
