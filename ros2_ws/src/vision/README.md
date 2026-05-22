# vision

ROS2 package for the OBB vision pipeline.

## Short commands

```bash
ros2 run vision yolo
ros2 run vision brain
ros2 launch vision vision_pipeline.launch.py
```

Defaults:

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

## M0609 D455 test

Use ROS domain 104 for the Isaac/YOLO/brain test:

```bash
export ROS_DOMAIN_ID=104
```

For the M0609 D455 test, start `tools/isaac/m0609_d455_yolo_rqt_smoke.py --gui --execute-goals` first. It prints the D455 `--t-world-camera` value that must be passed to `ros2 run vision brain`; the brain default transform is for the fixed top camera, not the wrist-mounted D455.

`goal_gateway` is only a cross-domain goal bridge. It is not a robot controller and is not needed when every test process uses domain 104.
