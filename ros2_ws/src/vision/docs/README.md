# Vision Pipeline Notes

This ROS2 package owns the migrated vision runtime, Isaac evidence tools, and YOLO fine-tuning workflow.

## Runtime flow

1. `ros2 run vision yolo` reads a top-camera image and publishes an OBB payload.
2. `ros2 run vision brain` converts the OBB payload into a calibrated pick/place goal.
3. `ros2 launch vision vision_pipeline.launch.py` runs both nodes with shared arguments.

## Package layout

| Path | Purpose |
| --- | --- |
| `vision/` | Runtime ROS2/Python modules. |
| `vision/models/` | Fine-tuned and fallback YOLO OBB weights. |
| `launch/` | ROS2 launch files. |
| `tools/isaac/` | Isaac Sim smoke, evidence, and synthetic dataset tools. |
| `tools/training/` | YOLO evaluation, dataset conversion, and training tools. |
| `work/` | Generated images, datasets, runs, and reports. Ignored by git. |

See `tools/README.md` for hard-v2 evidence generation and fine-tuning commands.
