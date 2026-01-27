# YOLO26 OAK-D Detector Setup

This package now includes a YOLO26-based detector for the OAK-D camera, optimized for detecting Coke cans using your RoboFlow v3 model.

## Features

1. **YOLO26 End-to-End Detection** - No NMS post-processing needed, 43% faster CPU inference
2. **RoboFlow v3 Model Support** - Direct integration with your trained model
3. **Image Capture** - Listens to `/sigyn/take_oakd_picture` topic to save training images
4. **Standard ROS2 Outputs**:
   - `/oakd/detections` (vision_msgs/Detection2DArray) - Detection bounding boxes and classes
   - `/oakd/annotated_image` (sensor_msgs/Image) - Image with bounding boxes drawn
   - `/oakd/raw_image` (sensor_msgs/Image) - Raw camera feed

## Setup Instructions

### 1. Download Your RoboFlow Model

When your RoboFlow model is ready:

1. Go to your RoboFlow project
2. Navigate to "Versions" → v3
3. Click "Export"
4. Select "YOLO11 PyTorch" (or YOLO26 if available)
5. Download the zip file
6. Extract and locate the `best.pt` file

### 2. Install the Model

Copy your model to the oakd_detector resources directory:

```bash
cp ~/Downloads/your-roboflow-model/best.pt /home/ros/sigyn_ws/src/Sigyn/oakd_detector/resources/
```

### 3. Verify Ultralytics Installation

Ensure you have the latest Ultralytics with YOLO26 support:

```bash
pip3 install --upgrade ultralytics
python3 -c "from ultralytics import YOLO; print('Ultralytics OK')"
```

### 4. Build the Package

```bash
cd /home/ros/sigyn_ws
colcon build --symlink-install --packages-select oakd_detector base
source install/setup.bash
```

### 5. Launch

Launch Sigyn with the OAK-D detector enabled:

```bash
ros2 launch base sigyn.launch.py do_oakd:=true use_sim_time:=false
```

Or launch just the detector:

```bash
ros2 launch oakd_detector yolo26_oakd_detector.launch.py
```

## Usage

### View Detections

```bash
# View detection messages
ros2 topic echo /oakd/detections

# View annotated image in RViz or:
ros2 run rqt_image_view rqt_image_view /oakd/annotated_image
```

### Capture Training Images

The detector listens to `/sigyn/take_oakd_picture` and saves images from the right camera when requested.

From the bluetooth joystick (or any publisher):

```bash
# Capture an image
ros2 topic pub --once /sigyn/take_oakd_picture std_msgs/Bool "data: true"
```

Images are saved to: `/home/ros/sigyn_ws/src/Sigyn/trained_images/`

The bluetooth_joystick package already publishes to this topic when you press the designated button.

## Configuration Parameters

Launch file parameters (see `yolo26_oakd_detector.launch.py`):

- `mxId` - OAK-D device ID (default: '14442C1051B665D700')
- `model_path` - Path to YOLO26 .pt file (default: resources/best.pt)
- `confidence_threshold` - Detection confidence threshold (default: 0.5)
- `image_size` - Input size for YOLO26 (default: 640)
- `trained_images_dir` - Where to save captured images (default: /home/ros/sigyn_ws/src/Sigyn/trained_images)

## Troubleshooting

### Model Not Found

If you see "Model file not found" error:
```bash
ls -la /home/ros/sigyn_ws/src/Sigyn/oakd_detector/resources/
# Ensure best.pt is there
```

### Import Errors

If you get depthai import errors:
```bash
source ~/depthai-venv/bin/activate
pip install depthai
deactivate
```

### No Detections

- Check confidence threshold (try lowering to 0.3)
- Verify your RoboFlow model is YOLO format (not TensorFlow)
- Check lighting conditions
- Use `ros2 topic echo /oakd/raw_image` to verify camera is working

## Topic Reference

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oakd/detections` | vision_msgs/Detection2DArray | Detected objects with bounding boxes |
| `/oakd/annotated_image` | sensor_msgs/Image | Image with detection boxes drawn |
| `/oakd/raw_image` | sensor_msgs/Image | Raw camera feed |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sigyn/take_oakd_picture` | std_msgs/Bool | Trigger image capture when true |

## Comparison: Old vs New Detector

| Feature | Old (oakd_detector_node) | New (yolo26_oakd_detector_node) |
|---------|-------------------------|--------------------------------|
| Model | YOLOv4 Tiny Blob | YOLO26 PyTorch |
| Input Size | 416x416 | 640x640 (configurable) |
| NMS Required | Yes (post-processing) | No (end-to-end) |
| Training | Pre-trained COCO | Your custom RoboFlow model |
| CPU Performance | Baseline | 43% faster |
| Image Capture | No | Yes (right camera) |
| Topics | Generic | Standard ROS2 vision_msgs |

## Next Steps

1. **Wait for your RoboFlow model** (15 minutes)
2. **Download and install** the best.pt file
3. **Rebuild** the workspace
4. **Test** with `do_oakd:=true`
5. **Collect more training data** using the capture feature
6. **Retrain** in RoboFlow with new images
7. **Repeat** to improve accuracy

## Notes

- The detector uses the **right mono camera** for image capture (better for training data)
- Images are saved with timestamp: `oakd_capture_YYYYMMDD_HHMMSS.jpg`
- YOLO26's end-to-end design means **no NMS configuration needed**
- The model runs on **CPU** by default (perfect for edge deployment)
