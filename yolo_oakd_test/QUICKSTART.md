# Quick Start Guide - YOLO OAK-D Can Detection

## Package Successfully Built! ✓

The `yolo_oakd_test` package has been created with:
- ✓ Custom message definition (CanDetection)
- ✓ Main detector node with extensive instrumentation
- ✓ Training and conversion scripts
- ✓ Launch file with URDF integration
- ✓ RViz configuration

## Next Steps

### Step 1: Train the YOLO Model (~15 minutes)

```bash
cd /home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test
python3 scripts/train_yolo.py
```

**What this does:**
- Downloads YOLOv5 repository if needed
- Installs dependencies
- Trains YOLOv5n on your 77 training images
- Uses pretrained weights for transfer learning
- Runs for 100 epochs with early stopping
- Saves best model to `models/can_detector/weights/best.pt`

**Expected output:**
```
Epoch    GPU_mem   box_loss   obj_loss   cls_loss  Instances       Size
  0/99      0.5G     0.0523     0.0312          0         32        640
...
100/100    0.5G     0.0123     0.0089          0         32        640

Training complete. Results saved to models/can_detector/
Best mAP@0.5: 0.892
```

### Step 2: Convert to OAK-D Blob (~5 minutes) **[KNOWN ISSUE - SEE BELOW]**

**⚠️ CONVERSION ISSUE:** There's a version incompatibility between modern OpenVINO (2024.x) and blobconverter's cloud service (OpenVINO 2021.4/2022.1). The IR XML format has changed and old compilers can't read new XML files.

**WORKAROUND:** Using existing pre-trained blob from `base/config/nn/`:
```bash
cp /home/ros/sigyn_ws/src/Sigyn/base/config/nn/can_yolov5n_640_openvino_2022_1_6shave.blob \
   /home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector.blob
```

**To convert your newly trained model (future):**
- Requires OpenVINO 2021.4 or 2022.1 environment for IR creation + blob compilation
- Cannot use modern OpenVINO with blobconverter cloud service
- See: https://docs.luxonis.com/ for updated conversion methods

**Expected IF conversion worked:**

```bash
cd /home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test
python3 scripts/convert_to_blob.py
```

**What this does:**
- Exports PyTorch model to ONNX format (opset 10 for Myriad X compatibility)
- Converts ONNX to OpenVINO IR (FP16)
- Compiles OpenVINO to OAK-D blob (6 shaves)
- Saves final blob to `models/can_detector.blob`

**Important:** Uses ONNX opset 10 because opset 11+ uses `Interpolate` layer which is not supported by Myriad X VPU. Opset 10 uses `Upsample` which is compatible.

**Expected output:**
```
Step 1/3: Exporting PyTorch model to ONNX...
✓ ONNX model saved

Step 2/3: Converting ONNX to OpenVINO IR...
✓ OpenVINO IR saved

Step 3/3: Converting to OAK-D blob...
✓ SUCCESS! OAK-D blob created
Blob file size: 6.2 MB
```

### Step 3: Run Detection Test

Position the can at the test location:
- ~80cm from robot center
- ~1.2m from camera
- ~4cm left of camera centerline

Then launch:

```bash
cd /home/ros/sigyn_ws
source install/setup.bash
ros2 launch yolo_oakd_test test_can_detection.launch.py
```

**What to expect:**
- Robot URDF loads with TF tree
- OAK-D connects and starts pipeline
- RViz2 opens showing:
  - Robot model
  - Annotated image with bounding boxes
  - RGB preview (640x640)
  - Depth visualization
- Terminal shows extensive logging:
  ```
  [OAKD_DETECTOR] Processing frame #1
  [YOLO] Detection #0:
    Confidence: 0.92
    BBox (1080p): xmin=830, ymin=420, xmax=1090, ymax=650
    Spatial (m): X=0.042, Y=-0.153, Z=1.187
    Point in base_link: X=0.682m, Y=0.042m, Z=0.118m
    Error: ΔX=-118mm, ΔY=+2mm, ΔZ=+18mm
    3D Error: 119mm
  ```

### Step 4: Analyze Results

**Success criteria:**
- Detection confidence > 0.8
- 3D position error < 50mm
- Consistent detections across frames
- FPS > 10

**If errors are too large:**
1. Check camera calibration
2. Verify can position with tape measure
3. Review URDF transform (oak_rgb_camera_optical_frame → base_link)
4. Check stereo depth quality (good lighting, textured surfaces)

## Troubleshooting

### Training fails
```bash
# Check CUDA availability (optional, faster)
python3 -c "import torch; print(torch.cuda.is_available())"

# Install missing dependencies
pip install -r /home/ros/sigyn_ws/src/Sigyn/yolov5/requirements.txt
```

### Conversion fails
```bash
# Install OpenVINO and blobconverter
pip install openvino-dev blobconverter
```

### No detections during test
- Verify blob path: `ls -lh models/can_detector.blob`
- Check camera connection: `lsusb | grep OAK`
- Review training metrics: Was mAP > 0.7?
- Adjust confidence threshold in node code

### TF errors
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# Should show: base_link → oak-d-base-frame → oak_rgb_camera_optical_frame

# Verify URDF
ros2 topic echo /robot_description --once | head -20
```

## Architecture Details

**Threading Model:** Option C
- DepthAI pipeline runs in background thread
- Detections sent via Queue to ROS thread
- MultiThreadedExecutor (4 threads) for publishers
- No blocking, no race conditions

**Coordinate Frames:**
```
Detection in image (pixels) → Normalize (0-1)
  ↓
YOLO processing (640x640)
  ↓  
Spatial fusion with depth → 3D point in camera optical frame (mm)
  ↓
Convert to meters → TF transform
  ↓
3D point in base_link (meters)
```

**Logging Strategy:**
- Every frame logged with timestamp
- Each detection shows all intermediate values
- Transform details logged (translation, rotation)
- Error analysis against expected values
- Debug messages in custom CanDetection message

## Next Integration Steps

Once this isolated test validates correctly:
1. Integrate into behavior tree as `DetectCan` node
2. Add visual servoing for approach (use 2D bbox center)
3. Implement gripper positioning based on 3D pose
4. Add grasp verification using same camera

## Files Created

```
yolo_oakd_test/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── can_detection.rviz
├── launch/
│   └── test_can_detection.launch.py
├── msg/
│   └── CanDetection.msg
├── models/
│   └── (generated: can_detector.blob)
├── scripts/
│   ├── train_yolo.py
│   └── convert_to_blob.py
└── yolo_oakd_test/
    └── oakd_can_detector.py
```

## Contact & Support

If you encounter issues, check:
1. Terminal output for detailed error messages
2. RViz for visual confirmation
3. `/tmp/build_yolo_oakd.log` for build errors
4. Logs show every processing step - use them!

Ready to train the model? Run:
```bash
cd /home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test
python3 scripts/train_yolo.py
```
