# OAK-D On‑Device YOLO Workflow (Repeatable)

This guide describes how to train a custom can detector and deploy it as an on‑device DepthAI blob with spatial detections (3D positions). It assumes your dataset is already exported in YOLO format.

## 0) Prereqs (one-time)
Create/activate the venv and install the toolchain you’ll use for training and export.

```bash
source ~/sigyn-venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade ultralytics onnx onnxruntime onnxslim blobconverter
```

Check versions so you can correlate with blob OpenVINO version:

```bash
python3 - <<'PY'
import depthai as dai
import pkg_resources
print('depthai:', dai.__version__)
print('ultralytics:', pkg_resources.get_distribution('ultralytics').version)
PY
ros2 pkg xml depthai_ros_driver | rg -n "<version>" -m 1
```

If you update DepthAI/driver versions later, regenerate the blob with the OpenVINO version your DepthAI stack expects (see Section 6).

## 1) Dataset location
- Dataset folder: [FCC3.v1i.yolo26](FCC3.v1i.yolo26)
- Dataset config: [FCC3.v1i.yolo26/data.yaml](FCC3.v1i.yolo26/data.yaml)
  - It defines 1 class: Can

## 2) Train a YOLO model (YOLOv5 repo)
**Chosen model:** YOLOv5n (anchor‑based)

**Why:** DepthAI v2 on Jazzy requires anchor‑based YOLO for on‑device decoding. The Ultralytics `yolo` CLI exports a single‑head output (YOLOv8‑style), which is **not** compatible with DepthAI’s YOLO parser. Use the YOLOv5 repository instead.

Train with fixed 640 size to match the dataset export.

Suggested settings (edit as needed):
- model: yolov5n.pt
- data: [FCC3.v1i.yolo26/data.yaml](FCC3.v1i.yolo26/data.yaml)
- imgsz: 640
- epochs: 100
- batch: 16

Commands (copy/paste):

```bash
cd ~/sigyn_ws/src/Sigyn
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
python3 -m pip install -r requirements.txt
python3 train.py --img 640 --batch 16 --epochs 100 --data ../FCC3.v1i.yolo26/data.yaml --weights yolov5n.pt --project ../can_do_challenge/resources/models --name can_yolov5n
```

What this does:
- Trains YOLOv5n at $640\times640$ with your dataset.
- Writes weights to [runs/detect/can_do_challenge/resources/models/can_yolov5n/weights](runs/detect/can_do_challenge/resources/models/can_yolov5n/weights).

Save the resulting weights as best.pt.

## 3) Export to OpenVINO IR (YOLOv5)
Export the trained model to OpenVINO IR via YOLOv5’s exporter. This preserves the multi‑scale YOLO heads required by DepthAI.

Commands (copy/paste):

```bash
cd ~/sigyn_ws/src/Sigyn/yolov5
python3 export.py --weights ../runs/detect/can_do_challenge/resources/models/can_yolov5n/weights/best.pt --img 640 --batch 1 --device cpu --include openvino
```

What this does:
- Creates OpenVINO IR files (XML + BIN) under the YOLOv5 export directory, usually alongside the weights.

Keep those IR files for blob conversion.

## 4) Convert OpenVINO IR to DepthAI blob (YOLOv5n)
DepthAI requires a .blob compiled for the OAK‑D’s OpenVINO version.

**Option A: blobconverter (recommended)**
1) Install tools:
  - `python3 -m pip install --upgrade blobconverter`
2) Convert (edit paths as needed):
  - `python3 -m blobconverter --openvino-xml <path>/best_openvino_model.xml --openvino-bin <path>/best_openvino_model.bin --shaves 6 --use_cache --output_dir base/config/nn --version 2022.1`

Commands (copy/paste, edit paths):

```bash
cd ~/sigyn_ws/src/Sigyn
python3 -m blobconverter --openvino-xml <path>/best_openvino_model/best.xml --openvino-bin <path>/best_openvino_model/best.bin --shaves 6 --use_cache --output_dir base/config/nn --version 2022.1
```

What this does:
- Compiles an OpenVINO blob for the OAK‑D (FP16, 6 shaves).

**Option B: depthai_tools**
1) Install tools:
  - `python3 -m pip install --upgrade depthai-tools`
2) Convert:
  - `python3 -m depthai_tools.compile_model --model <path>/best.xml --shaves 6`

Place the resulting blob in:
- [base/config/nn](base/config/nn)

## 5) Create a DepthAI YOLO config JSON (YOLOv5)
DepthAI needs a JSON config describing the model outputs and class labels.
Create a new JSON file and point the driver to it. Suggested location:
- [base/config/nn/can_yolov5.json](base/config/nn/can_yolov5.json)

Minimum fields you need:
- model.model_name: absolute path to the .blob file
- nn_config.NN_family: YOLO
- nn_config.NN_specific_metadata: classes, coordinates, anchors, anchor_masks, iou_threshold, confidence_threshold
- mappings.labels: ["Can"]

Notes:
- YOLOv5 is anchor‑based, so anchors and masks must be set. The template uses the standard COCO anchors for 640.
- The DepthAI ROS driver only supports `model.zoo` values of `depthai_examples` or `path`.
- For this workspace, we use `model.zoo: path` and an absolute blob path in [base/config/nn/can_yolov5.json](base/config/nn/can_yolov5.json).
- Keep class order aligned with your training labels.
- For $640\times640$ models, the grid sizes are 80, 40, 20, so `anchor_masks` keys should be `side80`, `side40`, `side20`.

## 6) Point the OAK‑D driver to the new config
Update the OAK‑D camera config to use your custom JSON:
- [base/config/oakd_camera.yaml](base/config/oakd_camera.yaml)

Set:
- nn.i_nn_config_path: a package-relative path of the form `package_name/config_name`.

For this workspace:
- [base/config/oakd_camera_can_yolov8.yaml](base/config/oakd_camera_can_yolov8.yaml) points to can_yolov5.json in the install share.

To keep the default pipeline untouched, you can switch to a dedicated config file:
- [base/config/oakd_camera_can_yolov8.yaml](base/config/oakd_camera_can_yolov8.yaml)

## 7) Build and launch
Rebuild so the installed share contains your new JSON/blob.

```bash
cd ~/sigyn_ws
colcon build --symlink-install --packages-select base can_do_challenge oakd_detector
source install/setup.bash
```

## 8) Verify spatial detections
The on‑device pipeline publishes 3D detections:
- Topic: /oakd_top/oak/nn/spatial_detections
- Message type: depthai_ros_msgs/msg/SpatialDetectionArray

Use that topic as the source of 3D can positions. The BT already consumes those spatial detections.

## 9) Repeatable checklist
1) Export new dataset in YOLO format.
2) Train YOLO model at imgsz=640.
3) Export to ONNX (fixed input size).
4) Convert ONNX to blob.
5) Update the YOLO JSON config (class list, anchors, thresholds).
6) Update [base/config/oakd_camera.yaml](base/config/oakd_camera.yaml) to point to the JSON.
7) Launch and verify spatial detections.

## 10) Updating for new OAK‑D software versions
When you update `depthai` or `depthai_ros_driver`, regenerate the blob to match the OpenVINO version that your DepthAI stack expects.

Commands:

```bash
python3 - <<'PY'
import depthai as dai
print('depthai:', dai.__version__)
print('default OpenVINO:', dai.OpenVINO.Version.VERSION_2022_1)
PY
```

If you switch to a different OpenVINO version (e.g., 2022.2), re-run blob conversion with `--version 2022.2` and update the blob name in [base/config/nn/can_yolov5.json](base/config/nn/can_yolov5.json).

## Where to store outputs
- ONNX: [runs/detect/can_do_challenge/resources/models/can_yolov5n/weights](runs/detect/can_do_challenge/resources/models/can_yolov5n/weights)
- Blob: [base/config/nn](base/config/nn)
- YOLO JSON: [base/config/nn](base/config/nn)

YOLOv5n is the selected family for this pipeline. The JSON template is provided below; update it once the blob is generated.