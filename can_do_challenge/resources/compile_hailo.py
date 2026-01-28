
import os
import sys
import numpy as np
from PIL import Image

# Ensure a writable working directory before importing Hailo SDK (it writes logs on import)
_home = os.environ.get("HOME", "/tmp")
try:
    os.makedirs(_home, exist_ok=True)
    os.chdir(_home)
except Exception:
    pass

from hailo_sdk_client import ClientRunner, CalibrationDataType

model_name = "yolov8_trained"
onnx_path = "/workspace/resources/models/yolov8_trained.onnx"
hef_path = "/workspace/resources/models/yolov8_trained.hef"
calib_dir = "/workspace/resources/calibration_imgs"
input_size = 320

print(f"Starting compilation for {model_name}...")
sys.stdout.flush()

# 1. Create Runner
try:
    runner = ClientRunner(hw_arch='hailo8l')
    print("Initialized ClientRunner for hailo8l.")
except Exception as e:
    print(f"Failed to init with hailo8l: {e}. Trying hailo8...")
    runner = ClientRunner(hw_arch='hailo8')
    print("Initialized ClientRunner for hailo8.")

# 2. Parse ONNX
print("Translating ONNX model...")
runner.translate_onnx_model(
    onnx_path, 
    model_name,
    start_node_names=['images'],
    end_node_names=['/model.22/Sigmoid', '/model.22/Concat'], 
    net_input_shapes={"images": [1, 3, input_size, input_size]}
)
print("Translation successful.")
sys.stdout.flush()

print("Loading calibration images...")
images = [f for f in os.listdir(calib_dir) if f.endswith('.jpg')]
images = images[:64]  # Limit to 64

if len(images) == 0:
    print("ERROR: No images found!")
    sys.exit(1)

calib_imgs = []
for img_name in images:
    img_path = os.path.join(calib_dir, img_name)
    try:
        image = Image.open(img_path).convert("RGB").resize((input_size, input_size))
        img_data = np.array(image).astype(np.float32)
        img_data = img_data / 255.0
        calib_imgs.append(img_data)
    except Exception as e:
        print(f"Failed to load image {img_name}: {e}")

if len(calib_imgs) == 0:
    print("ERROR: Failed to load any calibration images!")
    sys.exit(1)

# Stack to shape (N, H, W, C) to match Hailo optimizer expectations
calib_np = np.stack(calib_imgs, axis=0)

print(f"Prepared {calib_np.shape[0]} images for calibration with shape {calib_np.shape}.")

# 4. Optimize
print("Starting optimization (quantization)...")
sys.stdout.flush()

# Explicitly pass numpy array data and data type
runner.optimize(calib_np, data_type=CalibrationDataType.np_array)

print("Optimization successful.")

# 5. Compile
print("Starting compilation to HEF...")
hef = runner.compile()

with open(hef_path, "wb") as f:
    f.write(hef)

print(f"HEF file saved to {hef_path}")
