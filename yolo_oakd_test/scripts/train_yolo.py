#!/usr/bin/env python3
"""
Train YOLOv5n model on FCC4 can detection dataset
Fine-tunes pretrained YOLOv5n weights for can detection
"""

import os
import sys
import subprocess
from pathlib import Path

def main():
    # Paths
    # __file__ = .../Sigyn/yolo_oakd_test/scripts/train_yolo.py
    # parents[0] = scripts/, parents[1] = yolo_oakd_test/, parents[2] = Sigyn/
    workspace_root = Path(__file__).resolve().parents[2]  # /home/ros/sigyn_ws/src/Sigyn
    dataset_dir = workspace_root / "FCC4.v1i.yolov5pytorch"
    yolov5_dir = workspace_root / "yolov5"
    output_dir = workspace_root / "yolo_oakd_test" / "models"
    
    # Verify paths exist
    if not dataset_dir.exists():
        print(f"ERROR: Dataset not found at {dataset_dir}")
        sys.exit(1)
    
    if not yolov5_dir.exists():
        print(f"ERROR: YOLOv5 repo not found at {yolov5_dir}")
        print("Cloning YOLOv5 repository...")
        subprocess.run([
            "git", "clone", "https://github.com/ultralytics/yolov5.git",
            str(yolov5_dir)
        ], check=True)
    
    # Install YOLOv5 requirements
    print("Installing YOLOv5 dependencies...")
    subprocess.run([
        sys.executable, "-m", "pip", "install", "-r",
        str(yolov5_dir / "requirements.txt")
    ], check=True)
    
    # Create data.yaml with absolute paths
    data_yaml = output_dir / "can_data.yaml"
    data_yaml.parent.mkdir(parents=True, exist_ok=True)
    
    with open(data_yaml, 'w') as f:
        f.write(f"""# Can detection dataset configuration
train: {dataset_dir}/train/images
val: {dataset_dir}/valid/images
test: {dataset_dir}/test/images

nc: 1
names: ['CokeZero-can']
""")
    
    print(f"Created dataset config: {data_yaml}")
    
    # Train YOLOv5n
    print("\n" + "="*80)
    print("Training YOLOv5n on can detection dataset...")
    print("="*80 + "\n")
    
    train_cmd = [
        sys.executable,
        str(yolov5_dir / "train.py"),
        "--img", "640",
        "--batch", "16",
        "--epochs", "100",
        "--data", str(data_yaml),
        "--weights", "yolov5n.pt",  # Pretrained weights
        "--project", str(output_dir),
        "--name", "can_detector",
        "--patience", "20",  # Early stopping
        "--save-period", "10",
        "--device", "cpu",  # Use CPU (change to '0' for GPU if available)
    ]
    
    subprocess.run(train_cmd, cwd=yolov5_dir, check=True)
    
    # Find best weights
    best_weights = output_dir / "can_detector" / "weights" / "best.pt"
    if best_weights.exists():
        print(f"\n✓ Training complete! Best weights: {best_weights}")
        print(f"\nNext step: Run convert_to_blob.py to create OAK-D compatible model")
    else:
        print(f"\n✗ Training failed - weights not found at {best_weights}")
        sys.exit(1)

if __name__ == "__main__":
    main()
