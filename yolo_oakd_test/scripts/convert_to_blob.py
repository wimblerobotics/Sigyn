#!/usr/bin/env python3
"""
Convert trained YOLOv5 model to OAK-D blob format
Pipeline: PyTorch (.pt) → OpenVINO IR (via YOLOv5 export.py) → Blob (.blob)

CRITICAL: YOLOv5 has built-in OpenVINO export that creates 2021.4-compatible IR files.
Using this instead of separate ONNX→OpenVINO conversion avoids version incompatibilities.
"""

import os
import sys
import subprocess
from pathlib import Path

def main():
    # Paths
    workspace_root = Path(__file__).resolve().parents[2]  # /home/ros/sigyn_ws/src/Sigyn
    yolov5_dir = workspace_root / "yolov5"
    models_dir = workspace_root / "yolo_oakd_test" / "models"
    weights_path = models_dir / "can_detector" / "weights" / "best.pt"
    
    if not weights_path.exists():
        print(f"ERROR: Trained weights not found at {weights_path}")
        print("Run train_yolo.py first!")
        sys.exit(1)
    
    print("="*80)
    print("Converting YOLOv5 model to OAK-D blob format")
    print("="*80)
    
    # Step 1: Export directly to OpenVINO IR (YOLOv5 has built-in support)
    print("\nStep 1/2: Exporting PyTorch model to OpenVINO IR...")
    print("NOTE: Using YOLOv5's built-in OpenVINO export for 2021.4 compatibility")
    
    export_cmd = [
        sys.executable,
        str(yolov5_dir / "export.py"),
        "--weights", str(weights_path),
        "--include", "openvino",  # Direct OpenVINO export
        "--img", "640",
    ]
    
    subprocess.run(export_cmd, cwd=yolov5_dir, check=True)
    
    # YOLOv5 exports to same directory as weights with _openvino_model suffix
    exported_openvino = weights_path.parent / "best_openvino_model"
    openvino_dir = models_dir / "can_detector_openvino"
    
    if exported_openvino.exists():
        # Move to our desired location
        if openvino_dir.exists():
            import shutil
            shutil.rmtree(openvino_dir)
        subprocess.run(["mv", str(exported_openvino), str(openvino_dir)], check=True)
        
        ir_xml = openvino_dir / "best.xml"
        ir_bin = openvino_dir / "best.bin"
        
        # Rename files
        (openvino_dir / "best.xml").rename(openvino_dir / "can_detector.xml")
        (openvino_dir / "best.bin").rename(openvino_dir / "can_detector.bin")
        
        ir_xml = openvino_dir / "can_detector.xml"
        ir_bin = openvino_dir / "can_detector.bin"
        
        print(f"✓ OpenVINO IR saved to: {openvino_dir}")
        print(f"  XML: {ir_xml}")
        print(f"  BIN: {ir_bin}")
    else:
        print(f"✗ OpenVINO export failed")
        sys.exit(1)
    
    # Step 2: Convert OpenVINO IR to OAK-D blob
    print("\nStep 2/2: Converting OpenVINO IR to OAK-D blob...")
    print("NOTE: This requires blobconverter. Install with:")
    print("  pip install blobconverter")
    
    try:
        import blobconverter
    except ImportError:
        print("\nInstalling blobconverter...")
        subprocess.run([sys.executable, "-m", "pip", "install", "blobconverter"], check=True)
        import blobconverter
    
    blob_path = models_dir / "can_detector.blob"
    
    # Use blobconverter with 2021.4 (matches YOLOv5's OpenVINO export)
    print("Compiling blob with blobconverter (OpenVINO 2021.4, 6 shaves, FP16)...")
    blob = blobconverter.from_openvino(
        xml=str(ir_xml),
        bin=str(ir_bin),
        data_type="FP16",
        shaves=6,
        version="2021.4",  # Must match YOLOv5's export version
        output_dir=str(models_dir)
    )
    
    if Path(blob).exists():
        subprocess.run(["mv", blob, str(blob_path)], check=True)
        print(f"\n{'='*80}")
        print(f"✓ SUCCESS! OAK-D blob created: {blob_path}")
        print(f"{'='*80}")
        print(f"\nModel is ready to use with OAK-D!")
        print(f"Blob file size: {blob_path.stat().st_size / 1024 / 1024:.2f} MB")
    else:
        print(f"\n✗ Blob conversion failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
