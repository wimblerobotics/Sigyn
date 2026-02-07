import blobconverter
import sys

def convert():
    onnx_path = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector/weights/best.onnx"
    output_dir = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/"
    
    print("Converting...")
    blob_path = blobconverter.from_onnx(
        model=onnx_path,
        output_dir=output_dir,
        data_type="FP16",
        shaves=6,
        use_cache=False,
        optimizer_params=[
            "--scale_values=[255,255,255]",
            "--reverse_input_channels"
        ],
        version="2022.1" 
    )
    print(f"Done. Saved to {blob_path}")

if __name__ == "__main__":
    convert()
