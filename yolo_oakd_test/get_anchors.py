
import torch
import sys

def get_anchors(pt_path):
    try:
        # Load the model
        # YOLOv5 Checkpoint usually is a dict with 'model', 'ema', etc.
        # Set weights_only=False to allow loading older pickles/custom models
        ckpt = torch.load(pt_path, map_location='cpu', weights_only=False)
        model = ckpt['model']
        
        # Detect layer is usually the last one, or identifiable
        # For YOLOv5, it's model.model[-1]
        
        if hasattr(model, 'model'):
            detect = model.model[-1]
            if hasattr(detect, 'anchors'):
                print("Anchors found:")
                print(detect.anchors)
                
                # Also need stride to convert anchors to pixels (if they are grid-normalized)
                # YOLOv5 anchors are usually stored in pixels relative to the original image size? 
                # Or sometimes grid relative.
                # In YOLOv5s Pytorch model, anchors are usually in pixels.
                
                print("\nFlattened for DepthAI:")
                # DepthAI expects a flat list of integers
                anchors_flat = []
                for i in range(detect.anchors.shape[0]):
                    row = detect.anchors[i].view(-1).tolist()
                    row_int = [int(x) for x in row]
                    anchors_flat.extend(row_int)
                print(anchors_flat)
                
                print("\nAnchor Masks (guessing standard 3-layer):")
                # Usually stride 8 (P3), 16 (P4), 32 (P5)
                # Anchors are usually sorted small to large.
                # default masks: { "side80": [0,1,2], "side40": [3,4,5], "side20": [6,7,8] }
                # But we should verify.
            else:
                print("No 'anchors' attribute in last layer.")
        else:
            print("Could not access 'model.model'.")
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    get_anchors('/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector/weights/best.pt')
