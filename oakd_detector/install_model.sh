#!/bin/bash
# Quick setup script for installing your RoboFlow YOLO26 model

echo "=== YOLO26 OAK-D Model Installation ==="
echo ""
echo "When your RoboFlow model is ready, follow these steps:"
echo ""
echo "1. Download your model from RoboFlow:"
echo "   - Go to your project → Versions → v3"
echo "   - Click Export → Select 'YOLO11 PyTorch'"
echo "   - Download and extract the zip file"
echo ""
echo "2. Copy the model file to the resources directory:"
echo "   cp ~/Downloads/your-model/best.pt $HOME/sigyn_ws/src/Sigyn/oakd_detector/resources/"
echo ""
echo "3. Verify the model is in place:"
echo "   ls -lh $HOME/sigyn_ws/src/Sigyn/oakd_detector/resources/best.pt"
echo ""
echo "4. Rebuild the workspace:"
echo "   cd $HOME/sigyn_ws"
echo "   colcon build --symlink-install --packages-select oakd_detector base"
echo "   source install/setup.bash"
echo ""
echo "5. Test the detector:"
echo "   ros2 launch base sigyn.launch.py do_oakd:=true use_sim_time:=false"
echo ""
echo "6. In another terminal, view detections:"
echo "   ros2 topic echo /oakd/detections"
echo "   # or view images:"
echo "   ros2 run rqt_image_view rqt_image_view /oakd/annotated_image"
echo ""
echo "7. Test image capture (from bluetooth or manually):"
echo "   ros2 topic pub --once /sigyn/take_oakd_picture std_msgs/Bool \"data: true\""
echo "   ls -lht $HOME/sigyn_ws/src/Sigyn/trained_images/ | head"
echo ""
echo "=== Current Status ==="
echo ""

# Check if model exists
if [ -f "$HOME/sigyn_ws/src/Sigyn/oakd_detector/resources/best.pt" ]; then
    echo "✅ Model file found!"
    ls -lh "$HOME/sigyn_ws/src/Sigyn/oakd_detector/resources/best.pt"
else
    echo "❌ Model file NOT found yet"
    echo "   Expected location: $HOME/sigyn_ws/src/Sigyn/oakd_detector/resources/best.pt"
fi

echo ""

# Check if trained_images directory exists
if [ -d "$HOME/sigyn_ws/src/Sigyn/trained_images" ]; then
    echo "✅ Trained images directory exists"
    echo "   Location: $HOME/sigyn_ws/src/Sigyn/trained_images"
    count=$(ls -1 "$HOME/sigyn_ws/src/Sigyn/trained_images" 2>/dev/null | wc -l)
    echo "   Current images: $count"
else
    echo "❌ Trained images directory NOT found"
fi

echo ""

# Check if ultralytics is installed
if python3 -c "from ultralytics import YOLO" 2>/dev/null; then
    echo "✅ Ultralytics (YOLO) is installed"
    python3 -c "from ultralytics import __version__; print(f'   Version: {__version__}')"
else
    echo "❌ Ultralytics NOT installed or not accessible"
    echo "   Run: pip3 install --upgrade ultralytics"
fi

echo ""
echo "=== Ready to go! ==="
echo "Once you have your RoboFlow model, copy it to resources/best.pt and rebuild."
