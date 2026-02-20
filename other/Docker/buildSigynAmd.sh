#!/bin/bash
# Build script for Sigyn AMD64 Docker image with GPU support
# Execute this command within the Docker folder

echo "Building Sigyn AMD64 Docker image with GPU support..."
docker build --platform linux/amd64 -t sigyn_amd -f DockerSigynAmd .
echo "Build complete! Image name: sigyn_amd"
echo ""
echo "To run with GPU support:"
echo "docker run --gpus all -it --rm --name sigyn_container sigyn_amd"
echo ""
echo "To run without GPU:"
echo "docker run -it --rm --name sigyn_container sigyn_amd"
echo ""
echo "To run with workspace mounted:"
echo "docker run --gpus all -it --rm --name sigyn_container -v /home/ros/sigyn_ws:/home/ros/sigyn_ws sigyn_amd"
