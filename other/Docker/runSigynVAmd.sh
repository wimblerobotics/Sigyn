#!/bin/bash

# --- Configuration ---
IMAGE_NAME="sigyn_vamd"
CONTAINER_NAME="sigyn_vamd_container"
TARGET_PLATFORM="linux/amd64"

# --- Main Script ---
# Check if a container with the same name is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "ℹ️  Attaching to running container '$CONTAINER_NAME'..."
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0
fi

# Check if a container with the same name exists but is stopped
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    echo "ℹ️  Starting and attaching to existing container '$CONTAINER_NAME'..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0
fi

# If no container exists, create a new one
echo "🚀 Launching new AMD64 container '$CONTAINER_NAME' via emulation..."

# Set up X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Get the absolute path to the parent directory of the script's location
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SIGYN_DIR=$(dirname "$SCRIPT_DIR")

docker run -it \
    --platform $TARGET_PLATFORM \
    --name $CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="$SIGYN_DIR:/home/ros/sigyn_ws/src/Sigyn" \
    --network=host \
    --privileged \
    --rm \
    $IMAGE_NAME

echo "✅ Container '$CONTAINER_NAME' has been stopped and removed."