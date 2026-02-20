#!/bin/bash

# X11 setup (keep existing code)
XAUTH=/tmp/.docker.xauth
if [ -f $XAUTH ]; then
    rm -f $XAUTH
fi
touch $XAUTH
chmod 644 $XAUTH

xhost +local:docker 2>/dev/null || echo "Warning: xhost command failed"
if [ ! -z "$DISPLAY" ]; then
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || echo "Warning: xauth setup failed"
fi

# Run Docker with --network=host to share ROS network
docker run \
    -it \
    --rm \
    --privileged \
    --network=host \
    -v /dev/:/dev/ \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "XAUTHORITY=/tmp/.docker.xauth" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:/tmp/.docker.xauth:ro \
    --device /dev/snd \
    -e ALSADEV=hw:2,0 \
    --dns=8.8.8.8 \
    --dns=8.8.4.4 \
    -v /home/ros/sigyn_ws:/home/ros/sigyn_ws \
    --workdir /home/ros/sigyn_ws \
    --user ros \
    sigynm4_base

xhost -local:docker 2>/dev/null || true
rm -f $XAUTH