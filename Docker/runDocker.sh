docker run \
    --platform linux/amd64 \
    -it \
    -v /dev/:/dev/ \
    -e "DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/snd \
    -e ALSADEV=hw:2,0 \
    --net=host \
    -v /home/ros/sigyn_ws:/home/ros/sigyn_ws \
    --workdir /home/ros/sigyn_ws \
    --user ros \
    rosjazzy

#docker run -it --device /dev/snd -e ALSADEV=hw:2,0 -v /dev/:/dev/ -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ros/sigyn_ws:/home/ros/sigyn_ws --workdir /home/ros/sigyn_ws  --user 1000:1000 macros