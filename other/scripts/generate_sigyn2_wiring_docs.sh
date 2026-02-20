#!/bin/bash
pushd ~/sigyn_ws/src/Sigyn/Documentation/wiring2/Sigyn2
m4 <wireviz_sigyn2.yml >foo.yml; /home/ros/sigyn-venv/bin/wireviz --format hp -O sigyn2_wiring foo.yml
popd