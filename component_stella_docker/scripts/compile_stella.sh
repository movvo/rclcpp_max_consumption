#!/bin/bash

cd /stella/build
CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}/lib/cmake cmake ..
make -j 16
make install