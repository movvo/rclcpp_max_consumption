#!/bin/bash

docker run -it --rm \
    --volume=$(dirname $(dirname $(realpath $0)))/inputs:/inputs:rw \
    --env=RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --name stella_vslam-rosbag \
    stella_vslam-rosbag
