#!/bin/bash

docker run -it --rm \
    --volume=$(dirname $(dirname $(realpath $0)))/component:/ros2_ws/src/component:rw \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    component_docker
