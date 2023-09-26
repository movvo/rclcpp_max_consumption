#!/bin/bash

docker run -it --rm \
    --volume=$(dirname $(dirname $(realpath $0)))/component_stella:/ros2_ws/src/component_stella:rw \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    component_stella_docker
