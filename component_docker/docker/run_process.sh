#!/bin/bash

docker run -it --rm \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    component_docker:latest \
    ros2 run component Component
