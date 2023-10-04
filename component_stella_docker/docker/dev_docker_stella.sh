#!/bin/bash

docker run -it --rm \
    --volume=$(dirname $(dirname $(realpath $0)))/stella_vslam_ros:/ros2_ws/src/stella_vslam_ros:rw \
    --volume=$(dirname $(dirname $(realpath $0)))/dataset_publisher_ros2:/ros2_ws/src/dataset_publisher_ros2:rw \
    --volume=$(dirname $(dirname $(realpath $0)))/inputs:/inputs:rw \
    --env=RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --name stella_vslam-ros-socket \
    stella_vslam-ros-socket
