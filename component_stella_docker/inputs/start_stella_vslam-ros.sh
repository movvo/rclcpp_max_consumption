#!/bin/bash

source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0 &
ros2 run stella_vslam_ros run_slam -v /inputs/orb_vocab.fbow -c /inputs/euroc_example.yaml --map-db-out /inputs/map.db --ros-args -r /camera/image_raw:=/cam0/image_raw