#!/bin/bash
docker build -t stella_vslam-ros-socket stella_vslam_ros -f stella_vslam_ros/Dockerfile.socket --build-arg NUM_THREADS=8 --no-cache