#!/bin/bash
docker build -t stella_vslam-rosbag stella_vslam-rosbag -f stella_vslam_ros/Dockerfile.rosbag --build-arg NUM_THREADS=8 --no-cache