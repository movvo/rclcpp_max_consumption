#!/bin/bash

if [ $# -eq 0 ]
  then
    echo "usage is ./run_processes <num_processes>"
    exit 1
fi

docker run -it --rm -d\
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --volume=$(dirname $(dirname $(realpath $0)))/inputs:/inputs:rw \
    --name=stella_vslam-ros-socket \
    stella_vslam-ros-socket:latest \
    tail -f /dev/null

docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link"
docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0"

for (( c=1; c<=$1; c++ ))
do
  echo "Load process $i"
  docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 run stella_vslam_ros run_slam -v /inputs/orb_vocab.fbow -c /inputs/euroc_example.yaml --map-db-out /inputs/map.db --ros-args -r /camera/image_raw:=/cam0/image_raw"
  sleep 0.2
done

sleep 1
echo "Starting processes"
docker run -it --rm -d\
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --volume=$(dirname $(dirname $(realpath $0)))/inputs:/inputs:rw \
    --name=stella_vslam-rosbag \
    stella_vslam-rosbag:latest \
    /inputs/start_stella_vslam-rosbag.sh
# docker exec -it -d component_stella_docker /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 service call /component/change_state std_srvs/srv/Trigger"

read -p "Press [Enter] key to finish all processes and kill the docker"

docker kill stella_vslam-ros-socket
docker kill stella_vslam-rosbag
