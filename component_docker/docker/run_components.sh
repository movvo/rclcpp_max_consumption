#!/bin/bash

if [ $# -eq 0 ]
  then
    echo "usage is ./run_components <num_processes>"
    exit 1
fi

docker run -it --rm -d\
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --name=component_docker \
    component_docker:latest \
    ros2 run rclcpp_components component_container_mt

for (( c=1; c<=$1; c++ ))
do
  echo "Load process $i"
  docker exec -it -d component_docker /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 component load /ComponentManager component component::Component"
  sleep 0.8
done

sleep 1
echo "Starting processes"
docker exec -it -d component_docker /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 service call /component/change_state std_srvs/srv/Trigger"

read -p "Press [Enter] key to finish all processes and kill the docker"

docker kill component_docker
