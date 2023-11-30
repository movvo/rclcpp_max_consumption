#!/bin/bash

echo $(dirname $(dirname $(realpath $0)))
docker run -it --rm -d\
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --net=host \
    --volume=$(dirname $(dirname $(realpath $0)))/stella_vslam/src:/stella/stella_vslam/src:rw \
    --volume=$(dirname $(dirname $(realpath $0)))/stella_vslam_ros:/ros2_ws/src/stella_vslam_ros:rw \
    --volume=$(dirname $(dirname $(realpath $0)))/inputs:/inputs:rw \
    --volume=$(dirname $(dirname $(realpath $0)))/scripts:/ros2_ws/scripts:rw \
    --name=stella_vslam-ros-socket \
    stella_vslam-ros-socket:latest \
    tail -f /dev/null

read -p 'In another terminal do: 
        docker exec -it stella_vslam-ros-socket /bin/bash
        source install/setup.bash && ros2 run rclcpp_components component_container_isolated
        and press [ENTER]'
    
docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link"
docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0"

#COPY THIS LINE AND EXECUTE INSIDE THE DOCKER
#docker exec -it -d stella_vslam-ros-socket /bin/bash -c ". /ros2_ws/install/setup.bash && ros2 component load /ComponentManager stella_vslam_ros stella_vslam_ros::System -p vocab_file_path:="/inputs/orb_vocab.fbow" -p setting_file_path:="/inputs/euroc_example.yaml" -p map_db_path_out:="/inputs/map.db" -r /camera/image_raw:=/cam0/image_raw" 

echo 'In another terminal do: 
      docker exec -it stella_vslam-ros-socket /bin/bash
      Copy the following lines inside the docker:
      source install/setup.bash && source install/setup.bash && ros2 component load /ComponentManager stella_vslam_ros stella_vslam_ros::System -p vocab_file_path:="/inputs/aist_living_lab_3/orb_vocab.fbow" -p setting_file_path:="/inputs/euroc_example.yaml" -p map_db_path_out:="/inputs/map.db"
      and press [ENTER]'
echo "Starting processes"

docker exec -it stella_vslam-ros-socket /bin/bash

read -p "Press [Enter] key to finish all processes and kill the docker"

docker kill stella_vslam-ros-socket