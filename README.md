# rclcpp_max_consumption
Project that creates a simple process in ros2 that consumes a lot of CPU. This process can be loaded as a single process as many times as requested and can also be loaded as a rclcpp_component. It does the same but using stella_vslam repo to test the performance of multiple stella_vslam components running at the same time.

## Installation simple process
Build dockers using:
`./component_docker/build_docker.sh`

## Execution simple process
#### Eexecute multiple separated processes
`NUM_OF_PROCESSES` can be any number, the bash file will launch as many processes as specified.
`./component_docker/docker/run_processes.sh NUM_OF_PROCESSES`

#### Eexecute multiple processes as ros2 components
`NUM_OF_PROCESSES` can be any number, the bash file will launch as many processes as specified.
`./component_docker/docker/run_components.sh NUM_OF_PROCESSES`

## Installation stella_vslam process
Clone the submodules using: `git submodule update --recursive -i`

Build dockers using:
`./component_stella_docker/build_stella_docker.sh`
`./component_stella_docker/build_stella_rosbag.sh`
