FROM ros:humble
ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws
WORKDIR $WORKSPACE

ENV WORKSPACE $WORKSPACE

RUN apt update -qq && \
    apt upgrade -y -qq --no-install-recommends && \
    apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

COPY ./component /ros2_ws/src/component

RUN set -x && \
  : "build ROS2 packages" && cd /ros2_ws && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  source /ros2_ws/install/setup.bash; \
  colcon build --symlink-install"