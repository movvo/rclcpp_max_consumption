FROM ros:humble
ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws
WORKDIR $WORKSPACE

ENV WORKSPACE $WORKSPACE

RUN apt update -qq && \
    apt upgrade -y -qq --no-install-recommends && \
    apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

COPY ./component /ros2_ws/src/component

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install

# source overlay from entrypoint
ENV WORKSPACE $WORKSPACE
RUN sed --in-place \
      's|^source .*|source "$WORKSPACE/install/setup.bash"|' \
      /ros_entrypoint.sh