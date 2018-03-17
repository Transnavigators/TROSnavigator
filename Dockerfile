FROM ros:kinetic-robot-xenial
ARG ROS_CI_DESKTOP=xenial
ARG CI_SOURCE_PATH=/catkin_ws/src
ARG CI_DEVEL_PATH=/catkin_ws/devel
ARG ROS_DISTRO=kinetic
ARG ROSINSTALL_FILE=${CI_SOURCE_PATH}/dependencies.rosinstall
ENV CI_SOURCE_PATH=${CI_SOURCE_PATH}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROSINSTALL_FILE=${ROSINSTALL_FILE}
RUN apt-get update -qq
RUN apt-get install -y ros-kinetic-xv-11-laser-driver ros-kinetic-robot-pose-ekf ros-kinetic-map-server ros-kinetic-base-local-planner ros-kinetic-costmap-2d ros-kinetic-amcl ros-kinetic-tf ros-kinetic-move-base
RUN apt-get install -y python-smbus python-pip socat python-pyaudio kmod
RUN pip install pyserial AWSIoTPythonSDK PyCRC evdev
WORKDIR ${CI_SOURCE_PATH}