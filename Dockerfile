ARG ROS_CI_DESKTOP=xenial
ARG CI_SOURCE_PATH=/catkin_ws/src
ARG CI_DEVEL_PATH=/catkin_ws/devel
ARG ROS_DISTRO=kinetic
ARG ROSINSTALL_FILE=${CI_SOURCE_PATH}/dependencies.rosinstall
FROM ros:${ROS_DISTRO}-robot-${ROS_CI_DESKTOP}
ENV CI_SOURCE_PATH=${CI_SOURCE_PATH}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROSINSTALL_FILE=${ROSINSTALL_FILE}
RUN apt-get update -qq; \
    apt-get install -y ros-kinetic-xv-11-laser-driver ros-kinetic-robot-pose-ekf ros-kinetic-map-server ros-kinetic-base-local-planner ros-kinetic-costmap-2d ros-kinetic-amcl ros-kinetic-tf ros-kinetic-move-base; \
    apt-get install -y python-smbus python-pip socat python-pyaudio wget; \
    pip install pyserial AWSIoTPythonSDK PyCRC evdev; \
    wget https://cfhcable.dl.sourceforge.net/project/raspberry-gpio-python/RPi.GPIO-0.6.3.tar.gz && \
    tar -xvf RPi.GPIO-0.6.3.tar.gz && \
    cd RPi.GPIO-0.6.3 && \
    python setup.py install && \
    cd .. && \
    rm -rf RPi.GPIO-0.6.3