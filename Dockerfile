ARG ROS_DISTRO=kinetic
ARG ROS_CI_DESKTOP=xenial
ARG CI_SOURCE_PATH=/catkin_ws/src
ARG CI_DEVEL_PATH=/catkin_ws/devel
ARG ROSINSTALL_FILE=$CI_SOURCE_PATH/dependencies.rosinstall
FROM ros:${ROS_DISTRO}-robot-$ROS_CI_DESKTOP
RUN apt-get update -qq; \
    apt-get install -y ros-kinetic-xv-11-laser-driver ros-kinetic-robot-pose-ekf ros-kinetic-map-server ros-kinetic-base-local-planner ros-kinetic-costmap-2d ros-kinetic-amcl ros-kinetic-tf ros-kinetic-move-base; \
    apt-get install -y python-smbus python-pip socat python-pyaudio; \
    pip install pyserial AWSIoTPythonSDK PyCRC evdev;
CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $CI_SOURCE_PATH && \
    catkin_init_workspace; \
    cd .. && \
    catkin_make && \
    export INSIDEDOCKER=true && \
    cd $CI_SOURCE_PATH && \
    source $CI_DEVEL_PATH/setup.bash && \
    roslaunch TROSnavigator.xml