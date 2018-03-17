#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
cd /catkin_ws/src
rm CMakeLists.txt
catkin_init_workspace
cd ..
catkin_make
cd /catkin_ws/
source /catkin_ws/devel/setup.bash
catkin_make run_tests