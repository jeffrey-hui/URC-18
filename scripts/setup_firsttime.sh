#!/bin/bash

# Setup ros, anything else will be added later.

echo "Installing deps"
scripts/deps/common.sh

echo "Setting up ROS workspace!"
source /opt/ros/kinetic/setup.bash
catkin_init_workspace rosws/src
cd rosws

echo Setting up device specific deps...
echo You need to choose which set of dependencies you want to install.
echo "You can install all of them, the bare ones required for running the robot, everything but the zed wrapper, and ones for simply remote controlling the robot."
read -p "All/Robot/Zed/rEmote [arze] " dep_set
wstool init src
cd src
case "${dep_set,,}" in
 "a") ../../scripts/deps/robot.sh y
      ../../scripts/deps/base_station.sh
      ;;
 "r") ../../scripts/deps/robot.sh y
      ;;
 "e") ../../scripts/deps/base_station.sh
      ;;
 "z")
      ../../scripts/deps/robot.sh n
      ../../scripts/deps/base_station.sh
      ;;
esac
wstool update
cd ..

echo "Doing a build to get all required libs and dirs"
cd ..
scripts/build.sh y
echo "Done!"



