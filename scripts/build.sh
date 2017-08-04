#!/usr/bin/env bash

if [ ! -d src ]; then
   echo "[!] Please run me from the rosws folder!"
   exit 1
fi

wstool update -t src
rosdep install --from-paths src --rosdistro kinetic -y -i
catkin_make
