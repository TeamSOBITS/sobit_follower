#!/bin/sh

cd ~/catkin_ws/src/

git clone https://gitlab.com/TeamSOBITS/turtlebot_edu.git

echo "turtlebot_edu install finished"

cd turtlebot_edu

chmod 755 *

cd ~/catkin_make

catkin_make

echo "catkin_make finished"

