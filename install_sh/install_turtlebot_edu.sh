#!/bin/sh

cd ~/catkin_ws/src/

git clone https://gitlab.com/TeamSOBITS/turtlebot_edu.git

echo "tutrle_edu install Finished"

cd turtlebot_edu/src/

chmod 755 *

cd ~/catkin_make

catkin_make

