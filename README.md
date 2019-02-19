# setup
https://gitlab.com/TeamSOBITS/sobits-setup.git　←参照

# install sobit_follower 
cd ~/catkin_ws/src

git clone https://gitlab.com/TeamSOBITS/sobit_follower.git

# install turtlebot_edu
roscd sobit_follower

chmod 755 *

sudo ./install_turtlebot_edu.sh

roscd turtlebot_edu

chmod 755 *

sudo ./install.sh

# 人追従プログラムの実行コマンド

roslaunch turtlebot_edu minimal.launch

roslaunch sobit_follower sobit_follower_test.launch



