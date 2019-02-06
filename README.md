# sobit_followerセットアップ手順

turtlebot_eduのインストール

./install_turtlebot_edu.sh

roscd turtlebot_edu

chmod 755 install.sh

sudo ./install.sh

#人追従プログラムの実行コマンド

roscd sobit_follower

chmod 755 *

roslaunch turtlebot_edu minimal.launch

roslaunch sobit_follower sobit_follower_test_2.launch



