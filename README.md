# setup
https://gitlab.com/TeamSOBITS/sobits-setup.git
　←参照

## install sobit_follower 

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/sobit_follower.git
```

## install turtlebot_edu

```bash
$ cd ~/catkin_ws/src/sobit_follower
$ chmod 755 *
$ sudo ./install_turtlebot_edu.sh
$ cd ~/catkin_ws/src/turtlebot_edu
$ sudo ./install.sh
```

## 人追従プログラムの実行コマンド

```bash
$ roslaunch turtlebot_edu minimal.launch
$ roslaunch sobit_follower sobit_follower_test.launch
```


