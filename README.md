# setup
https://gitlab.com/TeamSOBITS/sobits-setup.git
　←参照

## install sobit_follower 

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/sobit_follower.git
$ cd ~/catkin_ws/src/sobit_follower
$ chmod 755 *
$ cd ~/catkin_ws
$ catkin_make
```
## install sobit_education_bringup
### 基本的にsobit_education_bringupをインストールするだけで大丈夫です.(turtlebot_eduはしなくてok)
https://gitlab.com/TeamSOBITS/sobit_education_bringup.git
　←参照

## install turtlebot_edu

```bash
$ sudo ./install_turtlebot_edu.sh
$ cd ~/catkin_ws/src/turtlebot_edu
$ sudo ./install.sh
```

## 人追従プログラムの実行コマンド

```bash
$ roslaunch sobit_education_bringup minimal.launch
$ roslaunch sobit_follower sobit_follower_test_1.launch
```


