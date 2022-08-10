# SOBIT Follower (version 5.0)
- 複数のセンサを用いたロボットの人追従走行システム

## Multiple Sensor Person Tracking
- 2D-LiDARセンサ(URG)とパンチルト回転機構上のRGB-Dセンサ(xtion)を組み合わせた人物追跡
- DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡
- 詳細は[こちら](multiple_sensor_person_tracking)

<div align="center">
    <img src="multiple_sensor_person_tracking/doc/img/tracker.png" width="1080">
</div>

## Person Following Control
- 仮想ばねモデルを用いた人間追従制御にDynamic Window Approachによる障害物回避を組み込んだ手法
- 詳細は[こちら](person_following_control)

<div align="center">
    <img src="person_following_control/doc/img/person_following_control.png" width="1080">
</div>

## How to Use
### sobit_follower.launch
- [sobit_follower.launch](sobit_follower/launch/sobit_follower.launch)
    - [tracker.launch.xml](sobit_follower/launch/include/tracker.launch.xml)
    - [person_following_control.launch.xml](sobit_follower/launch/include/person_following_control.launch.xml)
- Parameter file
    - [topic_params.yaml](sobit_follower/param/topic_params.yaml)
    - [tracking_params.yaml](sobit_follower/param/tracking_params.yaml)
    - [ssd_params.yaml](sobit_follower/param/ssd_params.yaml)
    - [kalman_filter_params.yaml](sobit_follower/param/kalman_filter_params.yaml)
    - [sensor_rotator_param.yaml](sobit_follower/param/sensor_rotator_param.yaml)
    - [following_control_param.yaml](sobit_follower/param/following_control_param.yaml)
```bash
$ roslaunch sobit_follower sobit_follower.launch
```
### simulator.launch
```bash
$ roslaunch sobit_follower simulator.launch
```
<div align="center">
    <img src="sobit_follower/doc/img/sobit_follower.png" width="1080">
</div>