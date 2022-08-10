# SOBIT Follower (version 5.0)
- 複数のセンサを用いたロボットの人追従走行システム

## Multiple Sensor Person Tracking
- 2D-LiDARセンサ(URG)とパンチルト回転機構上のRGB-Dセンサ(xtion)を組み合わせた人物追跡
- DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡
- 詳細は[こちら](multiple_sensor_person_tracking)

<div align="center">
    <img src="multiple_sensor_person_tracking\doc\img\tracker.png" width="1080">
</div>

## Person Following Control
- 仮想ばねモデルを用いた人間追従制御にDynamic Window Approachによる障害物回避を組み込んだ手法
- 詳細は[こちら](person_following_control)

<div align="center">
    <img src="person_following_control\doc\img\person_following_control.png" width="1080">
</div>