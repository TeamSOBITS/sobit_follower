# Multiple Sensor Person Tracking
- 2D-LiDARセンサ(URG)とパンチルト回転機構上のRGB-Dセンサ(xtion)を組み合わせた人物追跡
- DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡

## software configuration
### Person detection by 2D-LiDAR
- DR-SPAAM（Distance Robust SPatial Attention and Auto-regressive Model）
    - 空間的注意と自己回帰モデルを用いた高速な2D LiDARベースの人物検出器
        - CNNベースの人物検出器である[DROW (Distance RObust Wheelchair/Walker)](https://arxiv.org/abs/1603.02636)を自動回帰モデルで拡張
            - [DROWの論文の日本語訳](doc/DROW_Japanese.pdf)
    - 2次元レンジデータに基づく人物検出において、速度と検出性能の両方で、従来の最先端技術を凌駕
    - [論文：DR-SPAAM: A Spatial-Attention and Auto-regressive Model for Person Detection in 2D Range Data](https://arxiv.org/abs/2004.14079)
        - [日本語訳](doc/DR-SPAAM_Japanese.pdf)
    - [GitHub：Person Detection in 2D Range Data](https://github.com/VisualComputingInstitute/2D_lidar_person_detection)
        - 本パッケージに，Python3で動作するように改良したものがあります（[2d_lidar_person_detection](2d_lidar_person_detection)）
        - そのディレクトリ内のREADMEは，本パッケージと関係ありません

### Person detection by RGB-D sensor
- SSD（Single Shot Multibox Detector）
    - １度のCNN演算で物体の「領域候補検出」と「クラス分類」の両方を行う手法
    - 物体検出処理の高速化を可能にした
    - [論文：SSD: Single Shot MultiBox Detector](https://arxiv.org/abs/1512.02325)
    - [GitHub：SSD Nodelet](https://gitlab.com/TeamSOBITS/ssd_nodelet)

### Tracking
- Kalman Filter
    - 2D-LiDARからの人の観測値とRGB-Dセンサからの人の観測値を入力とし、等速モデルによる状態方程式と組み合わせて、次の人の位置を推定することで追跡する．

### Pan Tilt Control
- パンチルト回転機構上のRGB-Dセンサを追跡対象の方向に回転させる

## Setup
```bash
$ cd ~/catkin_ws/src/
$ git clone https://gitlab.com/TeamSOBITS/multiple_sensor_person_tracking.git
$ cd multiple_sensor_person_tracking
$ bash install.sh
```

## How to Use
### DR-SPAAM ROS
DR-SPAAMによる人検出を行う
```bash
$ roslaunch multiple_sensor_person_tracking dr_spaam_ros.launch
# scan2d_data_publisher.launchを使うことで，擬似的にスキャンデータをパブリッシュすることができる
$ roslaunch multiple_sensor_person_tracking scan2d_data_publisher.launch
```
<div align="center">
    <img src="doc/img/dr_spaam_ros.png" width="1080">
</div>

### SSD ROS
SSDによる人検出を行う
```bash
$ roslaunch multiple_sensor_person_tracking ssd_ros.launch
# camera_720p_16_9.launchを使うことで，Webカメラから画像がパブリッシュされる
$ roslaunch multiple_sensor_person_tracking camera_720p_16_9.launch
```
<div align="center">
    <img src="doc/img/ssd_nodelet.png" width="1080">
</div>

### Multiple Sensor Person Tracking
```bash
$ roslaunch multiple_sensor_person_tracking tracker.launch
# sensor_data_publisher.launchを使うことで，擬似的にスキャンデータをパブリッシュすることができる
$ roslaunch virtual_multiple_sensor_publisher sensor_data_publisher.launch
```
<div align="center">
    <img src="doc/img/tracker.png" width="1080">
</div>