<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
    <a href="#概要">概要</a>
      <ul>
        <li><a href="#2d-lidarを用いた人物検出">2D-LiDARを用いた人物検出</a></li>
        <li><a href="#rgb-dセンサを用いた人物検出">RGB-Dセンサを用いた人物検出</a></li>
        <li><a href="#追跡">追跡</a></li>
        <li><a href="#パンチルト回転">パンチルト回転</a></li>
      </ul>
    </li>
    <li>
    <a href="#各パラメータについて">各パラメータについて</a>
      <ul>
        <li><a href="#node">Node</a></li>
        <li><a href="#subscriptions">Subscriptions</a></li>
        <li><a href="#publications">Publications</a></li>
        <li><a href="#parametersperson_tracker">Parameters(person_tracker)</a></li>
        <li><a href="#parameterssensor_rotator">Parameters(sensor_rotator)</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>

## 概要
- 2D-LiDARセンサとパンチルト回転機構上のRGB-Dセンサを組み合わせた人物追跡
- DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡

### 2D-LiDARを用いた人物検出
- DR-SPAAM（Distance Robust SPatial Attention and Auto-regressive Model）
    - 空間的注意と自己回帰モデルを用いた高速な2D LiDARベースの人物検出器
    - [GitHub：Person Detection in 2D Range Data](https://github.com/TeamSOBITS/2d_lidar_person_detection)
### RGB-Dセンサを用いた人物検出
- SSD（Single Shot Multibox Detector）
    - １度のCNN演算で物体の「領域候補検出」と「クラス分類」の両方を行う手法
    - Nodelet実装により物体検出処理が高速化
    - [GitHub：SSD Nodelet](https://github.com/TeamSOBITS/ssd_nodelet)

### 追跡
- Kalman Filter
    - 2D-LiDARからの脚の観測値とRGB-Dセンサからの人の観測値を入力とし、等速モデルによる状態方程式と組み合わせて、次の人の位置を推定することで追跡

### パンチルト回転
- パンチルト回転機構上のRGB-Dセンサを追跡対象の方向に回転

## 各パラメータについて
#### Node
|ノード名|意味|
|---|---|
|/multiple_sensor_person_tracking/tracker_nodelet_manager|追跡器のノードレットの管理|
|/multiple_sensor_person_tracking/person_tracker|DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡|
|/multiple_sensor_person_tracking/sensor_rotator|パンチルト回転機構上のRGB-Dセンサを追跡対象の方向に回転|

#### Subscriptions
|トピック名|型|意味|
|---|---|---|
|/dr_spaam_detections|multiple_sensor_person_tracking/LegPoseArray|検出結果|
|/ssd_object_detect/object_pose|sobits_msgs_msg/ObjectPoseArray|検出結果(３次元位置)|

#### Publications
|トピック名|型|意味|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|追従位置と障害物|
|/multiple_sensor_person_tracking/obstacles|sensor_msgs/PointCloud2|障害物点群|
|/multiple_sensor_person_tracking/tracker_marker|visualization_msgs/MarkerArray|検出結果のマーカ|

#### Parameters(person_tracker)
|パラメータ名|型|意味|
|---|---|---|
|/dr_spaam_topic_name|string|DR-SPAAMのトピック名|
|/ssd_topic_name|string|SSDのトピック名|
|/target_frame|string|基準フレーム名|
|/target_range|double|追跡対象を決定するときの最大範囲[m]|
|/leg_tracking_range|double|前フレームの推定値から人検出結果を観測値とする範囲(脚)[m]|
|/body_tracking_range|double|前フレームの推定値から人検出結果を観測値とする範囲(体)[m]|
|/target_cloud_radius|double|追跡位置から追跡点群とする半径[m]|
|/target_change_tolerance|double|追跡対象を変更する時間[s]|
|/process_noise|double|カルマンフィルタのプロセスノイズQ|
|/system_noise|double|カルマンフィルタのシステムノイズR|
|/outlier_radius|double|外れ値探索半径[m]|
|/outlier_min_pts|int|外れ値探索の最小点数|
|/leaf_size|double|ダウンサンプリング間隔[m]|
|/display_marker|bool|追跡Rvizマーカーを描画するか|

#### Parameters(sensor_rotator)
|パラメータ名|型|意味|
|---|---|---|
|/following_position_topic_name|string|following_positionのトピック名|
|/tilt_angle_min_deg|double|チルト方向の最小角度[deg]|
|/tilt_angle_max_deg|double|チルト方向の最大角度[deg]|
|/camera2person_height|double|カメラと人の頭の距離[m]|
|/use_rotate|bool|センサを回転させるか|
|/use_smoothing|bool|平滑化するか|
|/smoothing_gain|double|平滑化ゲイン|

<!-- マイルストーン -->
## マイルストーン

- [x] OSS
    - [x] tf2化 
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一

現時点のバッグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_follower.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_follower/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_follower.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_follower/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_follower.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_follower/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_follower.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_follower/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_follower.svg?style=for-the-badge
[license-url]: LICENSE
