<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT Follower

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <ul>
          <li><a href="#セットアップ">セットアップ</a></li>
          <li><a href="#対象者識別用の追加セットアップ">対象者識別用の追加セットアップ</a></li>
      </ul>
    </li>
    <li>
    　<a href="#パッケージ構成">パッケージ構成</a>
      <ul>
        <li><a href="#01-2d-lidar-person-detection">01. 2D Lidar Person Detection</a></li>
        <li><a href="#02-multiple-observation-kalman-filter">02. Multiple Observation Kalman Filter</a></li>
        <li><a href="#03-multiple-sensor-person-tracking">03. Multiple Sensor Person Tracking</a></li>
        <li><a href="#04-person-following-control">04. Person Following Control</a></li>
        <li><a href="#05-sobit-follower">05. SOBIT Follower</a></li>
        <li><a href="#06-target-identification-method">06. Target Identification Method</a></li>
      </ul>
    </li>
    <li>
    　<a href="#実行方法">実行方法</a>
      <ul>
        <li><a href="#launch構成">Launch構成</a></li>
        <li><a href="#parameterファイル">Parameterファイル</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>

## 概要
- 複数のセンサを用いたロボットの人追従走行システム
- [LRFとパン・チルト回転機構上のRGB-Dセンサを用いた人追従走行ロボットの開発](https://www.jstage.jst.go.jp/article/jsmermd/2021/0/2021_1P2-G07/_article/-char/ja/)

<div align="center">
    <img src="sobit_follower/doc/img/system_overview.jpg" width="800">
    <img src="sobit_follower/doc/img/move_control.jpg" width="1080">
</div>

## セットアップ
```python
$ cd ~/catkin_ws/src/
$ git clone -b humble-devel https://github.com/TeamSOBITS/sobit_follower
$ cd sobit_follower
# sobit_followerに必要なパッケージのインストールを行う
$ bash install.sh
# インストールしたパッケージのセットアップを行った後、colcon build
$ cd ~/colcon_ws/
$ colcon build --symlink-install
$ source ~/colcon_ws/install/setup.sh
```

## 対象者識別用の追加セットアップ
```python
$ cd sobit_follower
# 対象者識別に必要な追加パッケージのインストールを行う
$ bash install_target_identification.sh
# インストールしたパッケージのセットアップを行った後、colcon build
$ cd ~/colcon_ws/
$ colcon build --symlink-install
$ source ~/colcon_ws/install/setup.sh
```

## パッケージ構成
### 01. 2D Lidar Person Detection
- DR-SPAAMによる2次元点群脚検出
- [GitHub：Person Detection in 2D Range Data](https://github.com/VisualComputingInstitute/2D_lidar_person_detection)をPython3で動作するように改良したもの
- Multiple Sensor Person Trackingで使用
- 詳細は[こちら](2d_lidar_person_detection)

### 02. Multiple Observation Kalman Filter
- 2つの観測値を入力とするカルマンフィルタライブラリ
- 1つの観測値でも動作可能
-　状態方程式は等速モデル
- Multiple Sensor Person Trackingで使用
- 詳細は[こちら](multiple_observation_kalman_filter)

### 03. Multiple Sensor Person Tracking
- 2D-LiDARセンサとパンチルト回転機構上のRGB-Dセンサを組み合わせた人物追跡
- DR-SPAAMによる2次元点群脚検出とSSDによる画像人検出を用いた人物追跡
- 詳細は[こちら](multiple_sensor_person_tracking)

<!-- <div align="center">
    <img src="multiple_sensor_person_tracking/doc/img/tracker.png" width="1080">
</div> -->

### 04. Person Following Control
- 仮想ばねモデルを用いた人間追従制御にDynamic Window Approachによる障害物回避を組み込んだ走行制御
- 詳細は[こちら](person_following_control)

<!-- <div align="center">
    <img src="person_following_control/doc/img/person_following_control.png" width="1080">
</div> -->

### 05. SOBIT Follower
- Multiple Sensor Person TrackingとPerson Following Controlを用いた人追従走行
- ユーザはこのパッケージのLaunchを起動することで人追従走行を動作させることが可能
- 実験用のrosbag取得や取得したデータのplotも可能なシェルスクリプトも完備
- 詳細は[こちら](sobit_follower)

### 06. Target Identification Method
- 対象者識別手法を加えた人追従走行
- 以下に示す2つの対象者識別手法のどちらかを人追従走行ロボットシステムに追加することでロボットは対象者を識別しながら人追従走行が可能

対象者識別手法
1. 畳み込みチャネル機能とオンラインブースティングの組み合わせ(Koide_Model)
    - 10個の特徴マップに基づくランダムな矩形の画素値和による人物特徴抽出とオンラインブースティングによる対象者分類器から構成される
    - 追加が必要なパッケージは[monocular_person_following](https://github.com/TeamSOBITS/monocular_person_following)と[ccf_person_identification](https://github.com/TeamSOBITS/ccf_person_identification)であり，
    この2つのパッケージに依存して[open_face_recognition](https://github.com/TeamSOBITS/open_face_recognition)も追加する必要がある
    - 論文：
    - Kenji Koide, Jun Miura, and Emanuele Menegatti， “Monocular person tracking and identification with on-line deep feature selection for person following robots”，Robotics and Autonomous Systems，124: 103348，2020 [[link]](https://staff.aist.go.jp/k.koide/assets/pdf/ias15_ext.pdf).

2. OSNetとリッジ回帰モデルの組み合わせ(GRR_SLT)
    - OSNetによる人物特徴抽出とリッジ回帰モデルによる対象者分類器から構成される
    - 追加が必要なパッケージは[MPF_GRR_SLT](https://github.com/TeamSOBITS/MPF_GRR_SLT)である
        - MPF_GRR_SLTの元のパッケージでは人物特徴抽出手法にOSNetは用いられていないが，sobit_follwerに対象者識別機能を適用するにあたって人物特徴抽出手法としてOSNetを代わりに使用
    - OSNetのgithub： [[link]](https://github.com/KaiyangZhou/deep-person-reid)
    - OSNetの論文：
    - Kaiyang Zhou，Yongxin Yang，Andrea Cavallaro and Tao Xiang，“Omni-scale feature learning for person re-identification”，Proceedings of the IEEE/CVF international conference on computer vision，pp.3702-3712，2019  [[link]](https://openaccess.thecvf.com/content_ICCV_2019/papers/Zhou_Omni-Scale_Feature_Learning_for_Person_Re-Identification_ICCV_2019_paper.pdf).
    - MPF_GRR_SLT(この対象者識別手法の基盤となるパッケージ(リッジ回帰モデルによる対象者分類器はオリジナルのまま使用))の論文：
    - Hanjing Ye，Jieting Zhao，Yaling Pan，Weinan Chen and Hong Zhang，“Following Closely: A Robust Monocular Person Following System for Mobile Robot”，arXiv preprint arXiv:2204.10540，2022 [[link]](https://arxiv.org/pdf/2204.10540).

- 対象者識別実験からKoide_ModelよりGRR_SLTの方が対象者識別精度が高いことが示されている
- 一方でKoide_Modelの方はGRR_SLTより処理量が軽いという利点があるため使用する状況に応じて2つの対象者識別手法を上手く使い分けることをおすすめする
- 上に示す対象者識別用の追加セットアップを行うことで使用可能

## 実行方法
### [sobit_follower.launch](sobit_follower/launch/sobit_follower.launch)
- Multiple Sensor Person TrackingとPerson Following Controlによる人追従走行
- path：`sobit_follower/launch/sobit_follower.launch`
- 詳細は[こちら](sobit_follower)
```python
$ ros2 launch sobit_follower sobit_follower.launch.py
```

> [!IMPORTANT]
> 使用しているロボットに合わせて、`sobit_follower/launch/sobit_follower.launch` 内の `robot_type` を変更してください。

> [!CAUTION]
> 以下のエラーが発生した場合は、[この Google Drive](https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W) から重みファイルをダウンロードし、`sobit_follower/dr_spaam_ros/weights/` に配置したうえで、パッケージを再ビルドしてください。  
> `FileNotFoundError: [Errno 2] No such file or directory: '/home/username/colcon_ws/install/dr_spaam_ros/share/dr_spaam_ros/weights/ckpt_jrdb_ann_ft_dr_spaam_e20.pth'`

<!-- 
### [sobit_edu_follower_me_GRRSLT.launch](sobit_follower/launch/sobit_edu/sobit_edu_follower_me_GRRSLT.launch)
- SOBIT_EDUでOSNetとリッジ回帰モデルからなる(GRR_SLT)対象者識別手法を組み合わせて追従対象者を識別することを可能とした人追従走行
- path：`sobit_follower/launch/sobit_edu/sobit_edu_follower_me_GRRSLT.launch`
- ここでは人物検出としてSSDの代わりにYOLOv10を用いている
```python
$ roslaunch sobit_follower sobit_edu_follower_me_GRRSLT.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
```

### [sobit_edu_follower_me_KoideModel.launch](sobit_follower/launch/sobit_edu/sobit_edu_follower_me_KoideModel.launch)
- SOBIT_EDUで対象者識別手法(KoideModel)を組み合わせて追従対象者を識別することを可能とした人追従走行
- path：`sobit_follower/launch/sobit_edu/sobit_edu_follower_me_KoideModel.launch`
- ここでは人物検出としてSSDの代わりにYOLOv10を用いている
```python
$ roslaunch sobit_follower sobit_edu_follower_me_KoideModel.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
``` -->

#### Launch構成
- [ssd_pose_ros.launch.py](sobit_follower/launch/include/ssd_pose_ros.launch.py)
    - RGB画像ベースの人物検出器
    - path：`sobit_follower/launch/include/ssd_pose_ros.launch.py`
    - 詳細は[こちら](sobit_follower#ssd_pose_roslaunchxml)
- [dr_spaam_ros.launch.py](sobit_follower/launch/include/dr_spaam_ros.launch.py)
    - 2D LiDARベースの人物検出器
    - path：`sobit_follower/launch/include/dr_spaam_ros.launch.py`
    - 詳細は[こちら](sobit_follower#dr_spaam_roslaunchxml)
- [person_id.launch.xml](sobit_follower/launch/include/dr_spaam_ros.launch.xml)
    - RGB-Dセンサを用いた対象者同定手法
    - path：`sobit_follower/launch/include/person_id.launch.xml`
    - 詳細は[こちら](sobit_follower#peson_idlaunchxml)

#### Parameterファイル
- [tracker_param.yaml](sobit_follower/param/tracker_param.yaml)
    - 人物追跡に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/tracker_param.yaml`
    - パラメータの詳細は[こちら](sobit_follower#parametersperson_tracker)
- [ssd_param.yaml](sobit_follower/param/ssd_param.yaml)
    - RGB画像ベースの人物検出器に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/ssd_param.yaml`
    - パラメータの詳細は[こちら](sobit_follower#parameters)
- [dr_spaam_param.yaml](sobit_follower/param/dr_spaam_param.yaml)
    - 2D LiDARベースの人物検出器に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/dr_spaam_param.yaml`
    - パラメータの詳細は[こちら](sobit_follower#parameters-1)
- [sensor_rotator_param.yaml](sobit_follower/param/sensor_rotator_param.yaml)
    - RGB-Dセンサのパンチルト回転制御に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/sensor_rotator_param.yaml`
- [following_control_param.yaml](sobit_follower/param/following_control_param.yaml)
    - 走行制御に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/following_control_param.yaml`
    - パラメータの詳細は[こちら](sobit_follower##parameterfollowing-control)
- [velocity_smoother_param.yaml](sobit_follower/param/velocity_smoother_param.yaml)
    - 速度平滑化に関するパラメータ
    - path：`sobit_follower/param/<robot_type>/velocity_smoother_param.yaml`
    - パラメータの詳細は[こちら](sobit_follower##velocity_smoother_param)

<!-- マイルストーン -->
## マイルストーン

- [x] OSS
    - [x] tf2化 
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一

現時点のバッグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

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