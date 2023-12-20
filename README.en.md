<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT Follower

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#summary">Summary</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
    </li>
    <li>
    　<a href="#package-configuration">Package Configuration</a>
      <ul>
        <li><a href="#01-2d-lidar-person-detection">01. 2D Lidar Person Detection</a></li>
        <li><a href="#02-multiple-observation-kalman-filter">02. Multiple Observation Kalman Filter</a></li>
        <li><a href="#03-multiple-sensor-person-tracking">03. Multiple Sensor Person Tracking</a></li>
        <li><a href="#04-person-following-control">04. Person Following Control</a></li>
        <li><a href="#05-sobit-follower">05. SOBIT Follower</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#launch-configuration">Launch Configuration</a></li>
        <li><a href="#parameter-file">Parameter file</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <!-- <li><a href="#acknowledgments">Acknowledgments</a></li> -->
  </ol>
</details>

##Summary
- Robot person-following system with multiple sensors (usable with SOBIT EDU and SOBIT PRO)
<!-- - [論文](sobit_follower/doc/murakami_daiki_Master_research_summary.pdf) -->
- [Development of a person-following  robot using LRF and RGB-D sensor on the pan-tilt-rotate mechanism](https://www.jstage.jst.go.jp/article/jsmermd/2021/0/2021_1P2-G07/_article/-char/ja/)

<div align="center">
    <img src="sobit_follower/doc/img/system_overview.jpg" width="800">
    <img src="sobit_follower/doc/img/move_control.jpg" width="1080">
</div>

## Setup
```python
$ cd ~/catkin_ws/src/
$ git clone https://github.com/TeamSOBITS/sobit_follower
$ cd sobit_follower
# Install the necessary packages for follow me
$ bash install.sh
# Setup the installed package, then catkin_make
$ cd ~/catkin_ws
$ catkin_make
```

## Package Configuration
### 01. 2D Lidar Person Detection
- 2D Point Cloud Leg Detection with DR-SPAAM
- [GitHub：Person Detection in 2D Range Data](https://github.com/VisualComputingInstitute/2D_lidar_person_detection) modified to work with Python3
- Used in Multiple Sensor Person Tracking
- [For more information](2d_lidar_person_detection)

### 02. Multiple Observation Kalman Filter
- Kalman filter library with two observables as input
- Can also work with a single observation
- Equation of state is a constant velocity model
- Used in Multiple Sensor Person Tracking
- [For more information](multiple_observation_kalman_filter)

### 03. Multiple Sensor Person Tracking
- Person tracking using a 2D-LiDAR sensor combined with an RGB-D sensor on a pan-tilt rotation mechanism
- Person tracking using 2D point cloud leg detection with DR-SPAAM and person detection with SSD
- [For more information](multiple_sensor_person_tracking)

<!-- <div align="center">
    <img src="multiple_sensor_person_tracking/doc/img/tracker.png" width="1080">
</div> -->

### 04. Person Following Control
- Person-following control using a Virtual Spring Model with obstacle avoidance by Dynamic Window Approach
- [For more information](person_following_control)

<!-- <div align="center">
    <img src="person_following_control/doc/img/person_following_control.png" width="1080">
</div> -->

### 05. SOBIT Follower
- Person-following control using Multiple Sensor Person Tracking and Person Following Control
- Users can activate person-following by launching this package's Launch
- Shell scripts are also available to acquire rosbags for experiments and plot the acquired data
- [For more information](sobit_follower)

## Launch and Usage
### [sobit_edu_follower_me.launch](sobit_follower/launch/sobit_edu/sobit_edu_follower_me.launch)
- Person-following control by Multiple Sensor Person Tracking and Person Following Control using SOBIT EDU
- path：`sobit_follower/launch/sobit_edu/sobit_edu_follower_me.launch`
- [For more information](sobit_follower)
```python
$ roslaunch sobit_follower sobit_edu_follower_me.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
# Arguments
# rviz : whether to start Rviz (bool)
# rqt_reconfigure : whether to start rqt_reconfigure (bool)
# use_rotate : activate SensorRotator (bool)
# use_smoother : whether to perform velocity smoothing (bool)
```

### [sobit_pro_follower_me.launch](sobit_follower/launch/sobit_pro/sobit_pro_follower_me.launch)
- Person-following control by Multiple Sensor Person Tracking and Person Following Control using SOBIT PRO
- path：`sobit_follower/launch/sobit_pro/sobit_pro_follower_me.launch`
- [For more information](sobit_follower)
```python
$ roslaunch sobit_follower sobit_pro_follower_me.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
# Arguments
# rviz : whether to start Rviz (bool)
# rqt_reconfigure : whether to start rqt_reconfigure (bool)
# use_rotate : activate SensorRotator (bool)
# use_smoother : whether to perform velocity smoothing (bool)
```

#### Launch Configuration
- [ssd_pose_ros.launch.xml](sobit_follower/launch/include/ssd_pose_ros.launch.xml)
    - RGB image-based person detector
    - path：`sobit_follower/launch/include/ssd_pose_ros.launch.xml`
    - [For more information](sobit_follower#ssd_pose_roslaunchxml)
- [dr_spaam_ros.launch.xml](sobit_follower/launch/include/dr_spaam_ros.launch.xml)
    - 2D LiDAR-based person detector
    - path：`sobit_follower/launch/include/dr_spaam_ros.launch.xml`
    - [For more information](sobit_follower#dr_spaam_roslaunchxml)
- [sobit_edu_tracker.launch.xml](sobit_follower/launch/include/sobit_edu/sobit_edu_tracker.launch.xml)
    - 2D-LiDAR sensor with SOBIT EDU combined with RGB-D sensor on pan-tilt rotation mechanism for person tracking
    - path：`sobit_follower/launch/include/sobit_edu/sobit_edu_tracker.launch.xml`
    - [For more information](sobit_follower#sobit_edu_trackerlaunchxml)
- [sobit_pro_tracker.launch.xml](sobit_follower/launch/include/sobit_pro/sobit_pro_tracker.launch.xml)
    - 2D-LiDAR sensor with SOBIT PRO combined with RGB-D sensor on pan-tilt rotation mechanism for person tracking
    - path：`sobit_follower/launch/include/sobit_pro/sobit_pro_tracker.launch.xml`
    - [For more information](sobit_follower#sobit_pro_trackerlaunchxml)
- [sobit_edu_person_following_control.launch.xml](sobit_follower/launch/include/sobit_edu/sobit_edu_person_following_control.launch.xml)
    - Driving control that incorporates obstacle avoidance using the Dynamic Window Approach into tracking control using a Virtual Spring Model with SOBIT EDU
    - path：`sobit_follower/launch/include/sobit_edu/sobit_edu_person_following_control.launch.xml`
    - [For more information](sobit_follower#sobit_edu_person_following_controllaunchxml)
- [sobit_pro_person_following_control.launch.xml](sobit_follower/launch/include/sobit_pro/sobit_pro_person_following_control.launch.xml)
    - Driving control that incorporates obstacle avoidance using the Dynamic Window Approach into tracking control using a Virtual Spring Model with SOBIT PRO
    - path：`sobit_follower/launch/include/sobit_pro/sobit_pro_person_following_control.launch.xml`
    - [For more information](sobit_follower#sobit_pro_person_following_controllaunchxml)

#### Parameter file
- [tracker_param.yaml](sobit_follower/param/tracker_param.yaml)
    - Parameters for person tracking
    - path：`sobit_follower/param/tracker_param.launch.xml`
    - [For more information](sobit_follower#parametersperson_tracker)
- [ssd_param.yaml](sobit_follower/param/ssd_param.yaml)
    - RGB画像ベースの人物検出器に関するパラメータ
    - path：`sobit_follower/param/ssd_param.launch.xml`
    - [For more information](sobit_follower#parameters)
- [dr_spaam_param.yaml](sobit_follower/param/dr_spaam_param.yaml)
    - 2D LiDARベースの人物検出器に関するパラメータ
    - path：`sobit_follower/param/dr_spaam_param.launch.xml`
    - [For more information](sobit_follower#parameters-1)
- [sensor_rotator_param.yaml](sobit_follower/param/sensor_rotator_param.yaml)
    - RGB-Dセンサのパンチルト回転制御に関するパラメータ
    - path：`sobit_follower/param/sensor_rotator_param.launch.xml`
- [following_control_param.yaml](sobit_follower/param/following_control_param.yaml)
    - 走行制御に関するパラメータ
    - path：`sobit_follower/param/following_control_param.launch.xml`
    - [For more information](sobit_follower##parameterfollowing-control)
- [velocity_smoother_param.yaml](sobit_follower/param/velocity_smoother_param.yaml)
    - 速度平滑化に関するパラメータ
    - path：`sobit_follower/param/velocity_smoother_param.launch.xml`
    - [For more information](sobit_follower##velocity_smoother_param)

<!-- Milestone -->
## Milestone

- [x] OSS
    - [x] fix tf2 
    - [x] Improved documentation
    - [x] Unified coding style

現時点のバッグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

See the [open issues][license-url]  for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Acknowledgments -->
<!-- ## Acknowledgments

* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->
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