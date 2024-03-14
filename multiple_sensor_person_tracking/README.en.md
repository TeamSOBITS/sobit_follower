<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
    <a href="#summary">Summary</a>
      <ul>
        <li><a href="#person-detection-using-2d-lidar">Person detection using 2D Lidar</a></li>
        <li><a href="#person-detection-using-rgb-d-sensor">Person detection using RGB-D sensor</a></li>
        <li><a href="#tracking">Tracking</a></li>
        <li><a href="#pan-tilt">Pan Tilt</a></li>
      </ul>
    </li>
    <li>
    <a href="#about-each-parameter">About each parameter</a>
      <ul>
        <li><a href="#node">Node</a></li>
        <li><a href="#subscriptions">Subscriptions</a></li>
        <li><a href="#publications">Publications</a></li>
        <li><a href="#parametersperson_tracker">Parameters(person_tracker)</a></li>
        <li><a href="#parameterssensor_rotator">Parameters(sensor_rotator)</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>

## Summary
- Person tracking using 2D-LiDAR sensor combined with RGB-D sensor on pan-tilt rotation mechanism
- Person tracking using 2D point cloud leg detection with DR-SPAAM and image person detection with SSD

### Person detection using 2D Lidar
- DR-SPAAM（Distance Robust SPatial Attention and Auto-regressive Model）
    - Fast 2D LiDAR-based person detector with spatial attention and autoregressive modeling
    - [GitHub：Person Detection in 2D Range Data](https://github.com/TeamSOBITS/2d_lidar_person_detection)
### Person detection using RGB-D sensor
- SSD（Single Shot Multibox Detector）
    - A method that performs both "region candidate detection" and "class classification" of objects in a single CNN operation
    - Nodelet implementation speeds up object detection process
    - [GitHub：SSD Nodelet](https://github.com/TeamSOBITS/ssd_nodelet)

### Tracking
- Kalman Filter
    - Tracking by taking leg observations from 2D-LiDAR and person observations from RGB-D sensors as input and combining them with the equation of state from a constant velocity model to estimate the next person's position

### Pan Tilt
- Rotate the RGB-D sensor on the pan-tilt rotary mechanism in the direction of the target to be tracked

## About each parameter
#### Node
|Node Name|Meaning|
|---|---|
|/multiple_sensor_person_tracking/tracker_nodelet_manager|Tracker nodelet management|
|/multiple_sensor_person_tracking/person_tracker|Person tracking using 2D point cloud leg detection with DR-SPAAM and image person detection with SSD|
|/multiple_sensor_person_tracking/sensor_rotator|Rotate the RGB-D sensor on the pan-tilt rotary mechanism in the direction of the target to be tracked|

#### Subscriptions
|Topic Name|Type|Meaning|
|---|---|---|
|/dr_spaam_detections|multiple_sensor_person_tracking/LegPoseArray|Detection results|
|/ssd_object_detect/object_pose|sobits_msgs_msg/ObjectPoseArray|Detection result(3D position)|

#### Publications
|Topic Name|Type|Meaning|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|追従位置と障害物|
|/multiple_sensor_person_tracking/obstacles|sensor_msgs/PointCloud2|障害物点群|
|/multiple_sensor_person_tracking/tracker_marker|visualization_msgs/MarkerArray|検出結果のマーカ|

#### Parameters(person_tracker)
|Parameter Name|Type|Meaning|
|---|---|---|
|/dr_spaam_topic_name|string|DR-SPAAM Topic Name|
|/ssd_topic_name|string|SSD Topic Name|
|/target_frame|string|Base Frame Name |
|/target_range|double|Maximum range when determining the tracking target[m]|
|/leg_tracking_range|double|Range of human detection results from the previous frame's estimate to the observed value (leg)[m]|
|/body_tracking_range|double|Range of human detection results from the previous frame's estimate to the observed value (body)[m]|
|/target_cloud_radius|double|Radius from tracking location to tracking point cloud[m]|
|/target_change_tolerance|double|Time to change tracking target[s]|
|/process_noise|double|Kalman filter process noise Q|
|/system_noise|double|Kalman filter system noise R|
|/outlier_radius|double|outlier search radius[m]|
|/outlier_min_pts|int|Minimum number of points for outlier search|
|/leaf_size|double|Down-sampling interval[m]|
|/display_marker|bool|Whether to draw tracking rviz markers|

#### Parameters(sensor_rotator)
|Parameter Name|Type|Meaning|
|---|---|---|
|/following_position_topic_name|string|Topic name for following_position|
|/tilt_angle_min_deg|double|Minimum angle in tilt direction[deg]|
|/tilt_angle_max_deg|double|Maximum angle in tilt direction[deg]|
|/camera2person_height|double|Distance between camera and human head[m]|
|/use_rotate|bool|Whether to rotate the sensor|
|/use_smoothing|bool|Whether to smooth out|
|/smoothing_gain|double|Smoothening gain|

<!-- マイルストーン -->
## Milestone

- [x] OSS
    - [x] fix tf2 
    - [x] Improved documentation
    - [x] Unified coding style

See the open issues for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

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
