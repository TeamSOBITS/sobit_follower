<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT Follower
---
<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#summary">Summary</a>
    </li>
    <li>
      <a href="#launch-and-usage">Launch and Usage</a>
    </li>
    <li>
      <a href="#configuration">Configuration</a>
      <ul>
        <li><a href="#trackerlaunchxml">tracker.launch.xml</a></li>
        <li><a href="#ssd_pose_roslaunchxml">ssd_pose_ros.launch.xml</a></li>
        <li><a href="#dr_spaam_roslaunchxml">dr_spaam_ros.launch.xml</a></li>
        <li><a href="#person_following_controllaunchxml">person_following_control.launch.xml</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
  </ol>
</details>

---
## Summary
- Multiple Sensor Person Tracking and Person Following Control
- Users can activate person-following by launching this package.
- Shell scripts are also provided to acquire rosbags for experiments and to plot the acquired data.

<div align="center">
    <img src="doc/img/system_overview.jpg" width="800">
    <img src="doc/img/move_control.jpg" width="1080">
</div>

## Launch and Usage
### [sobit_edu_follower_me.launch](launch/sobit_edu/sobit_edu_follower_me.launch)
- Person-following driving using SOBIT EDU
- path：`sobit_follower/launch/sobit_edu/sobit_edu_follower_me.launch`
```python
$ roslaunch sobit_follower sobit_edu_follower_me.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
# Arguments
# rviz : whether to start Rviz (bool)
# rqt_reconfigure : whether to start rqt_reconfigure (bool)
# use_rotate : activate SensorRotator (bool)
# use_smoother : whether to perform velocity smoothing (bool)
```
※To activate SOBIT EDU, RGB-D sensor, and 2D LiDAR

### [sobit_pro_follower_me.launch](launch/sobit_pro/sobit_pro_follower_me.launch)
- Person-following driving using SOBIT PRO
- path：`sobit_follower/launch/sobit_pro/sobit_pro_follower_me.launch`
```python
$ roslaunch sobit_follower sobit_pro_follower_me.launch rviz:=false rqt_reconfigure:=false use_rotate:=true use_smoother:=true
# Arguments
# rviz : whether to start Rviz (bool)
# rqt_reconfigure : whether to start rqt_reconfigure (bool)
# use_rotate : activate SensorRotator (bool)
# use_smoother : whether to perform velocity smoothing (bool)
```
※To activate SOBIT PRO, RGB-D sensor, and 2D LiDAR


## Configuration
### tracker.launch.xml
- #### [sobit_edu_tracker.launch.xml](launch/include/sobit_edu/sobit_edu_tracker.launch.xml)
- #### [sobit_pro_tracker.launch.xml](launch/include/sobit_pro/sobit_pro_tracker.launch.xml)
#### Node
|Node|Meaning|
|---|---|
|/multiple_sensor_person_tracking/tracker_nodelet_manager|Tracker nodelet management|
|/multiple_sensor_person_tracking/person_tracker|Person tracking using 2D point cloud leg detection with DR-SPAAM and image person detection with SSD|
|/multiple_sensor_person_tracking/sensor_rotator|Rotate the RGB-D sensor on the pan-tilt rotary mechanism in the direction of the target to be tracked|

#### Subscriptions
|Topic|Type|Meaning|
|---|---|---|
|/dr_spaam_detections|multiple_sensor_person_tracking/LegPoseArray|Detection results|
|/ssd_object_detect/object_pose|sobits_msgs/ObjectPoseArray|Detection results(3D Position)|

#### Publications
|Topic|Type|Meaning|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|Tracking position and obstacles|
|/multiple_sensor_person_tracking/obstacles|sensor_msgs/PointCloud2|obstacle point cloud|
|/multiple_sensor_person_tracking/tracker_marker|visualization_msgs/MarkerArray|Marker for detection results|

#### Parameters(person_tracker)
|Parameter|Type|Meaning|
|---|---|---|
|/dr_spaam_topic_name|string|DR-SPAAM Topic Name|
|/ssd_topic_name|string|SSD Topic Name|
|/target_frame|string|Reference Frame Name|
|/target_range|double|Maximum range when determining the tracking target[m]|
|/leg_tracking_range|double|Range from the previous frame's estimate to the observed value of the person detection result(leg)[m]|
|/body_tracking_range|double|Range from the previous frame's estimate to the observed value of the person detection result(body)[m]|
|/target_cloud_radius|double|Radius from tracking location to tracking point cloud[m]|
|/target_change_tolerance|double|Time to change tracking target[s]|
|/process_noise|double|Kalman filter process noise Q|
|/system_noise|double|The system noise of the Kalman filter R|
|/outlier_radius|double|Outlier search radius[m]|
|/outlier_min_pts|int|Minimum number of points for outlier search|
|/leaf_size|double|Downsampling interval[m]|
|/display_marker|bool|Whether to draw tracking Rviz markers|

#### Parameters(sensor_rotator)
|Parameter|Type|Meaning|
|---|---|---|
|/following_position_topic_name|string|Topic name for following_position|
|/tilt_angle_min_deg|double|Minimum angle in tilt direction[deg]|
|/tilt_angle_max_deg|double|Maximum angle in tilt direction[deg]|
|/camera2person_height|double|Distance between camera and human head[m]|
|/use_rotate|bool|Whether to rotate the sensor|
|/use_smoothing|bool|Whether to smooth out|
|/smoothing_gain|double|smoothing gain|

#### Parameter file
- [tracker_param.yaml](param/tracker_param.yaml)
    - Parameters for person tracking
    - path：`sobit_follower/param/tracker_param.launch.xml`
- [sensor_rotator_param.yaml](param/sensor_rotator_param.yaml)
    - Parameters for pan-tilt rotation control of RGB-D sensor
    - path：`sobit_follower/param/sensor_rotator_param.launch.xml`

### ssd_pose_ros.launch.xml
- #### [ssd_pose_ros.launch.xml](launch/include/ssd_pose_ros.launch.xml)
  - RGB image-based person detector
  - path：`sobit_follower/launch/include/ssd_pose_ros.launch.xml`

#### Node
|Node|Meaning|
|---|---|
|/ssd_object_detect/ssd_nodelet_manager|Management of SSD nodelets|
|/ssd_object_detect/ssd_nodelet|Person detection by SSD|

#### Subscriptions
|Topic|Type|Meaning|
|---|---|---|
|/camera/depth/points|sensor_msgs/PointCloud2|input point cloud|
|/camera/camera/rgb/image_raw|sensor_msgs/Image|Input image|
|/ssd_object_detect/detect_ctrl|std_msgs/Bool|Detection on/off|

#### Publications
|Topic|Type|Meaning|
|---|---|---|
|/ssd_object_detect/detect_result|sensor_msgs/Image|Detection results(Image)|
|/ssd_object_detect/object_name|sobits_msgs/StringArray|Detection results(Object Name)|
|/ssd_object_detect/object_rect|sobits_msgs/BoundingBoxes|Detection results(bounding box)|
|/ssd_object_detect/object_pose|sobits_msgs/ObjectPoseArray|Detection results(3D Position)|

#### Parameters
|Parameter|Type|Meaning|
|---|---|---|
|/ssd_prototxt_name|string|prototxt file path|
|/ssd_caffemodel_name|string|caffemodel file path|
|/ssd_class_names_file|string|names file path|
|/ssd_image_topic_name|string|Topic name of input image|
|/ssd_cloud_topic_name|string|Topic name of input point cloud|
|/target_frame|string|Base Frame Name|
|/ssd_in_scale_factor|double|Scale parameter when converting Blob format handled by Caffe|
|/ssd_confidence_threshold|double|Confidence threshold for results to be added to the detection results list|
|/object_specified_enabled|bool|Whether to perform detection only on specific objects|
|/specified_object_name|string|Specific object name to be detected (names not in the object label will be rejected)|
|/ssd_img_show_flag|bool|Whether to draw the detection image|
|/ssd_execute_default|bool|Whether to initiate detection at startup|
|/ssd_pub_result_image|bool|Whether to publish the detect_result|
|/use_tf|bool|Whether to register coordinates by tf|

#### Parameter file
- [ssd_param.yaml](param/ssd_param.yaml)
    - Parameters for RGB image-based person detector
    - path：`sobit_follower/param/ssd_param.launch.xml`

### dr_spaam_ros.launch.xml
- #### [dr_spaam_ros.launch.xml](launch/include/dr_spaam_ros.launch.xml)
  - 2D LiDAR-based person detector
  - path：`sobit_follower/launch/include/dr_spaam_ros.launch.xml`

#### Node
|Node|Meaning|
|---|---|
|/dr_spaam/dr_spaam_ros|Person detection by DR-SPAAM|

#### Subscriptions
|Topic|Type|Meaning|
|---|---|---|
|/scan|sensor_msgs/LaserScan|Input scan|

#### Publications
|Topic|Type|Meaning|
|---|---|---|
|/dr_spaam_detections|multiple_sensor_person_tracking/LegPoseArray|Detection result|

※For demonstration purposes, it will be "geometry_msgs/PoseArray

#### Parameters
|Parameter|Type|Meaning|
|---|---|---|
|/detector_model|string|Model name(DROW3 or DR-SPAAM)|
|/weight_file|string|Weight file path|
|/conf_thresh|double|Confidence threshold|
|/stride|int|Stride Interval|
|/panoramic_scan|bool|Whether the scan covers 360 degrees|

#### Parameter file
- [dr_spaam_param.yaml](param/dr_spaam_param.yaml)
    - Parameters related to 2D LiDAR-based person detector
    - path：`sobit_follower/param/dr_spaam_param.launch.xml`

### person_following_control.launch.xml
- #### [sobit_edu_person_following_control.launch.xml](launch/include/sobit_edu/sobit_edu_person_following_control.launch.xml)
- #### [sobit_pro_person_following_control.launch.xml](launch/include/sobit_pro/sobit_pro_person_following_control.launch.xml)
  - Person-following control using a virtual spring model with obstacle avoidance by Dynamic Window Approach
  - path：`sobit_follower/launch/include/person_following_control.launch.xml`

#### Node
|Node|Meaning|
|---|---|
|/person_following_control/following_control_nodelet_manager|Management of nodelets for tracking control|
|/person_following_control/person_following_control|Tracking Control|

#### Subscriptions
|Topic|Type|Meaning|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|Tracking position and obstacles|

#### Publications
|Topic|Type|Meaning|
|---|---|---|
|/cmd_vel_mux/input/teleop|multiple_sensor_person_tracking/FollowingPosition|Speed information for tracking control|

#### Parameter：Following Control
|Parameter|Type|Meaning|
|---|---|---|
|/base_footprint_name|string|Base Frame Name|
|/following_distance|double|Distance to keep with people ( l_0 [m] )|
|/following_method|int|Following method|

##### following_method
```
0 : VirtualSpringModel-DynamicWindowApproach
1 : VirtualSpringModel
2 : DynamicWindowApproach
3 : PIDController
```

#### Parameter：Virtual Spring Model
|Parameter|Type|Meaning|
|---|---|---|
|/following_angle_deg|double|Angle of tracking position ( theta_sh [deg] )|
|/spring_constant_linear|double|spring constant(translational component) ( k_1 [N/m] )|
|/spring_constant_angular|double|spring constant(rotational component) ( k_2 [N・m/rad] )|
|/weight_robot|double|Robot weight ( M [Kg] )|
|/moment_inertia|double|moment of inertia ( I [Kg・m^2] )|
|/viscous_friction_linear|double|viscous friction(translational component) ( k_3 [N・s/m] )|
|/viscous_friction_angular|double|viscous friction(rotational component) ( k_4 [N・s/rad] )|
|/radius_robot|double|robot radius ( L [m] )|
|/display_vsm_path|bool|Whether to draw a path|
|/display_vsm_target|bool|Whether to draw the target position|

#### Parameter：Dynamic Window Approach
|Parameter|Type|Meaning|
|---|---|---|
|/min_linear|double|Minimum translational speed [m/s]|
|/max_linear|double|Maximum translational speed [m/s]|
|/min_angular_deg|double|Minimum rotation speed [deg/s]|
|/max_angular_deg|double|Maximum rotation speed [deg/s]|
|/predict_step|int|Predicted Steps|
|/sampling_time|double|Sampling time(Time for 1 prediction step)|
|/velocity_step|double|Number of predicted paths of translational speed|
|/angle_velocity_step|double|Number of predicted paths of rotational speed|
|/weight_heading|double|heading(v,ω) weight(Used in DWA)|
|/weight_obstacle|double|obstacle(v,ω) weight(Used in DWA)|
|/weight_velocity|double|velocity(v,ω) weight(Used in DWA)|
|/weight_vsm_heading|double|heading(v,ω) weight(VSM-Used in DWA)|
|/weight_vsm_obstacle|double|obstacle(v,ω) weight(VSM-Used in DWA)|
|/weight_vsm_linear|double|linear(v,ω) weight(VSM-Used in DWA)|
|/weight_vsm_angular|double|angular(v,ω) weight(VSM-Used in DWA)|
|/obstacle_cost_radius|double|Obstacle cost[m]|
|/display_optimal_path|bool|Whether to draw the optimal route|
|/display_all_path|bool|Whether to draw a predicted path|

##### About Evaluation Functions
- DWA evaluation function
```
G(v,ω) = α * heading(v,ω) + β * obstacle(v,ω) + γ * velocity(v,ω)
```
- VSM-DWA evaluation function
```
G(v,ω) = α * heading(v,ω) + β * obstacle(v,ω) + γ * linear(v,ω) + δ * angular(v,ω)
```
※ α，β，γ，δ：weight parameter

|term|Meaning|
|---|---|
|heading(v,ω)：directional term|・The difference angle between the robot's heading and goal direction at the time of the control input minus 180 degrees<br>・If the robot is heading straight for the goal, the value of the direction term is larger|
|obstacle(v,ω)：obstacle distance term<br>※Synonymous with dist(v,ω) in the DWA paper|・Value of the distance to the nearest obstacle at the time of control input<br>・Value of the obstacle distance term for control inputs far from the obstacle becomes larger|
|velocity(v,ω)：velocity term|・Value of the translational velocity of the control input<br>・The value of the velocity term for the control input with the faster velocity is larger|
|linear(v,ω)：VSM translational velocity term|・Inverse value of the difference between the translational velocity of the control input and the translational velocity obtained from the virtual spring model<br>・The value of the velocity term of the control input that matches the translational velocity obtained from the virtual spring model becomes larger|
|angular(v,ω)：VSM rotational velocity term|・Inverse value of the difference between the rotational speed of the control input and the rotational speed obtained from the virtual spring model<br>・The value of the velocity term of the control input that matches the rotational speed obtained from the virtual spring model becomes larger|

#### Parameter：PID Controller
|Parameter|Type|Meaning|
|---|---|---|
|/p_gain|double|Proportional control gain|
|/i_gain|double|Integral control gain|
|/d_gain|double|Differential control gain|
|/max_pid_angular_deg|double|Maximum rotation speed|

#### Parameter file
- [following_control_param.yaml](param/following_control_param.yaml)
    - Parameters for driving control
    - path：`sobit_follower/param/following_control_param.launch.xml`
- [velocity_smoother_param.yaml](param/velocity_smoother_param.yaml)
    - Parameters for speed smoothing
    - path：`sobit_follower/param/velocity_smoother_param.launch.xml`

<!-- Milestone -->
## Milestone

- [x] OSS
    - [x] Improved documentation
    - [x] Unified coding style


See the [open issues ][license-url] for a full list of proposed features (and known issues).
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- 参考文献 -->
<!-- ## 参考文献

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
