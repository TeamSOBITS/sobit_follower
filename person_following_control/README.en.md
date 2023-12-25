<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Person Following Control

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#summary">Summary</a>
    </li>
    <li>
    　<a href="#About each tracking control method">About each tracking control method</a>
      <ul>
        <li><a href="#virtual-spring-model">Virtual Spring Model</a></li>
        <li><a href="#dynamic-window-approach">Dynamic Window Approach</a></li>
        <li><a href="#virtual-spring-model-dynamic-window-approach">Virtual Spring Model Dynamic Window Approach</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#sobit-edu-following-control">SOBIT EDU Following control</a></li>
        <li><a href="#sobit-edu-simulator">SOBIT EDU Simulator</a></li>
        <li><a href="#sobit-pro-following-control">SOBIT PRO Following control</a></li>
      </ul>
    </li>
    <li>
    　<a href="#about-each-parameter">About each parameter</a>
      <ul>
        <li><a href="#node">Node</a></li>
        <li><a href="#subscriptions">Subscriptions</a></li>
        <li><a href="#publications">Publications</a></li>
        <li><a href="#parameterfollowing-control">Parameter：Following Control</a></li>
        <li><a href="#parametervirtual-spring-model">Parameter：Virtual Spring Model</a></li>
        <li><a href="#parameterdynamic-window-approach">Parameter：Dynamic Window Approach</a></li>
        <li><a href="#parameterpid-controller">Parameter：PID Controller</a></li>
        <li><a href="#parametersimulator">Parameter：Simulator</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>

## Summary
- Person-following control using Virtual Spring Model with obstacle avoidance by Dynamic Window Approach
## About each tracking control method
### Virtual Spring Model
* Tracking control by Virtual Spring Model
* Create a virtual "spring" between the robot and the target object and follow it
* English Papers : [Human-following mobile robot in a distributed intelligent sensor network](https://ieeexplore.ieee.org/document/1265801)
* Japanese Papers : [知能化空間における移動ロボットの人間追制御](https://www.jstage.jst.go.jp/article/jrsj1983/22/1/22_1_103/_pdf)

### Dynamic Window Approach
* Tracking control by Dynamic Window Approach
* A search-based local path generation algorithm that computes control inputs according to the robot's motion model from the following information
    * Current location information
    * Current speed information
    * Obstacle information around the robot
    * Goal location information
* English Papers : [The Dynamic Window Approach to Collision Avoidance](https://www.researchgate.net/publication/3344494_The_Dynamic_Window_Approach_to_Collision_Avoidance)
* Sample Program : [Dynamic Window Approachを利用したMotion planningのMATLAB, Python サンプルプログラム](https://myenigma.hatenablog.com/entry/20140624/1403618922)

### Virtual Spring Model Dynamic Window Approach
* Tracking control that performs DWA based on the path obtained from the Virtual Spring Model
* A search-based local path generation algorithm that computes control inputs according to the robot's motion model from the following information
    * Current Location
    * Obstacle information around the robot
    * Goal location information
    * Velocity information obtained from the Virtual Spring Model

## Launch and Usage
### SOBIT EDU Following control
```python
$ roslaunch person_following_control sobit_edu_person_following_control.launch
# Argument
    # manager_name      :   Nodelet manager name
    # manager_threads   :   Number of Nodelet Threads
    # rqt_reconfigure   :   Whether to show rqt_reconfigure
    # rviz              :   Whether to display rviz
    # rviz_cfg          :   rviz configuration file path
    # following_method  :   following method
    # use_smoother      :   Whether to use velocity_smother
```
### SOBIT EDU simulator
```python
$ roslaunch person_following_control sobit_edu_simulator.launch
# Argument
    # obstacle_number   :   Number of obstacles
    # observation_noise :   Observation noise (variance of normal distribution)
    # following_method  :   following method
    # use_smoother      :   Whether to use velocity_smother
```
### SOBIT PRO Following control
```python
$ roslaunch person_following_control sobit_pro_following_control.launch
# Argument
    # manager_name      :   Nodelet manager name
    # manager_threads   :   Number of Nodelet Threads
    # rqt_reconfigure   :   Whether to show rqt_reconfigure
    # rviz              :   Whether to display rviz
    # rviz_cfg          :   rviz configuration file path
    # following_method  :   following method
    # use_smoother      :   Whether to use velocity_smother
```
## About each parameter
#### Node
|Node Name|Meaning|
|---|---|
|/person_following_control/following_control_nodelet_manager|Management of nodelets for tracking control|
|/person_following_control/person_following_control|Following control|

#### Subscriptions
|Topic Name|Type|Meaning|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|Tracking position and obstacles|

#### Publications
|Topic Name|Type|Meaning|
|---|---|---|
|/cmd_vel_mux/input/teleop|multiple_sensor_person_tracking/FollowingPosition|Speed information for tracking control|

#### Parameter：Following Control
|Parameter Name|Type|Meaning|
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
|Parameter Name|Type|Meaning|
|---|---|---|
|/following_angle_deg|double|Angle of tracking position ( theta_sh [deg] )|
|/spring_constant_linear|double|Spring constant (translational component)( k_1 [N/m] )|
|/spring_constant_angular|double|Spring constant (rotational component) ( k_2 [N・m/rad] )|
|/weight_robot|double|Robot weight ( M [Kg] )|
|/moment_inertia|double|Moment of inertia ( I [Kg・m^2] )|
|/viscous_friction_linear|double|Viscous friction (translational component) ( k_3 [N・s/m] )|
|/viscous_friction_angular|double|Viscous friction (rotational component) ( k_4 [N・s/rad] )|
|/radius_robot|double|Robot radius ( L [m] )|
|/display_vsm_path|bool|Whether to draw a path|
|/display_vsm_target|bool|Whether to draw the target position|

#### Parameter：Dynamic Window Approach
|Parameter Name|Type|Meaning|
|---|---|---|
|/min_linear|double|Minimum translational speed [m/s]|
|/max_linear|double|Max translational speed [m/s]|
|/min_angular_deg|double|Minimum rotation speed [deg/s]|
|/max_angular_deg|double|Max rotation speed [deg/s]|
|/predict_step|int|Predicted Steps|
|/sampling_time|double|Sampling time (time of 1 prediction step)|
|/velocity_step|double|Number of predicted paths of translational speed|
|/angle_velocity_step|double|Number of predicted paths of rotational speed|
|/weight_heading|double|heading(v,ω) of weight(Used in DWA)|
|/weight_obstacle|double|obstacle(v,ω) of weight(Used in DWA)|
|/weight_velocity|double|velocity(v,ω) of weight(Used in DWA)|
|/weight_vsm_heading|double|heading(v,ω) of weight(Used in VSM-DWA)|
|/weight_vsm_obstacle|double|obstacle(v,ω) of weight(Used in VSM-DWA)|
|/weight_vsm_linear|double|linear(v,ω) of weight(Used in VSM-DWA)|
|/weight_vsm_angular|double|angular(v,ω) of weight(Used in VSM-DWA)|
|/obstacle_cost_radius|double|Obstacle cost[m]|
|/display_optimal_path|bool|Whether to draw the optimal path|
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
※ α，β，γ，δ：Weight

|Term|Meaning|
|---|---|
|heading(v,ω)：direction term|・The difference angle between the robot's heading and the direction of the goal at the time of the control input minus 180 degrees<br>・If the robot is heading straight for the goal, the value of the direction term is larger|
|obstacle(v,ω)：obstacle distance term<br>※Synonymous with dist(v,ω) in the DWA paper|・The value of dist(v,ω) in the DWA paper and the distance to the nearest obstacle at the same control input<br>・The value of the obstacle distance term for control inputs far from the obstacle becomes larger|
|velocity(v,ω)：velocity term|・Translational speed value of control input<br>・The value of the velocity term for control inputs with faster velocities is larger|
|linear(v,ω)：VSM translational velocity term|・Inverse value of the difference between the translational velocity of the control input and the translational velocity obtained from the virtual spring model<br>・The value of the velocity term of the control input that matches the translational velocity obtained from the virtual spring model becomes larger|
|angular(v,ω)：VSM rotational velocity term|・Inverse value of the difference between the rotational speed of the control input and the rotational speed obtained from the virtual spring model<br>・The value of the velocity term of the control input that matches the rotational speed obtained from the virtual spring model becomes larger|

#### Parameter：PID Controller
|Parameter Name|Type|Meaning|
|---|---|---|
|/p_gain|double|Proportional control gain|
|/i_gain|double|Integral control gain|
|/d_gain|double|Differential control gain|
|/max_pid_angular_deg|double|Maximum rotation speed|

#### Parameter：Simulator
|Parameter Name|Type|Meaning|
|---|---|---|
|/obstacle_number|int|Number of obstacles|
|/observation_noise|double|Observation noise (variance of normal distribution)|

<!-- Milestone -->
## Milestone

- [x] OSS
    - [x]  Improved documentation
    - [x]  Unified coding style

See the open issues for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- 参考文献 -->
<!-- ## 参考文献

* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->
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