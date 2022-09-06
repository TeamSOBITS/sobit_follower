# Person Following Control
- 仮想ばねモデルを用いた人間追従制御にDynamic Window Approachによる障害物回避を組み込んだ手法

## About
### Virtual Spring Model
* 仮想ばねモデルによる追従制御
* ロボットと目標物の間に仮想的な"ばね"を作り、追従
* 英語論文 : [Human-following mobile robot in a distributed intelligent sensor network](https://ieeexplore.ieee.org/document/1265801)
* 日本語論文 : [知能化空間における移動ロボットの人間追制御](https://www.jstage.jst.go.jp/article/jrsj1983/22/1/22_1_103/_pdf)

### Dynamic Window Approach
* Dynamic Window Approachによる追従制御
* 以下の情報から，ロボットの運動モデルに則した制御入力を計算する検索ベースのローカル経路生成アルゴリズム
    * 現在の位置情報
    * 現在の速度情報
    * ロボット周辺の障害物情報
    * ゴールの位置情報
* 英語論文 : [The Dynamic Window Approach to Collision Avoidance](https://www.researchgate.net/publication/3344494_The_Dynamic_Window_Approach_to_Collision_Avoidance)
* サンプルプログラム : [Dynamic Window Approachを利用したMotion planningのMATLAB, Python サンプルプログラム](https://myenigma.hatenablog.com/entry/20140624/1403618922)

### Virtual Spring Model Dynamic Window Approach
* 仮想ばねモデルで得た経路をベースにし，DWAを行う追従制御
* 以下の情報から，ロボットの運動モデルに則した制御入力を計算する検索ベースのローカル経路生成アルゴリズム
    * 現在の位置情報
    * ロボット周辺の障害物情報
    * ゴールの位置情報
    * 仮想ばねモデルで得た速度情報

## How To Use
### following_control
```bash
$ roslaunch person_following_control following_control.launch
# 引数
    # manager_name      :   Nodeletマネージャー名
    # manager_threads   :   Nodeletスレッド数
    # rqt_reconfigure   :   rqt_reconfigureを表示するか
    # rviz              :   rvizを表示するか
    # rviz_cfg          :   rvizの設定ファイルパス
    # following_method  :   追従手法
    # use_smoother      :   velocity_smootherを使用するか
```
### simulator
```bash
$ roslaunch person_following_control simulator.launch
# 引数
    # obstacle_number   :   障害物の個数
    # observation_noise :   観測ノイズ(正規分布の分散)
    # following_method  :   追従手法
    # use_smoother      :   velocity_smootherを使用するか
```

#### Node
|ノード名|意味|
|---|---|
|/person_following_control/following_control_nodelet_manager|追従制御のノードレットの管理|
|/person_following_control/person_following_control|追従制御|

#### Subscriptions
|トピック名|型|意味|
|---|---|---|
|/multiple_sensor_person_tracking/following_position|multiple_sensor_person_tracking/FollowingPosition|追従位置と障害物|

#### Publications
|トピック名|型|意味|
|---|---|---|
|/cmd_vel_mux/input/teleop|multiple_sensor_person_tracking/FollowingPosition|追従制御の速度情報|

#### Parameter：Following Control
|パラメータ名|型|意味|
|---|---|---|
|/base_footprint_name|string|基準フレーム名|
|/following_distance|double|人と保つ距離 ( l_0 [m] )|
|/following_method|int|追従手法|

##### following_method
```
0 : VirtualSpringModel-DynamicWindowApproach
1 : VirtualSpringModel
2 : DynamicWindowApproach
3 : PIDController
```

#### Parameter：Virtual Spring Model
|パラメータ名|型|意味|
|---|---|---|
|/following_angle_deg|double|追従位置の角度 ( theta_sh [deg] )|
|/spring_constant_linear|double|ばね定数(並進成分) ( k_1 [N/m] )|
|/spring_constant_angular|double|ばね定数(回転成分) ( k_2 [N・m/rad] )|
|/weight_robot|double|ロボットの重量 ( M [Kg] )|
|/moment_inertia|double|慣性モーメント ( I [Kg・m^2] )|
|/viscous_friction_linear|double|粘性摩擦(並進成分) ( k_3 [N・s/m] )|
|/viscous_friction_angular|double|粘性摩擦(回転成分) ( k_4 [N・s/rad] )|
|/radius_robot|double|ロボット半径 ( L [m] )|
|/display_vsm_path|bool|経路を描画するか|
|/display_vsm_target|bool|目標位置を描画するか|

#### Parameter：Dynamic Window Approach
|パラメータ名|型|意味|
|---|---|---|
|/min_linear|double|最小並進速度 [m/s]|
|/max_linear|double|最大並進速度 [m/s]|
|/min_angular_deg|double|最小回転速度 [deg/s]|
|/max_angular_deg|double|最大回転速度 [deg/s]|
|/predict_step|int|予測ステップ数|
|/sampling_time|double|サンプリングタイム(1予測ステップの時間)|
|/velocity_step|double|並進速度の予測経路数|
|/angle_velocity_step|double|回転速度の予測経路数|
|/weight_heading|double|heading(v,ω)の重み(DWAで使用)|
|/weight_obstacle|double|obstacle(v,ω)の重み(DWAで使用)|
|/weight_velocity|double|velocity(v,ω)の重み(DWAで使用)|
|/weight_vsm_heading|double|heading(v,ω)の重み(VSM-DWAで使用)|
|/weight_vsm_obstacle|double|obstacle(v,ω)の重み(VSM-DWAで使用)|
|/weight_vsm_linear|double|linear(v,ω)の重み(VSM-DWAで使用)|
|/weight_vsm_angular|double|angular(v,ω)の重み(VSM-DWAで使用)|
|/obstacle_cost_radius|double|障害物コスト[m]|
|/display_optimal_path|bool|最適経路を描画するか|
|/display_all_path|bool|予測経路を描画するか|

#### Parameter：PID Controller
|パラメータ名|型|意味|
|/p_gain|double|比例制御ゲイン|
|/i_gain|double|積分制御ゲイン|
|/d_gain|double|微分制御ゲイン|
|/max_pid_angular_deg|double|最大回転速度|

#### Parameter：Simulator
|パラメータ名|型|意味|
|---|---|---|
|/obstacle_number|int|障害物の個数|
|/observation_noise|double|観測ノイズ(正規分布の分散)|

#### 評価関数について
- DWAの評価関数
```
G(v,ω) = α * heading(v,ω) + β * obstacle(v,ω) + γ * velocity(v,ω)
```
- VSM-DWAの評価関数
```
G(v,ω) = α * heading(v,ω) + β * obstacle(v,ω) + γ * linear(v,ω) + δ * angular(v,ω)
```
※ α，β，γ，δ：重みパラメータ

|項|意味|
|---|---|
|heading(v,ω)：方向項|・制御入力の時のロボットの方位とゴール方向の差の角度を180度から引いた値<br>・ロボットがゴールに真っ直ぐ向かっている場合は，方向項の値は大きくなる|
|obstacle(v,ω)：障害物距離項<br>※DWAの論文でのdist(v,ω)と同義|・制御入力の時の最近傍の障害物までの距離の値<br>・障害物から遠い制御入力の障害物距離項の値が大きくなる|
|velocity(v,ω)：速度項|・制御入力の並進速度の値<br>・速度が早い制御入力の速度項の値が大きくなる|
|linear(v,ω)：VSM並進速度項|・制御入力の並進速度と仮想ばねモデルで得た並進速度の差の逆数値<br>・仮想ばねモデルで得た並進速度と一致する制御入力の速度項の値が大きくなる|
|angular(v,ω)：VSM回転速度項|・制御入力の回転速度と仮想ばねモデルで得た回転速度の差の逆数値<br>・仮想ばねモデルで得た回転速度と一致する制御入力の速度項の値が大きくなる|