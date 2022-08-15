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
    * 仮想ばねモデルで得た経路情報

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
```
### simulator
```bash
$ roslaunch person_following_control simulator.launch
# 引数
    # obstacle_number   :   障害物の個数
    # observation_noise :   観測ノイズ(正規分布の分散)
    # following_method  :   追従手法
```

## Parameter
### Following Control
|パラメータ名|型|意味|
|---|---|---|
|base_footprint_name|string|基準フレーム名|
|following_distance|double|人と保つ距離 ( l_0 [m] )|
|following_method|int|追従手法|

#### following_method
```
0 : virtual_spring_model_dynamic_window_approach
1 : virtual_spring_model
2 : dynamic_window_approach
```

### Virtual Spring Model
|パラメータ名|型|意味|
|---|---|---|
|following_angle_deg|double|追従位置の角度 ( theta_sh [deg] )|
|spring_constant_linear|double|ばね定数(並進成分) ( k_1 [N/m] )|
|spring_constant_angular|double|ばね定数(回転成分) ( k_2 [N・m/rad] )|
|weight_robot|double|ロボットの重量 ( M [Kg] )|
|moment_inertia|double|慣性モーメント ( I [Kg・m^2] )|
|viscous_friction_linear|double|粘性摩擦(並進成分) ( k_3 [N・s/m] )|
|viscous_friction_angular|double|粘性摩擦(回転成分) ( k_4 [N・s/rad] )|
|radius_robot|double|ロボット半径 ( L [m] )|
|display_vsm_path|bool|経路を描画するか|
|display_vsm_target|bool|目標位置を描画するか|

### Dynamic Window Approach
|パラメータ名|型|意味|
|---|---|---|
|min_velocity|double|最小並進速度 [m/s]|
|max_velocity|double|最大並進速度 [m/s]|
|min_angle_velocity_deg|double|最小回転速度 [deg/s]|
|max_angle_velocity_deg|double|最大回転速度 [deg/s]|
|predict_step|int|予測ステップ数|
|sampling_time|double|サンプリングタイム(1予測ステップの時間)|
|velocity_step|double|並進速度の予測経路数|
|angle_velocity_step|double|回転速度の予測経路数|
|weight_goal|double|goal(v,ω)の重み|
|weight_obstacle|double|obstacle(v,ω)の重み|
|weight_angle|double|heading(v,ω)の重み(DWAで使用)|
|weight_velocity|double|velocity(v,ω)の重み(DWAで使用)|
|weight_vsm_angular|double|angular(v,ω)の重み(VSM-DWAで使用)|
|weight_vsm_linear|double|linear(v,ω)の重み(VSM-DWAで使用)|
|obstacle_cost_radius|double|障害物コスト[m]|
|display_optimal_path|bool|最適経路を描画するか|
|display_all_path|bool|予測経路を描画するか|

### Simulator
|パラメータ名|型|意味|
|---|---|---|
|obstacle_number|int|障害物の個数|
|observation_noise|double|観測ノイズ(正規分布の分散)|