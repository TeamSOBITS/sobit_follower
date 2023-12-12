# Multiple Observation Kalman Filter
- 2つの観測値を入力とするカルマンフィルタライブラリ
- 1つの観測値でも動作可能
-　状態方程式は等速モデル
- Multiple Sensor Person Trackingで使用
- [multiple_observation_kalman_filter.cpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)
- [multiple_observation_kalman_filter.hpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)
- カルマンフィルタについては[kalman_filter_tutorial](https://github.com/TeamSOBITS/kalman_filter_tutorial)を参照

## Functions
### init
- 観測値の初期化
```c++
void init(
    const Eigen::Vector2f& observed_value // 初期の観測値
)
```
### compute
- 現在の観測値を入力し，状態を更新

#### 観測値が2つ
```c++
void compute(
        const double dt,                            // フレーム間の時間[s]
        const Eigen::Vector2f& observed_value1,     // 観測値
        const Eigen::Vector2f& observed_value2,     // 観測値
        Eigen::Vector4f* estimated_value            // 推定値
    )
```

#### 観測値が1つ
```c++
void compute(
    const double dt,                            // フレーム間の時間[s]
    const Eigen::Vector2f& observed_value1,     // 観測値
    Eigen::Vector4f* estimated_value            // 推定値
)
```

#### 観測値が0(観測値が得られなかった場合)
```c++
void compute(
    const double dt,                            // フレーム間の時間[s]
    Eigen::Vector4f* estimated_value            // 推定値
)
```

## 参考文献
- [tracking-with-Extended-Kalman-Filter](https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter/blob/master/src/tracking.cpp)
- [カルマンフィルターのプロセスノイズ共分散行列
](https://gordiustears.net/process-noise-covariance-matrix-of-kalman-filter/)
- [Is acceleration noise modelled differently in EKF and UKF Kalman Filters?]( https://dsp.stackexchange.com/questions/43966/is-acceleration-noise-modelled-differently-in-ekf-and-ukf-kalman-filters)