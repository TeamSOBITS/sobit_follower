<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Multiple Observation Kalman Filter

<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#機能">機能</a>
      <ul>
        <li><a href="#初期化">初期化</a></li>
        <li><a href="#観測値が2つの場合の計算">観測値が2つの場合の計算</a></li>
        <li><a href="#観測値が1つの場合の計算">観測値が1つの場合の計算</a></li>
        <li><a href="#観測値が0観測値が得られなかった場合">観測値が0(観測値が得られなかった場合)</a></li>
      </ul>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>

## 概要
- 2つの観測値を入力とするカルマンフィルタライブラリ
- 1つの観測値でも動作可能
-　状態方程式は等速モデル
- Multiple Sensor Person Trackingで使用
- [multiple_observation_kalman_filter.cpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)
- [multiple_observation_kalman_filter.hpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)
## 機能
### 初期化
- 観測値の初期化
```c++
void init(
    const Eigen::Vector2f& observed_value // 初期の観測値
)
```

### 観測値が2つの場合の計算
- 2つの観測値を入力し，状態を更新
```c++
void compute(
        const double dt,                            // フレーム間の時間[s]
        const Eigen::Vector2f& observed_value1,     // 観測値
        const Eigen::Vector2f& observed_value2,     // 観測値
        Eigen::Vector4f* estimated_value            // 推定値
    )
```

### 観測値が1つの場合の計算
- 1つの観測値を入力し，状態を更新
```c++
void compute(
    const double dt,                            // フレーム間の時間[s]
    const Eigen::Vector2f& observed_value1,     // 観測値
    Eigen::Vector4f* estimated_value            // 推定値
)
```

### 観測値が0(観測値が得られなかった場合)
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


[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_follower.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_follower/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_follower.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_follower/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_follower.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_follower/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_follower.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_follower/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_follower.svg?style=for-the-badge
[license-url]: LICENSE