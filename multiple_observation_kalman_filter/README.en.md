<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Multiple Observation Kalman Filter

<details>
  <summary>Table of Contents
</summary>
  <ol>
    <li>
      <a href="#summary">Summary</a>
    </li>
    <li>
      <a href="#function">Function</a>
      <ul>
        <li><a href="#initialization">initialization</a></li>
        <li><a href="#calculation-with-two-observations">Calculation with two observations</a></li>
        <li><a href="#calculation-with-one-observation">Calculation with one observation</a></li>
        <li><a href="#observation-value-is-0-when-no-observation-value-was-obtained">Observation value is 0 (when no observation value was obtained)</a></li>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

## Summary
- Kalman filter library with two observables as input
- Can work with one observation value
- Equation of state is a constant velocity model
- Used in Multiple Sensor Person Tracking
- [multiple_observation_kalman_filter.cpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)
- [multiple_observation_kalman_filter.hpp](multiple_observation_kalman_filter/src/multiple_observation_kalman_filter.cpp)

## Function
### initialization
- Initialization of observed values
```c++
void init(
    const Eigen::Vector2f& observed_value // initial observations
)
```

### Calculation with two observations
- Input two observations and update the state
```c++
void compute(
        const double dt,                            // frame time[s]
        const Eigen::Vector2f& observed_value1,     // observed value
        const Eigen::Vector2f& observed_value2,     // observed value
        Eigen::Vector4f* estimated_value            // estimated value
    )
```

### Calculation with one observation
- Input one observation and update the state
```c++
void compute(
    const double dt,                            // frame time[s]
    const Eigen::Vector2f& observed_value1,     // observed value
    Eigen::Vector4f* estimated_value            // estimated value
)
```

### Observation value is 0 (when no observation value was obtained)
```c++
void compute(
    const double dt,                            // frame time[s]
    Eigen::Vector4f* estimated_value            // estimated value
)
```

## Acknowledgments
- [tracking-with-Extended-Kalman-Filter](https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter/blob/master/src/tracking.cpp)
- [カルマンフィルターのプロセスノイズ共分散行列
](https://gordiustears.net/process-noise-covariance-matrix-of-kalman-filter/)
- [Is acceleration noise modelled differently in EKF and UKF Kalman Filters?]( https://dsp.stackexchange.com/questions/43966/is-acceleration-noise-modelled-differently-in-ekf-and-ukf-kalman-filters)


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