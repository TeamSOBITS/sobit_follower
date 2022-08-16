#include <multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp>

// https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter/blob/master/src/tracking.cpp
multiple_observation_kalman_filter::KalmanFilter::KalmanFilter( const double dt, const double process_noise, const double system_noise ) {
    // x : 推定値(正規分布の平均値) -> x, y, v_x. v_y
    estimated_value_ << 0.0, 0.0, 0.0, 0.0;
    // u : 外部要素
    external_elements_ << 0.0, 0.0, 0.0, 0.0;
    // P : 推定値の初期共分散行列H(初期値は適当に設定しても修正される)
    estimated_covariance_matrix_ << 0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    1000.0, 0.0,
                                    0.0,    0.0,    0.0,    1000.0;
    // F : 状態遷移行列(線形数理モデルy=axのa) 等速モデル
    state_transition_matrix_ << 1.0,    0.0,    dt,     0.0,
                                0.0,    1.0,    0.0,    dt,
                                0.0,    0.0,    1.0,    0.0,
                                0.0,    0.0,    0.0,    1.0;
    // H :  観測行列
    observation_matrix_ <<  1.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0;
    // Q : プロセスノイズ(予測ステップで使用) https://gordiustears.net/process-noise-covariance-matrix-of-kalman-filter/
    double noise_ax = process_noise;
    double noise_ay = process_noise;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    // https://dsp.stackexchange.com/questions/43966/is-acceleration-noise-modelled-differently-in-ekf-and-ukf-kalman-filters
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;
    process_noise_ = process_noise;
    // R : システムノイズ(カルマンゲイン算出で使用)
    kalman_error_covariance_matrix_ <<  system_noise,   0,
                                        0,              system_noise;
}

void multiple_observation_kalman_filter::KalmanFilter::changeParameter( const double process_noise, const double system_noise ) {
    process_noise_ = process_noise;

    // R : システムノイズ(カルマンゲイン算出で使用)
    kalman_error_covariance_matrix_ <<  system_noise,   0,
                                        0,              system_noise;
}

void multiple_observation_kalman_filter::KalmanFilter::init( const Eigen::Vector2f& observed_value ) {
    // x : 推定値(正規分布の平均値)
    estimated_value_ << observed_value[0], observed_value[1], 0.0, 0.0;
}

void multiple_observation_kalman_filter::KalmanFilter::compute( const double dt, const Eigen::Vector2f& observed_value1, const Eigen::Vector2f& observed_value2, Eigen::Vector4f* estimated_value ) {
    // Q : プロセスノイズ(予測ステップで使用) https://gordiustears.net/process-noise-covariance-matrix-of-kalman-filter/
    double noise_ax = process_noise_;
    double noise_ay = process_noise_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    // https://dsp.stackexchange.com/questions/43966/is-acceleration-noise-modelled-differently-in-ekf-and-ukf-kalman-filters
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;

    Eigen::Matrix4f invertible_matrix;
    invertible_matrix <<    1.0,    0.0,    0.0,    0.0,
                            0.0,    1.0,    0.0,    0.0,
                            0.0,    0.0,    1.0,    0.0,
                            0.0,    0.0,    0.0,    1.0;

    // 予測ステップ : 前フレームの予測値mと数理モデルから追跡位置の予測(事前確率)
    // x_t = F * x_t+1 + u + W : 予測位置の正規分布の平均値m
    Eigen::Vector4f estimated_value_average = ( state_transition_matrix_ * estimated_value_ ) + external_elements_;
    // P_t = F * P * F_T + Q 予測位置の正規分布の分散V
    Eigen::Matrix4f estimated_value_covariance = state_transition_matrix_ * estimated_covariance_matrix_ * state_transition_matrix_.transpose() + model_error_covariance_matrix_;

    // 観測更新ステップ : 事前確率と観測値から予測位置の補正(事後確率) observed_value1 & observed_value2
    // y_t = Z - ( H * x ) : 観測更新
    Eigen::Vector2f observed_value_update = observed_value1 - ( observation_matrix_ * estimated_value_average );
    // S = H * P * H_T
    Eigen::Matrix2f tmp = observation_matrix_ * estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : カルマンゲイン
    Eigen::Matrix<float, 4, 2> kalman_gain = estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : 予測位置の正規分布の平均値m
    Eigen::Vector4f fixed_estimated_value_average = estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : 予測位置の正規分布の分散V
    Eigen::Matrix4f fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * estimated_value_covariance;

    // y_t = Z - ( H * x ) : 観測更新
    observed_value_update = observed_value2 - ( observation_matrix_ * fixed_estimated_value_average );
    // S = H * P * H_T
    tmp = observation_matrix_ * fixed_estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : カルマンゲイン
    kalman_gain = fixed_estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : 予測位置の正規分布の平均値m
    fixed_estimated_value_average = fixed_estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : 予測位置の正規分布の分散V
    fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * fixed_estimated_value_covariance;

    *estimated_value = fixed_estimated_value_average;
    estimated_value_ = fixed_estimated_value_average;
    estimated_covariance_matrix_ = fixed_estimated_value_covariance;
    return;
}

void multiple_observation_kalman_filter::KalmanFilter::compute( const double dt, const Eigen::Vector2f& observed_value1, Eigen::Vector4f* estimated_value ) {
    // Q : プロセスノイズ(予測ステップで使用) https://gordiustears.net/process-noise-covariance-matrix-of-kalman-filter/
    double noise_ax = process_noise_;
    double noise_ay = process_noise_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    // https://dsp.stackexchange.com/questions/43966/is-acceleration-noise-modelled-differently-in-ekf-and-ukf-kalman-filters
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;

    Eigen::Matrix4f invertible_matrix;
    invertible_matrix <<    1.0,    0.0,    0.0,    0.0,
                            0.0,    1.0,    0.0,    0.0,
                            0.0,    0.0,    1.0,    0.0,
                            0.0,    0.0,    0.0,    1.0;

    // 予測ステップ : 前フレームの予測値mと数理モデルから追跡位置の予測(事前確率)
    // x_t = F * x_t+1 + u + W : 予測位置の正規分布の平均値m
    Eigen::Vector4f estimated_value_average = ( state_transition_matrix_ * estimated_value_ ) + external_elements_;
    // P_t = F * P * F_T + Q 予測位置の正規分布の分散V
    Eigen::Matrix4f estimated_value_covariance = state_transition_matrix_ * estimated_covariance_matrix_ * state_transition_matrix_.transpose() + model_error_covariance_matrix_;

    // 観測更新ステップ : 事前確率と観測値から予測位置の補正(事後確率) observed_value1 & observed_value2
    // y_t = Z - ( H * x ) : 観測更新
    Eigen::Vector2f observed_value_update = observed_value1 - ( observation_matrix_ * estimated_value_average );
    // S = H * P * H_T
    Eigen::Matrix2f tmp = observation_matrix_ * estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : カルマンゲイン
    Eigen::Matrix<float, 4, 2> kalman_gain = estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : 予測位置の正規分布の平均値m
    Eigen::Vector4f fixed_estimated_value_average = estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : 予測位置の正規分布の分散V
    Eigen::Matrix4f fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * estimated_value_covariance;

    *estimated_value = fixed_estimated_value_average;
    estimated_value_ = fixed_estimated_value_average;
    estimated_covariance_matrix_ = fixed_estimated_value_covariance;
    return;
}