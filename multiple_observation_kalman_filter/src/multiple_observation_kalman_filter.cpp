#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

multiple_observation_kalman_filter::KalmanFilter::KalmanFilter( const double dt, const double process_noise, const double system_noise ) {
    // x : estimate (mean of normal distribution) -> x, y, v_x. v_y
    estimated_value_ << 0.0, 0.0, 0.0, 0.0;
    // u : external element
    external_elements_ << 0.0, 0.0, 0.0, 0.0;
    // P : The initial covariance matrix of the estimates H
    estimated_covariance_matrix_ << 0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    1000.0, 0.0,
                                    0.0,    0.0,    0.0,    1000.0;
    // F : State transition matrix (linear mathematical model y=ax a) Constant velocity model
    state_transition_matrix_ << 1.0,    0.0,    dt,     0.0,
                                0.0,    1.0,    0.0,    dt,
                                0.0,    0.0,    1.0,    0.0,
                                0.0,    0.0,    0.0,    1.0;
    // H :  observation matrix
    observation_matrix_ <<  1.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0;
    // Q : Process noise (used in prediction step)
    double noise_ax = process_noise;
    double noise_ay = process_noise;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    //
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;
    process_noise_ = process_noise;
    // R : System noise (used in Kalman gain calculation)
    kalman_error_covariance_matrix_ <<  system_noise,   0,
                                        0,              system_noise;
}

void multiple_observation_kalman_filter::KalmanFilter::changeParameter( const double process_noise, const double system_noise ) {
    process_noise_ = process_noise;

    // R : System noise (used in Kalman gain calculation)
    kalman_error_covariance_matrix_ <<  system_noise,   0,
                                        0,              system_noise;
}

void multiple_observation_kalman_filter::KalmanFilter::init( const Eigen::Vector2f& observed_value ) {
    // x : Estimated value (mean value of normal distribution)
    estimated_value_ << observed_value[0], observed_value[1], 0.0, 0.0;
}

void multiple_observation_kalman_filter::KalmanFilter::compute( const double dt, const Eigen::Vector2f& observed_value1, const Eigen::Vector2f& observed_value2, Eigen::Vector4f* estimated_value ) {
    // Q : Process noise (used in prediction step)
    double noise_ax = process_noise_;
    double noise_ay = process_noise_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    //
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;

    Eigen::Matrix4f invertible_matrix;
    invertible_matrix <<    1.0,    0.0,    0.0,    0.0,
                            0.0,    1.0,    0.0,    0.0,
                            0.0,    0.0,    1.0,    0.0,
                            0.0,    0.0,    0.0,    1.0;

    // Prediction step : Predicts the tracking location based on the previous frame's prediction m and the mathematical model (prior probability)
    // x_t = F * x_t+1 + u + W : mean value of the normal distribution of the predicted positions m
    Eigen::Vector4f estimated_value_average = ( state_transition_matrix_ * estimated_value_ ) + external_elements_;
    // P_t = F * P * F_T + Q Variance of the normal distribution of predicted positions V
    Eigen::Matrix4f estimated_value_covariance = state_transition_matrix_ * estimated_covariance_matrix_ * state_transition_matrix_.transpose() + model_error_covariance_matrix_;

    // Observation update step : correction of predicted position from prior probability and observed value (posterior probability) observed_value1 & observed_value2
    // y_t = Z - ( H * x ) : observation update
    Eigen::Vector2f observed_value_update = observed_value1 - ( observation_matrix_ * estimated_value_average );
    // S = H * P * H_T
    Eigen::Matrix2f tmp = observation_matrix_ * estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : Kalman gain
    Eigen::Matrix<float, 4, 2> kalman_gain = estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : mean value of the normal distribution of the predicted positions m
    Eigen::Vector4f fixed_estimated_value_average = estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : variance V of the normal distribution of the predicted positions
    Eigen::Matrix4f fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * estimated_value_covariance;

    // y_t = Z - ( H * x ) : observation update
    observed_value_update = observed_value2 - ( observation_matrix_ * fixed_estimated_value_average );
    // S = H * P * H_T
    tmp = observation_matrix_ * fixed_estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : Kalman gain
    kalman_gain = fixed_estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : mean value of the normal distribution of the predicted positions m
    fixed_estimated_value_average = fixed_estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : variance V of the normal distribution of the predicted positions
    fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * fixed_estimated_value_covariance;

    *estimated_value = fixed_estimated_value_average;
    estimated_value_ = fixed_estimated_value_average;
    estimated_covariance_matrix_ = fixed_estimated_value_covariance;
    return;
}

void multiple_observation_kalman_filter::KalmanFilter::compute( const double dt, const Eigen::Vector2f& observed_value1, Eigen::Vector4f* estimated_value ) {
    // Q : Process noise (used in prediction step)
    double noise_ax = process_noise_;
    double noise_ay = process_noise_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    //
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;

    Eigen::Matrix4f invertible_matrix;
    invertible_matrix <<    1.0,    0.0,    0.0,    0.0,
                            0.0,    1.0,    0.0,    0.0,
                            0.0,    0.0,    1.0,    0.0,
                            0.0,    0.0,    0.0,    1.0;

    // Prediction step : Predicts the tracking location based on the previous frame's prediction m and the mathematical model (prior probability)
    // x_t = F * x_t+1 + u + W : mean value of the normal distribution of the predicted positions m
    Eigen::Vector4f estimated_value_average = ( state_transition_matrix_ * estimated_value_ ) + external_elements_;
    // P_t = F * P * F_T + Q Variance of the normal distribution of predicted positions V
    Eigen::Matrix4f estimated_value_covariance = state_transition_matrix_ * estimated_covariance_matrix_ * state_transition_matrix_.transpose() + model_error_covariance_matrix_;

    // Observation update step : correction of predicted position from prior probability and observed value (posterior probability) observed_value1 & observed_value2
    // y_t = Z - ( H * x ) : observation update
    Eigen::Vector2f observed_value_update = observed_value1 - ( observation_matrix_ * estimated_value_average );
    // S = H * P * H_T
    Eigen::Matrix2f tmp = observation_matrix_ * estimated_value_covariance * observation_matrix_.transpose() + kalman_error_covariance_matrix_;
    // K_t = P * H_T * S_-1 : Kalman gain
    Eigen::Matrix<float, 4, 2> kalman_gain = estimated_value_covariance * observation_matrix_.transpose() * tmp.inverse();
    // x'_t = x_t + K * y_t : mean value of the normal distribution of the predicted positions m
    Eigen::Vector4f fixed_estimated_value_average = estimated_value_average + ( kalman_gain * observed_value_update);
    // P'_t = ( I - ( K * H ) ) * P : variance V of the normal distribution of the predicted positions
    Eigen::Matrix4f fixed_estimated_value_covariance = ( invertible_matrix - ( kalman_gain * observation_matrix_ ) ) * estimated_value_covariance;

    *estimated_value = fixed_estimated_value_average;
    estimated_value_ = fixed_estimated_value_average;
    estimated_covariance_matrix_ = fixed_estimated_value_covariance;
    return;
}

void multiple_observation_kalman_filter::KalmanFilter::compute( const double dt, Eigen::Vector4f* estimated_value ) {
    // Q : Process noise (used in prediction step)
    double noise_ax = process_noise_;
    double noise_ay = process_noise_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    //
    model_error_covariance_matrix_ <<  dt_4/4*noise_ax,     0,                  dt_3/2*noise_ax,    0,
                                        0,                  dt_4/4*noise_ay,    0,                  dt_3/2*noise_ay,
                                        dt_3/2*noise_ax,    0,                  dt_2*noise_ax,      0,
                                        0,                  dt_3/2*noise_ay,    0,                  dt_2*noise_ay;

    Eigen::Matrix4f invertible_matrix;
    invertible_matrix <<    1.0,    0.0,    0.0,    0.0,
                            0.0,    1.0,    0.0,    0.0,
                            0.0,    0.0,    1.0,    0.0,
                            0.0,    0.0,    0.0,    1.0;

    // Prediction step : Predicts the tracking location based on the previous frame's prediction m and the mathematical model (prior probability)
    // x_t = F * x_t+1 + u + W : mean value of the normal distribution of the predicted positions m
    Eigen::Vector4f estimated_value_average = ( state_transition_matrix_ * estimated_value_ ) + external_elements_;
    // P_t = F * P * F_T + Q Variance of the normal distribution of predicted positions V
    Eigen::Matrix4f estimated_value_covariance = state_transition_matrix_ * estimated_covariance_matrix_ * state_transition_matrix_.transpose() + model_error_covariance_matrix_;

    *estimated_value = estimated_value_average;
    estimated_value_ = estimated_value_average;
    estimated_covariance_matrix_ = estimated_value_covariance;
    return;
}