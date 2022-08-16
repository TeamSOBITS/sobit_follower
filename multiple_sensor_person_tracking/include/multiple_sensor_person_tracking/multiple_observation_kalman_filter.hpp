#ifndef MULTIPLE_OBSERVATION_KALMAN_FILTER
#define MULTIPLE_OBSERVATION_KALMAN_FILTER

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

namespace multiple_sensor_person_tracking{
    class KalmanFilter {
        private :
            // x : 推定値(正規分布の平均値)
            Eigen::Vector4f estimated_value_;
            // u : 外部要素
            Eigen::Vector4f external_elements_;
            // P : 推定値の初期共分散行列H(初期値は適当に設定しても修正される)
            Eigen::Matrix4f estimated_covariance_matrix_;
            // F : 状態遷移行列
            Eigen::Matrix4f state_transition_matrix_;
            // H :  観測行列
            Eigen::Matrix<float, 2, 4> observation_matrix_;
            // Q : プロセスノイズ(予測ステップで使用)
            Eigen::Matrix4f model_error_covariance_matrix_;
            // R : システムノイズ(カルマンゲイン算出で使用)
            Eigen::Matrix2f kalman_error_covariance_matrix_;
            // プロセスノイズ
            double process_noise_;

        public :
            KalmanFilter ( const double dt, const double process_noise, const double system_noise );
            void changeParameter( const double process_noise, const double system_noise );
            void init( const Eigen::Vector2f& observed_value );
            void compute( const double dt, const Eigen::Vector2f& observed_value1, const Eigen::Vector2f& observed_value2, Eigen::Vector4f* estimated_value );
            void compute( const double dt, const Eigen::Vector2f& observed_value1, Eigen::Vector4f* estimated_value );

    };
}

#endif