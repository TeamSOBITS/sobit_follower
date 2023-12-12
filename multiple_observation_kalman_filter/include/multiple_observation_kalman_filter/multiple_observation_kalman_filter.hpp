#ifndef MULTIPLE_OBSERVATION_KALMAN_FILTER
#define MULTIPLE_OBSERVATION_KALMAN_FILTER

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

namespace multiple_observation_kalman_filter{
    class KalmanFilter {
        private :
            // x : Estimated value (mean value of normal distribution)
            Eigen::Vector4f estimated_value_;
            // u : external element
            Eigen::Vector4f external_elements_;
            // P : Initial covariance matrix of the estimators H
            Eigen::Matrix4f estimated_covariance_matrix_;
            // F : state transition matrix
            Eigen::Matrix4f state_transition_matrix_;
            // H :  observation matrix
            Eigen::Matrix<float, 2, 4> observation_matrix_;
            // Q : Process noise (used in prediction step)
            Eigen::Matrix4f model_error_covariance_matrix_;
            // R : System noise (used in Kalman gain calculation)
            Eigen::Matrix2f kalman_error_covariance_matrix_;
            // Process Noise
            double process_noise_;

        public :
            KalmanFilter ( const double dt, const double process_noise, const double system_noise );
            void changeParameter( const double process_noise, const double system_noise );
            void init( const Eigen::Vector2f& observed_value );
            void compute( const double dt, const Eigen::Vector2f& observed_value1, const Eigen::Vector2f& observed_value2, Eigen::Vector4f* estimated_value );
            void compute( const double dt, const Eigen::Vector2f& observed_value1, Eigen::Vector4f* estimated_value );
            void compute( const double dt, Eigen::Vector4f* estimated_value );

    };
}

#endif