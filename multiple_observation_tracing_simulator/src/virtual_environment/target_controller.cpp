#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <random>

#define FREE 0
#define LINE 1
#define CIRCLE 2
#define RANDOM 3

namespace multiple_observation_tracing_simulator {
    class TargetController : public rclcpp::Node {
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_teleop_;
        rclcpp::TimerBase::SharedPtr timer_;

        unsigned int moving_mode_;
        double linear_speed_;
        double circle_theta_deg_;

        std::random_device rnd_;
        std::mt19937 mt_;
        std::unique_ptr<std::uniform_real_distribution<double>> rand_theta_deg_;

        double pre_time_;
        double rand_ang_;

        void callbackTimer();

    public:
        TargetController();
    };

    TargetController::TargetController() : Node("target_controller"), mt_(rnd_()) {
        pub_teleop_ = create_publisher<geometry_msgs::msg::Twist>( "/target/teleop", 1 );
        timer_ = create_wall_timer( std::chrono::milliseconds(33), std::bind(&TargetController::callbackTimer, this) );

        declare_parameter( "moving_mode", FREE );
        declare_parameter( "linear_speed", 0.0 );
        declare_parameter( "circle_theta_deg", 0.0 );
        declare_parameter( "random_theta_deg", 10.0 );

        moving_mode_ = get_parameter( "moving_mode" ).as_int();
        linear_speed_ = get_parameter( "linear_speed" ).as_double();
        circle_theta_deg_ = get_parameter( "circle_theta_deg" ).as_double();
        rand_theta_deg_ = std::make_unique<std::uniform_real_distribution<double>>( -get_parameter("random_theta_deg").as_double(), get_parameter("random_theta_deg").as_double() );

        pre_time_ = now().seconds();
    }

    void TargetController::callbackTimer() {
        auto vel = geometry_msgs::msg::Twist();
        double curt_time = now().seconds();

        switch ( moving_mode_ ) {
            case LINE:
                vel.linear.x = linear_speed_;
                break;
            case CIRCLE:
                vel.linear.x = linear_speed_;
                vel.angular.z = circle_theta_deg_ * M_PI / 180.0;
                break;
            case RANDOM:
                vel.linear.x = linear_speed_;
                if ( curt_time - pre_time_ > 1.0 ) {
                    pre_time_ = curt_time;
                    rand_ang_ = (*rand_theta_deg_)(mt_) * M_PI / 180.0;
                }
                vel.angular.z = rand_ang_;
                break;
            case FREE:
            default:
                return;
        }

        pub_teleop_->publish( vel );
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multiple_observation_tracing_simulator::TargetController>());
    rclcpp::shutdown();
    return 0;
}
