#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>
#include <string>
#endif

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTimer>

namespace person_following_control {
    class TwistPanel : public rviz_common::Panel {
            Q_OBJECT
        public:
            TwistPanel(QWidget* parent = nullptr);
            ~TwistPanel() override;

            void save(rviz_common::Config config) const override;
            void load(const rviz_common::Config& config) override;

            public Q_SLOTS:
            void tick();

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_stamped_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
            QCheckBox* enable_check_;
            QLineEdit* topic_edit_;
            QCheckBox* stamped_check_;
            QLineEdit* frame_edit_;
            QRadioButton* radio1_;
            QRadioButton* radio2_;
            QLineEdit* max1_edit_;
            QLineEdit* max2_edit_;
            QLineEdit* max3_edit_;
            TouchWidget* touch_;
            bool pub_stamped_ = false;
            std::string pub_frame_;
            QTimer* output_timer_;
    };
}  // namespace person_following_control