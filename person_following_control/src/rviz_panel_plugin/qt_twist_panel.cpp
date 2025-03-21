#include "qt_touch.h"
#include "qt_twist_panel.h"
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QCheckBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

namespace person_following_control {
    TwistPanel::TwistPanel(QWidget* parent) : rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>("twist_panel")) {
    QVBoxLayout* layout = new QVBoxLayout;

    QHBoxLayout* layout_1st = new QHBoxLayout;
    enable_check_ = new QCheckBox("Enable");
    layout_1st->addWidget(enable_check_);
    layout_1st->addWidget(new QLabel("Topic:"));
    topic_edit_ = new QLineEdit("");
    layout_1st->addWidget(topic_edit_);
    layout->addLayout(layout_1st);

    QHBoxLayout* layout_2nd = new QHBoxLayout;
    stamped_check_ = new QCheckBox("Stamped");
    layout_2nd->addWidget(stamped_check_);
    layout_2nd->addWidget(new QLabel("Frame:"));
    frame_edit_ = new QLineEdit("");
    layout_2nd->addWidget(frame_edit_);
    layout->addLayout(layout_2nd);

    QHBoxLayout* layout_3rd = new QHBoxLayout;
    radio1_ = new QRadioButton("X-Y");
    layout_3rd->addWidget(radio1_);
    layout_3rd->addWidget(new QLabel("X max:"));
    max1_edit_ = new QLineEdit("");
    layout_3rd->addWidget(max1_edit_);
    layout_3rd->addWidget(new QLabel("Y max:"));
    max2_edit_ = new QLineEdit("");
    layout_3rd->addWidget(max2_edit_);
    layout->addLayout(layout_3rd);

    QHBoxLayout* layout_4th = new QHBoxLayout;
    radio2_ = new QRadioButton("X-Yaw");
    layout_4th->addWidget(radio2_);
    layout_4th->addWidget(new QLabel("Yaw max:"));
    max3_edit_ = new QLineEdit("");
    layout_4th->addWidget(max3_edit_);
    layout->addLayout(layout_4th);

    QButtonGroup* group1 = new QButtonGroup();
    group1->addButton(radio1_);
    group1->addButton(radio2_);

    touch_ = new TouchWidget();
    layout->addWidget(touch_);

    setLayout(layout);

    QTimer* output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
    output_timer->start(100);

    touch_->setEnabled(false);
    touch_->update();
    }

    TwistPanel::~TwistPanel() {
    }

    void TwistPanel::tick() {
        if (!rclcpp::ok()) return;
    
        if (enable_check_->isChecked()) {
            std::string topic_name = topic_edit_->text().toStdString();
            if (!topic_name.empty()) {
                if (!twist_publisher_ && !twist_publisher_stamped_) {
                    if (stamped_check_->isChecked()) {
                        twist_publisher_stamped_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name, 10);
                        pub_stamped_ = true;
                    } else {
                        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
                        pub_stamped_ = false;
                    }
                }
            }
    
            if (pub_stamped_ && twist_publisher_stamped_) {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.frame_id = pub_frame_;
                msg.header.stamp = node_->get_clock()->now();
                if (radio1_->isChecked()) {
                    msg.twist.linear.x = -1 * max1_edit_->text().toFloat() * (touch_->y_value);
                    msg.twist.linear.y = -1 * max2_edit_->text().toFloat() * (touch_->x_value);
                } else if (radio2_->isChecked()) {
                    msg.twist.linear.x = -1 * max1_edit_->text().toFloat() * (touch_->y_value);
                    msg.twist.angular.z = -1 * max3_edit_->text().toFloat() * (touch_->x_value);
                }
                twist_publisher_stamped_->publish(msg);
            } else if (!pub_stamped_ && twist_publisher_) {
                geometry_msgs::msg::Twist msg;
                if (radio1_->isChecked()) {
                    msg.linear.x = -1 * max1_edit_->text().toFloat() * (touch_->y_value);
                    msg.linear.y = -1 * max2_edit_->text().toFloat() * (touch_->x_value);
                } else if (radio2_->isChecked()) {
                    msg.linear.x = -1 * max1_edit_->text().toFloat() * (touch_->y_value);
                    msg.angular.z = -1 * max3_edit_->text().toFloat() * (touch_->x_value);
                }
                twist_publisher_->publish(msg);
            }
        }
    }

    void TwistPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Topic", topic_edit_->text());
        config.mapSetValue("Stamped", stamped_check_->isChecked());
        config.mapSetValue("Frame", frame_edit_->text());
        config.mapSetValue("radio1", radio1_->isChecked());
        config.mapSetValue("radio2", radio2_->isChecked());
        config.mapSetValue("max1", max1_edit_->text());
        config.mapSetValue("max2", max2_edit_->text());
        config.mapSetValue("max3", max3_edit_->text());
    }

    void TwistPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);

        QString tmp_text;
        bool tmp_bool;
        if (config.mapGetString("Topic", &tmp_text)) topic_edit_->setText(tmp_text);
        if (config.mapGetBool("Stamped", &tmp_bool)) stamped_check_->setChecked(tmp_bool);
        if (config.mapGetString("Frame", &tmp_text)) frame_edit_->setText(tmp_text);
        if (config.mapGetBool("radio1", &tmp_bool)) radio1_->setChecked(tmp_bool);
        if (config.mapGetBool("radio2", &tmp_bool)) radio2_->setChecked(tmp_bool);
        if (config.mapGetString("max1", &tmp_text)) max1_edit_->setText(tmp_text);
        if (config.mapGetString("max2", &tmp_text)) max2_edit_->setText(tmp_text);
        if (config.mapGetString("max3", &tmp_text)) max3_edit_->setText(tmp_text);
    }
}  // namespace person_following_control

PLUGINLIB_EXPORT_CLASS(person_following_control::TwistPanel, rviz_common::Panel)