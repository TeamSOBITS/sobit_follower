#include <iostream>
#include <ros/ros.h> 
#include <ros/time.h>
#include <ros/duration.h>  
#include <tf/transform_listener.h>  
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <sobit_follower/FollowPosition.h>

// PersonTracker Mode
constexpr int NON_WORKING = -1;
constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;
// Robot Mode
constexpr int STOP = 4;
constexpr int FOLLOW = 5;
constexpr int ATTENTION = 6;
// Path Type
constexpr int DWA_PATH = 7;
constexpr int PID_ROTATE = 8;
constexpr int PID_REVESE = 9;
constexpr int PATH_STOP = 10;

class FollowMeTalker {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_speech_; 
        ros::Subscriber sub_fp_;
        ros::Subscriber sub_path_type_;
        ros::Subscriber sub_robot_mode_;
        int robot_mode_;
        int pre_tracker_mode_;
        int curt_tracker_mode_;
        int path_type_;

        void callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input );
        void callbackPathType( const std_msgs::Int64ConstPtr &input );
        void callbackRobotMode ( const std_msgs::Int64ConstPtr &input );
    public :
        FollowMeTalker();
        void main();
};

void FollowMeTalker::callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input ) {
    curt_tracker_mode_ = input->process_flag;
}
void FollowMeTalker::callbackPathType( const std_msgs::Int64ConstPtr &input ) {
    path_type_ = input->data;
}
void FollowMeTalker::callbackRobotMode ( const std_msgs::Int64ConstPtr &input ){ 
    robot_mode_ = input->data; 
    if( input->data == STOP ) {
        curt_tracker_mode_ = NON_WORKING;
        pre_tracker_mode_ = NON_WORKING;
    }
}
FollowMeTalker::FollowMeTalker() : nh_(), pnh_("~") {
    pub_speech_ = nh_.advertise<std_msgs::String>("/speech_word", 1);
    sub_fp_ = nh_.subscribe("/follow_position", 1, &FollowMeTalker::callbackFollowPosition, this);  
    sub_path_type_ = nh_.subscribe("/path_generator/path_type", 1, &FollowMeTalker::callbackPathType, this);
    sub_robot_mode_ = nh_.subscribe("/follow_me_handle/path_mode", 1, &FollowMeTalker::callbackRobotMode, this); 
    robot_mode_ = STOP;
    curt_tracker_mode_ = NON_WORKING;
    pre_tracker_mode_ = NON_WORKING;
}
void FollowMeTalker::main() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        std::cout << "===========================" << std::endl;
        ros::spinOnce();
        std::cout << "robot_mode : " << robot_mode_ << std::endl;
        if ( robot_mode_ == STOP ) continue;
        std_msgs::String word;
        int curt_mode = curt_tracker_mode_;
        int pre_mode = pre_tracker_mode_;
        std::cout << "pre_mode : " << pre_mode << std::endl;
        std::cout << "curt_mode : " << curt_mode << std::endl;
        std::cout << "PATH TYPE : " << path_type_ << std::endl;
        if ( pre_mode == NON_WORKING && curt_mode == DETECTION  ) word.data = "検出中";
        else if ( pre_mode == NON_WORKING && curt_mode == TRACKING ) word.data = "走行開始";
        else if ( pre_mode == DETECTION && curt_mode == TRACKING ) word.data = "走行開始";
        else if ( pre_mode == PREDICTION && curt_mode == PREDICTION ) word.data = "再検出中";
        else if ( pre_mode == PREDICTION && curt_mode == TRACKING ) word.data = "再検出成功";
        else if ( pre_mode == PREDICTION && curt_mode == DETECTION ) {
            word.data = "見失いました";
            curt_tracker_mode_ = NON_WORKING;
            pre_tracker_mode_ = NON_WORKING;
        }
        if ( !word.data.empty() ) {
            pre_tracker_mode_ = curt_tracker_mode_;
            pub_speech_.publish( word ); 
        }
        if( path_type_ == PID_REVESE ) {
            word.data = "経路なし";
            if (!word.data.empty()) pub_speech_.publish( word );
        }
      loop_rate.sleep();
    }
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "follow_me_talker");
	FollowMeTalker fmt;
    fmt.main();
	ros::spin();
}
