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

class FollowMeHandle {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_speech_; 
        ros::Publisher pub_switch_tracker_; 
        ros::Publisher pub_switch_path_; 
        ros::Subscriber sub_vc_;
        int robot_mode_;
        int vc_flag_;

        void callbackVoiceTrigger ( const std_msgs::StringConstPtr &input );
    public :
        FollowMeHandle();
};

void FollowMeHandle::callbackVoiceTrigger ( const std_msgs::StringConstPtr &input ){
    ROS_INFO("VoiceTrigger : %s",input->data.c_str());
    std_msgs::String word;
    std_msgs::Bool is_track;
    std_msgs::Int64 path_mode;
    if( input->data == "stop" ) {
        word.data = "停止します";
        robot_mode_ = STOP;
        is_track.data = false;
        path_mode.data = robot_mode_;
    } else if( input->data == "follow" ) {
        word.data = "ついじゅう走行を開始します";
        robot_mode_ = FOLLOW;
        is_track.data = true;
        path_mode.data = robot_mode_;
    } /*else if( input->data == "attention" ){
        word.data = "注目します";
        robot_mode_ = ATTENTION;
    }*/
    if (!word.data.empty()) {
        pub_speech_.publish( word );
        if( robot_mode_ != STOP )ros::Duration(5.0).sleep();
        pub_switch_tracker_.publish( is_track );
        pub_switch_path_.publish( path_mode );
    }
}

FollowMeHandle::FollowMeHandle() : nh_(), pnh_("~") {
    pub_speech_ = nh_.advertise<std_msgs::String>("/speech_word", 1);
    pub_switch_tracker_ = nh_.advertise<std_msgs::Bool>("is_track", 1);
    pub_switch_path_ = nh_.advertise<std_msgs::Int64>("path_mode", 1);
    sub_vc_ = nh_.subscribe("/multiple_voice_trigger/word", 1, &FollowMeHandle::callbackVoiceTrigger, this); 
    robot_mode_ = STOP;
    std_msgs::String word;
    word.data = "フォローというとついじゅう走行を開始します";
    pub_speech_.publish( word );
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "follow_me_handle");
	FollowMeHandle fmt;
	ros::spin();
}
