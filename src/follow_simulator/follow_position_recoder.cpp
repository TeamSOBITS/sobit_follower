#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h> 
#include <geometry_msgs/PointStamped.h>
#include <sobit_follower/FollowPosition.h>
#include <fstream>

constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;

std::ofstream writing_file;

class FollowPositionRecoder {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        tf::TransformListener tf_listener_;
        ros::Publisher pub_marker_;
        ros::Publisher pub_trajectory_;
		ros::Subscriber sub_fp_;
        visualization_msgs::Marker fp_mrk_;
        visualization_msgs::Marker trajectory_;
        int cnt_fp_;
        geometry_msgs::Point pre_pt;
        std::string filename_;

        geometry_msgs::Point transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
            geometry_msgs::PointStamped pt_transformed;
            geometry_msgs::PointStamped pt;
            pt.header.frame_id = org_frame;
            pt.header.stamp = ros::Time(0);
            pt.point = point;
            if ( tf_listener_.frameExists( target_frame ) ) {
                try {
                    tf_listener_.transformPoint( target_frame, pt, pt_transformed );
                } catch ( tf::TransformException ex ) {
                    ROS_ERROR( "%s",ex.what( ) );
                }
            } else {
                ROS_ERROR("target_frame is not Exists");
            }
            return pt_transformed.point;
        }

        void callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input ) { 
            geometry_msgs::Point tmp;
            tmp = transformPoint ( "base_footprint", "odom", input->target);
            writing_file << tmp.x << "\t" << tmp.y << std::endl;
            std::cout << tmp.x << "\t" << tmp.y << std::endl;
            /*tmp.z = 0.1;
            fp_mrk_.points.push_back( tmp );
            cnt_fp_++;
            if ( cnt_fp_ > 1 ) {
                geometry_msgs::Point tmp2;
                double alpha = 0.95;
                tmp2.x = alpha * pre_pt.x + ( 1 - alpha ) * tmp.x;
                tmp2.y = alpha * pre_pt.y + ( 1 - alpha ) * tmp.y;
                tmp2.z = 0.1;      
                trajectory_.points.push_back( tmp2 );
                pre_pt = tmp2;
            } else {
                pre_pt = tmp;
            }
            
            if( input->process_flag == DETECTION ) {
                fp_mrk_.points.clear();
                trajectory_.points.clear();
                cnt_fp_ = 0;
            }
            if( input->process_flag != DETECTION && tmp.x == 0.0 && tmp.y == 0.0 ) ROS_WARN("Target position doesn't exist.");
            std::cout << "[ FollowPosition Data]"
                << "\ntarget        : x = " << input->target.x << " [m], y = " << input->target.y << " [m]"
                << "\nexists_target : " << ( input->exists_target ? "TRUE" : "FALSE" ) 
                << "\nprocess_flag  : " << ( input->process_flag == DETECTION ? "DETECTION" : ( input->process_flag == TRACKING ? "TRACKING" :"PREDICTION" ) ) << "\n"
            << std::endl;
            */
        }

    public :
        FollowPositionRecoder() : nh_(), pnh_("~") {
            fp_mrk_.header.frame_id = "base_footprint";
            fp_mrk_.header.stamp = ros::Time::now();
            fp_mrk_.ns = "optimal_path";
            fp_mrk_.id = 1;
            fp_mrk_.type = visualization_msgs::Marker::LINE_STRIP;
            fp_mrk_.action = visualization_msgs::Marker::ADD;
            fp_mrk_.lifetime = ros::Duration(1.0);
            fp_mrk_.scale.x = 0.05;
            fp_mrk_.color.a = 1.0;
            fp_mrk_.color.r = 0.0;
            fp_mrk_.color.g = 1.0;
            fp_mrk_.color.b = 1.0;
            trajectory_ = fp_mrk_;
            trajectory_.color.r = 1.0;
            trajectory_.color.g = 1.0;
            trajectory_.color.b = 0.0;
            cnt_fp_ = 0;
            pub_marker_ = nh_.advertise<visualization_msgs::Marker>("follow_position_marker", 1);
            pub_trajectory_ = nh_.advertise<visualization_msgs::Marker>("trajectory_marker", 1);
            sub_fp_ = nh_.subscribe( "/follow_position", 1, &FollowPositionRecoder::callbackFollowPosition, this );
            ros::param::get("filename", filename_);  
            writing_file.open(filename_, std::ios::out);
        }

        void displayFollowPosition() {
            ros::Rate rate(15);
            while(ros::ok()){
                fp_mrk_.header.stamp = ros::Time::now();
                pub_marker_.publish ( fp_mrk_ );
                pub_trajectory_.publish ( trajectory_ );
                ros::spinOnce();
                rate.sleep();
            }
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follow_position_recoder");
    FollowPositionRecoder fpr;
    fpr.displayFollowPosition();
    ros::spin();
}