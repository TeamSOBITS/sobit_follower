#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <random>
#include<bits/stdc++.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sobit_follower/FollowPosition.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class FollowPosition {
    private :
        sobit_follower::FollowPositionPtr follow_position_;
        double dist_nearby_obs_;
    public :
        FollowPosition ( ) { 
            follow_position_.reset ( new sobit_follower::FollowPosition );
            follow_position_->is_nearby_obstacle = false;
            dist_nearby_obs_ = 0.35;
            initFollowPosition ( ); 
        }
        void initFollowPosition (  ) { 
            follow_position_->obstacles.clear();
            follow_position_->is_nearby_obstacle = false;
        }
        void setDistanceNearbyObstacle ( double distance ) { dist_nearby_obs_ = distance; }
        void setTarget( const geometry_msgs::Point target  ) { 
            follow_position_->target = target; 
            follow_position_->target_distance = std::hypotf( target.x, target.y);
			follow_position_->target_angle = std::atan2 ( target.y, target.x );
        }
        void setExistsTarget ( bool exists ) { follow_position_->exists_target = exists; }

        void addObstacle ( const PointCloud::Ptr obstacle_cloud ) {
            sobit_follower::FollowPosition tmp_fp = *follow_position_;
            double dist_nearby_obs = dist_nearby_obs_;
            bool is_nearby_obstacle = false;
            for ( auto& pt : obstacle_cloud->points ) {
                sobit_follower::Obstacle obstacle;
                obstacle.center.x = pt.x;
                obstacle.center.y = pt.y;
                geometry_msgs::Point tmp_pt;
                tmp_pt.x = pt.x;
                tmp_pt.y = pt.y;
                obstacle.points.push_back( tmp_pt ); 

                double dist = std::hypotf( pt.x, pt.y );
                if( dist < dist_nearby_obs ) is_nearby_obstacle = true; 

                tmp_fp.obstacles.push_back( obstacle );
            }
            if ( is_nearby_obstacle ) tmp_fp.is_nearby_obstacle = is_nearby_obstacle;
            *follow_position_ = tmp_fp;
        }
        sobit_follower::FollowPosition getFollowPosition ( ) { return *follow_position_; }
};

class FollowSimulator {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_tgt_pose_;
		ros::Publisher pub_mrk_obs_;
        ros::Publisher pub_mrk_tgt_;
        ros::Publisher pub_follow_pos_;
        tf::TransformListener tf_listener_;
        PointCloud::Ptr cloud_obs_base_;
        geometry_msgs::Point tgt_pt_base_;
        FollowPosition fp_;
         
        bool setObstacle ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud )  {
            try {
                tf_listener_.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud("base_footprint", ros::Time(0), *input_cloud, "map",  *output_cloud, tf_listener_);
                output_cloud->header.frame_id = "base_footprint";
                return true;
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return false;
            }
        }
        geometry_msgs::Point transformPoint ( geometry_msgs::Point point ) {
			geometry_msgs::PointStamped pt_transformed;
			geometry_msgs::PointStamped pt;
			pt.header.frame_id = "map";
			pt.header.stamp = ros::Time(0);
			pt.point = point;
			if ( !tf_listener_.frameExists( "base_footprint" ) ) {
				ROS_ERROR("base_footprint is not Exists");
			}
            try {
                tf_listener_.transformPoint( "base_footprint", pt, pt_transformed );
            } catch ( tf::TransformException ex ) {
                ROS_ERROR( "%s",ex.what( ) );
            }
			return pt_transformed.point;
		}

        void displayObstacleMarker ( const PointCloud::Ptr obstacle_cloud ) {
            int marker_num = 0;
			visualization_msgs::MarkerArray marker_array;
			for ( auto& pt : obstacle_cloud->points ) {
				visualization_msgs::Marker obs;
				obs.header.frame_id = "map";
				obs.header.stamp = ros::Time::now();
                obs.type = visualization_msgs::Marker::SPHERE;
                obs.action = visualization_msgs::Marker::ADD;
				obs.ns = "obstacles";
				obs.id = marker_num;
                obs.pose.position.x = pt.x;
                obs.pose.position.y = pt.y;
                obs.pose.position.z = pt.z;
                obs.pose.orientation.x = 0.0;
                obs.pose.orientation.y = 0.0;
                obs.pose.orientation.z = 0.0;
                obs.pose.orientation.w = 1.0;
                obs.scale.x = 0.1;
                obs.scale.y = 0.1;
                obs.scale.z = 0.05;
                obs.color.r = 0.0f;
                obs.color.g = 1.0f;
                obs.color.b = 0.0f;
                obs.color.a = 1.0f;
                marker_num++;
				marker_array.markers.push_back( obs );

                visualization_msgs::Marker cost;
				cost.header.frame_id = "map";
				cost.header.stamp = ros::Time::now();
                cost.type = visualization_msgs::Marker::SPHERE;
                cost.action = visualization_msgs::Marker::ADD;
				cost.ns = "cost";
				cost.id = marker_num;
                cost.pose.position.x = pt.x;
                cost.pose.position.y = pt.y;
                cost.pose.position.z = pt.z;
                cost.pose.orientation.x = 0.0;
                cost.pose.orientation.y = 0.0;
                cost.pose.orientation.z = 0.0;
                cost.pose.orientation.w = 1.0;
                cost.scale.x = 0.35;
                cost.scale.y = 0.35;
                cost.scale.z = 0.01;
                cost.color.r = 1.0f;
                cost.color.g = 0.0f;
                cost.color.b = 0.0f;
                cost.color.a = 1.0f;
                marker_num++;
				marker_array.markers.push_back( cost );
			}
            pub_mrk_obs_.publish( marker_array );
        }

        void displayTargetMarker ( const geometry_msgs::Point &target_pt ) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "target_pose";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position = target_pt;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            pub_mrk_tgt_.publish( marker );

            return;
        }

        void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg ) {
            tgt_pt_base_.x = msg->pose.position.x;
            tgt_pt_base_.y = msg->pose.position.y;
            ROS_INFO( "\nSet a new destination : ( x = %.3f, y = %.3f )\n", tgt_pt_base_.x, tgt_pt_base_.y );
            return;
        }

    public :
        FollowSimulator ( ) : nh_(), pnh_("~") {
            int obs_size = pnh_.param<int>( "obstacle_size", 50 );

            cloud_obs_base_ .reset ( new PointCloud() );

            pub_mrk_obs_ = nh_.advertise< visualization_msgs::MarkerArray >( "obstacle_marker", 1 );
            pub_mrk_tgt_ = nh_.advertise< visualization_msgs::Marker >( "target_marker", 1 );
            pub_follow_pos_ = nh_.advertise<sobit_follower::FollowPosition>("follow_position", 1);

            cloud_obs_base_->header.frame_id = "map";
            cloud_obs_base_->width  = obs_size;
            cloud_obs_base_->height = 1;
            cloud_obs_base_->points.resize (cloud_obs_base_->width * cloud_obs_base_->height);

            std::random_device rnd;
            std::mt19937 mt(rnd()); 
            std::uniform_real_distribution<> rand5( -5.0, 5.0); 
            for ( auto& obs : cloud_obs_base_->points ) {
                obs.x = rand5(mt);
                obs.y = rand5(mt);
                obs.z = 0.0;

                obs.x = ( std::fabs(obs.x) < 0.4 ) ? ( obs.x > 0 ) ? obs.x + 0.4 : obs.x - 0.4 : obs.x;
                obs.y = ( std::fabs(obs.y) < 0.4 ) ? ( obs.y > 0 ) ? obs.y + 0.4 : obs.y - 0.4 : obs.y;
            }

            tgt_pt_base_.x = 5.0;
            tgt_pt_base_.y = 1.0;

            sub_tgt_pose_ = nh_.subscribe( "/move_base_simple/goal", 10, &FollowSimulator::callbackTarget, this );
        }

        void main( ) {
            ros::Rate rate(30);

            ROS_INFO( "\n< Start path generation simulation > \
                        \nDestination     : ( x = %.3f, y = %.3f )\
                        \nObstacle points : %zu\
                        \n\nYou can set a new destination with ''2D Nav Goal( rviz top tab )''\n"
                        , tgt_pt_base_.x, tgt_pt_base_.y, cloud_obs_base_->points.size() );
               

            while(ros::ok()){
                PointCloud::Ptr cloud_obs (new PointCloud() );
                setObstacle ( cloud_obs_base_,  cloud_obs);
                geometry_msgs::Point tgt_pt = transformPoint ( tgt_pt_base_ ); 
                fp_.setExistsTarget( true );
                fp_.setTarget( tgt_pt );
                fp_.addObstacle( cloud_obs );
                displayObstacleMarker ( cloud_obs_base_ );
                displayTargetMarker ( tgt_pt_base_ );
                pub_follow_pos_.publish( fp_.getFollowPosition ( ) );
                fp_.initFollowPosition();
                ros::spinOnce();
                rate.sleep();
            }
        }


};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follow_simulator_node");
	FollowSimulator follow_simulator;
    follow_simulator.main();
    ros::spin();
}