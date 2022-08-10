#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sobit_follower/sub_functions/standard_point_cloud_handle.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace sdp {
    class SensorDataPublisher {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_cloud_;
            ros::Publisher pub_laser_;
            ros::Publisher pub_img_;

            bool readSensorData( 
                sensor_msgs::PointCloud2* cloud_msg, 
                sensor_msgs::PointCloud2* laser_msg, 
                cv_bridge::CvImage* img_msg, 
                const std::string& pub_data_path, const int cnt_pub ) {
                std::string pub_cloud_path, pub_laser_path, pub_img_path;
                pub_cloud_path = pub_data_path + "/cloud/cloud_binary_" + std::to_string(cnt_pub) + ".pcd";
                pub_laser_path = pub_data_path + "/laser/laser_binary_" + std::to_string(cnt_pub) + ".pcd";
                pub_img_path = pub_data_path + "/image/image_" + std::to_string(cnt_pub) + ".png";

                std::cout << "[ Data path to be loaded ]"
                            << "\n* cloud : " <<  pub_cloud_path
                            << "\n* laser : " << pub_laser_path
                            << "\n* image : " << pub_img_path << "\n" << std::endl;

                ros::Time curt_stamp = ros::Time::now();
                if ( !readCloud( cloud_msg, pub_cloud_path, curt_stamp ) ) return false;
                if ( !readLaser( laser_msg, pub_laser_path, curt_stamp ) ) return false;
                if ( !readImage( img_msg, pub_img_path, curt_stamp ) ) return false;
                return true;
            }
            bool readCloud( sensor_msgs::PointCloud2* cloud_msg, const std::string& pub_cloud_path, ros::Time curt_stamp ) {
                PointCloud::Ptr cloud (new PointCloud());
                if (pcl::io::loadPCDFile<pcl::PointXYZ> ( pub_cloud_path, *cloud) == -1){
                    PCL_ERROR ( "Couldn't read file %s.\n", pub_cloud_path.c_str() );
                    return false;
                }
                pcl::toROSMsg(*cloud, *cloud_msg );
                cloud_msg->header.frame_id = "camera_rgb_optical_frame";
                cloud_msg->header.stamp = curt_stamp;
                return true;
            }
            bool readLaser( sensor_msgs::PointCloud2* laser_msg, const std::string& pub_laser_path, ros::Time curt_stamp ) {
                PointCloud::Ptr laser (new PointCloud());
                if (pcl::io::loadPCDFile<pcl::PointXYZ> ( pub_laser_path, *laser) == -1){
                    PCL_ERROR ( "Couldn't read file %s.\n", pub_laser_path.c_str() );
                    return false;
                }
                pcl::toROSMsg(*laser, *laser_msg );
                laser_msg->header.frame_id = "base_laser_link";
                laser_msg->header.stamp = curt_stamp;
                return true;   
            }
            bool readImage( cv_bridge::CvImage* img_msg, const std::string& pub_img_path, ros::Time curt_stamp ) {
                cv::Mat image = cv::imread( pub_img_path );
                img_msg->encoding = sensor_msgs::image_encodings::BGR8;
                img_msg->header.frame_id = "camera_rgb_optical_frame";
                img_msg->header.stamp = curt_stamp;
                img_msg->image = image.clone();
                return true;
            }

        public :
            SensorDataPublisher( ) : nh_(), pnh_("~") {
                pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
                pub_laser_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_laserscan", 1);
                pub_img_ = nh_.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 1);
            }

            void publishSensorData() {
                std::string pub_data_path = pnh_.param<std::string>( "publish_data_path", "/sensor_data" );
                int init_num_data = pnh_.param<int>( "initial_number_of_data", 0 );
                int last_num_data = pnh_.param<int>( "last_number_of_data", 0 );
                double switch_time = pnh_.param<double>( "switching_time", 1.0 );
                double hz = pnh_.param<double>( "hz", 30.0 );
                sensor_msgs::PointCloud2 cloud_msg;
                sensor_msgs::PointCloud2 laser_msg;
                cv_bridge::CvImage img_msg;
                std::vector<sensor_msgs::PointCloud2> cloud_msgs;
                std::vector<sensor_msgs::PointCloud2> laser_msgs;
                std::vector<cv_bridge::CvImage> img_msgs;
                for( int num_data = init_num_data; num_data <= last_num_data; num_data++  ) {
                    readSensorData( &cloud_msg, &laser_msg, &img_msg, pub_data_path, num_data );
                    cloud_msgs.push_back( cloud_msg );
                    laser_msgs.push_back( laser_msg );
                    img_msgs.push_back( img_msg );
                }
                bool ascending_order = true;
                int cnt_num_data = 0;
                int msgs_size =  cloud_msgs.size() - 1;
                double curt_time = ros::Time::now().toSec();
                double pre_time = curt_time;
                ROS_INFO( "Start to publish sensor datas !!" );
                ROS_INFO("Publish the data of NO.%d",init_num_data );
                ros::Rate rate(hz);
			    while(ros::ok()) {
                    ros::Time curt_stamp = ros::Time::now();
                    cloud_msgs[ cnt_num_data ].header.stamp = curt_stamp;
                    laser_msgs[ cnt_num_data ].header.stamp = curt_stamp;
                    img_msgs[ cnt_num_data ].header.stamp = curt_stamp;
                    pub_cloud_.publish( cloud_msgs[ cnt_num_data ] );
                    pub_laser_.publish( laser_msgs[ cnt_num_data ] );
                    pub_img_.publish( img_msgs[ cnt_num_data ].toImageMsg() );
                    if( init_num_data != last_num_data  && curt_time - pre_time > switch_time ) {
                        if( ascending_order ) {
                            cnt_num_data++; 
                            if ( cnt_num_data == msgs_size ) ascending_order = false;
                        } else {
                            cnt_num_data--; 
                            if ( cnt_num_data == 0 ) ascending_order = true;
                        } 
                        pre_time = curt_time;
                        ROS_INFO("Publish the data of NO.%d",init_num_data+cnt_num_data );
                    }
                    curt_time = ros::Time::now().toSec();
                    ros::spinOnce();
                    rate.sleep();
                }
            }
    };
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sensor_data_publisher");
    sdp::SensorDataPublisher sdp;
    sdp.publishSensorData();
    ros::spin();
}