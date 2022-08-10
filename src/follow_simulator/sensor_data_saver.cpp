#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sobit_follower/sub_functions/standard_point_cloud_handle.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::LaserScan> MySyncPolicy;

namespace sds {
class SensorDataSaver {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_laser_;
        ros::Publisher pub_img_;
        mypcl::StandardPointCloudHandle pch_;
        cv_bridge::CvImagePtr cv_ptr_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_img_;
		std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
		std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_laser_;
		std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
        PointCloud::Ptr cloud_save_;
        PointCloud::Ptr laser_save_;
        cv::Mat img_save_;
        sensor_msgs::Image img_msg_;
        
        double rotate_degree_;
        std::string rotate_axis_;
        bool is_saved_;

        int kbhit(void) {
            struct termios oldt, newt;
            int ch;
            int oldf;

            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

            ch = getchar();

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            fcntl(STDIN_FILENO, F_SETFL, oldf);

            if (ch != EOF) {
                ungetc(ch, stdin);
                return 1;
            }

            return 0;
        }
        void calcRotationMatrix( Eigen::Matrix4f* rotation_matrix, const std::string &axis, const double &degree ) {
            Eigen::Matrix4f rotation_matrix_tmp;
            float theta = ( degree ) * ( M_PI / 180 );
            float cos_theta = std::cos( theta );
            float sin_theta = std::sin( theta );
            if( axis == "x" ) {
                 rotation_matrix_tmp << \
                        1,          0,            0,        0, \
                        0,  cos_theta,   -sin_theta,        0, \
                        0,  sin_theta,    cos_theta,        0, \
                        0,          0,            0,        1;
            } else if ( axis == "y" ){
                rotation_matrix_tmp << \
                cos_theta,          0,    sin_theta,        0, \
                        0,          1,            0,        0, \
               -sin_theta,          0,    cos_theta,        0, \
                        0,          0,            0,        1;
            } else if ( axis == "z" ){
                rotation_matrix_tmp << \
                cos_theta, -sin_theta,            0,        0, \
                sin_theta,  cos_theta,            0,        0, \
                        0,          0,            0,        0, \
                        0,          0,            0,        1;
            } else ROS_ERROR("This axis does not exist");
            *rotation_matrix = rotation_matrix_tmp;
        }
        void setSaveImage( const sensor_msgs::ImageConstPtr &input_img ){
            img_msg_ = *input_img;
            try {
				cv_ptr_ = cv_bridge::toCvCopy(input_img, sensor_msgs::image_encodings::BGR8);
				img_save_ = cv_ptr_->image.clone(); 
			} catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

            ROS_INFO("set new Save Image");
        }
        void setSaveCloud( const PointCloud::Ptr input_cloud ) {
            Eigen::Matrix4f rotation_matrix;
            calcRotationMatrix( &rotation_matrix, rotate_axis_, rotate_degree_ );
            pcl::transformPointCloud(*input_cloud, *input_cloud, rotation_matrix );
            cloud_save_ = input_cloud;
            ROS_INFO("set new Save Cloud");
        }
        void setSaveLaser( const PointCloud::Ptr input_laser ) {
            laser_save_ = input_laser;
            ROS_INFO("set new Save Laser");
        }
        void callbackSenserData( const sensor_msgs::ImageConstPtr &img_msg, 
                                    const sensor_msgs::PointCloud2ConstPtr &cloud_msg, 
                                    const sensor_msgs::LaserScanConstPtr &laser_msg ) {
            ROS_INFO("Get SenserData!!");
            PointCloud::Ptr cloud (new PointCloud());
            PointCloud::Ptr laser (new PointCloud());
            if ( !pch_.transformFramePointCloud( "camera_rgb_optical_frame", cloud_msg, cloud ) ) return;
            if ( !pch_.transformFramePointCloud( "base_laser_link", laser_msg, laser ) ) return;
            setSaveCloud( cloud );
            setSaveLaser( laser );
            setSaveImage( img_msg );
            ROS_INFO("\nEnter 's' in the terminal to save the sensor data.\n");
            is_saved_ = false;
        }

    public :
        SensorDataSaver() : nh_(), pnh_("~") {
            std::string img_topic_name = pnh_.param<std::string>( "image_topic_name", "/camera/rgb/image_raw" );
            std::string cloud_topic_name = pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" );
            std::string laser_topic_name = pnh_.param<std::string>( "laser_topic_name", "/scan" );
            rotate_axis_ = pnh_.param<std::string>( "rotate_axis", "x" );
            rotate_degree_ = pnh_.param<double>( "rotate_degree", 15 );

            ROS_INFO("\n* imgae : %s\n* cloud : %s\n* laser : %s\n* rotate_axis : %s\n* rotate_degree : %lf\n", 
                img_topic_name.c_str(), cloud_topic_name.c_str(), laser_topic_name.c_str(), rotate_axis_.c_str(), rotate_degree_ );
            
            pub_cloud_ = nh_.advertise<PointCloud>( "save_cloud", 1 );
            pub_laser_ = nh_.advertise<PointCloud>( "save_laser", 1 );
            pub_img_ = nh_.advertise<sensor_msgs::Image>( "save_image", 1 ); 
            sub_img_ .reset ( new message_filters::Subscriber<sensor_msgs::Image> ( nh_, img_topic_name, 1 ) );
            sub_cloud_ .reset ( new message_filters::Subscriber<sensor_msgs::PointCloud2> ( nh_, cloud_topic_name, 1 ) );
            sub_laser_ .reset ( new message_filters::Subscriber<sensor_msgs::LaserScan> ( nh_, laser_topic_name, 1 ) );
            sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(100), *sub_img_, *sub_cloud_, *sub_laser_ ) );
            sync_ ->registerCallback ( boost::bind( &SensorDataSaver::callbackSenserData, this, _1, _2, _3 ) );
            cloud_save_.reset( new PointCloud() );
            laser_save_.reset( new PointCloud() );
            is_saved_ = true;
        }

        void saveSensorData() {
            std::string save_data_path = pnh_.param<std::string>( "save_data_path", "/sensor_data" );
            std::string save_cloud_path, save_laser_path, save_img_path;
            int cnt_save = 0;
            ros::Rate rate(33);
            ROS_INFO("\nStart Save SensorData");
			while(ros::ok()){
                pub_img_.publish( img_msg_ );
				pub_laser_.publish( laser_save_ );
                pub_cloud_.publish( cloud_save_ );
                if (kbhit()) {
                    int key = getchar();
                    if ( key == 's' && !is_saved_ ) {
                        ROS_INFO("\n[ Save SensorData ]");
                        save_cloud_path = save_data_path + "/cloud/cloud_binary_" + std::to_string(cnt_save) + ".pcd";
                        save_laser_path = save_data_path + "/laser/laser_binary_" + std::to_string(cnt_save) + ".pcd";
                        save_img_path = save_data_path + "/image/image_" + std::to_string(cnt_save) + ".png";
                        if( !cloud_save_->points.empty() ) pcl::io::savePCDFileBinary<PointT> ( save_cloud_path, *cloud_save_ );
                        if( !laser_save_->points.empty() ) pcl::io::savePCDFileBinary<PointT> ( save_laser_path, *laser_save_ );
                        if (!img_save_.empty()) cv::imwrite( save_img_path, img_save_ );
                        std::cout << "* cloud : " <<  save_cloud_path
                                  << "\n* laser : " << save_laser_path
                                  << "\n* image : " << save_img_path << "\n" << std::endl;
                        cnt_save++;
                        is_saved_ = true;
                    } else if ( key == 's' && is_saved_ ) ROS_ERROR("\nI have already saved that sensor data\n");
                }
				ros::spinOnce();
                rate.sleep();
			}
        }

        void saveAutoSensorData() {
            std::string save_data_path = pnh_.param<std::string>( "save_data_path", "/sensor_data" );
            std::string save_cloud_path, save_laser_path, save_img_path;
            int cnt_save = 0;
            ros::Rate rate(50);
            ROS_INFO("\nStart Auto Save SensorData");
			while(ros::ok()){
                pub_img_.publish( img_msg_ );
				pub_laser_.publish( laser_save_ );
                pub_cloud_.publish( cloud_save_ );
                if ( !is_saved_ ) {
                    ROS_INFO("\n[ Auto Save SensorData ]");
                    save_cloud_path = save_data_path + "/cloud/cloud_binary_" + std::to_string(cnt_save) + ".pcd";
                    save_laser_path = save_data_path + "/laser/laser_binary_" + std::to_string(cnt_save) + ".pcd";
                    save_img_path = save_data_path + "/image/image_" + std::to_string(cnt_save) + ".png";
                    if( !cloud_save_->points.empty() ) pcl::io::savePCDFileBinary<PointT> ( save_cloud_path, *cloud_save_ );
                    if( !laser_save_->points.empty() ) pcl::io::savePCDFileBinary<PointT> ( save_laser_path, *laser_save_ );
                    if (!img_save_.empty()) cv::imwrite( save_img_path, img_save_ );
                    std::cout << "* cloud : " <<  save_cloud_path
                                << "\n* laser : " << save_laser_path
                                << "\n* image : " << save_img_path << "\n" << std::endl;
                    cnt_save++;
                    is_saved_ = true;
                } 
				ros::spinOnce();
                rate.sleep();
			}
        }
};
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sensor_data_saver");
    sds::SensorDataSaver sds;
    ros::NodeHandle pnh("~");
    bool is_auto_save = pnh.param<bool>( "is_auto_save", false );
    if( is_auto_save ) sds.saveAutoSensorData();
    else sds.saveSensorData();
    ros::spin();
}