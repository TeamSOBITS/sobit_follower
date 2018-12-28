#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <float.h>
#include <std_msgs/Bool.h>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

#include<trajectory_msgs/JointTrajectory.h>//HSRの首の動作用
#include<trajectory_msgs/JointTrajectoryPoint.h>//HSRの首の動作用


class target_follower_class
{
	ros::NodeHandle n;

	//subscriber
	ros::Subscriber sub_costmap_updates;
	ros::Subscriber sub_state;
	ros::Subscriber sub_map;
	ros::Subscriber sub_ctrl;

	//publisher
	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_neck_ctrl;//HSRの首の動作用

	//msgの型変数
	geometry_msgs::Point target;
	geometry_msgs::Twist msg_twist;

	//map基準の座標[pixel]の初期化
	int map_x = 0.0;
	int map_y = 0.0;
	int map_z = 0.0;

	//各パラメータの初期化
	double resolution_val = 0.0;
	double keep_distance = 2.0;//ロボットとtargetの保つ距離[m]
	int data_size = 0;//costmapのデータサイズ
	int map_width = 0;//mapの幅[pix]
	int map_height = 0;//mapの高さ[pix]

	//注意として，(possible_range * 0.5) >= axis_radiusでないとコアダンプする可能性がある
	double axis_radius = 0.4;	//base_foot_printの位置を中心とした半径(axis_radius)[m]離れた位置に軸を作成
	double possible_range = 1.2;	//base_foot_print基準とした画像で表示する時のサイズ(possible_range　×　possible_range)

	tf::TransformListener listener;

	//移動希望フラグ
	int front_hope_flag;
	int left_hope_flag;
	
	//移動可能フラグ
	bool front_flag = true;
	bool behind_flag = true;
	bool right_flag = true;
	bool left_flag = true;

	//mapのコスト付与終了するまでfalse
	bool map_copy_flag = false;

	//falseの場合ロボットの移動速度とhsrのpanのパブリッシュを終了
	bool hsr_ctrl_flag = true;

	//コスト付与した地図
	cv::Mat white_costmap_img_to_show = cv::Mat(cv::Size(map_width, map_height), CV_8UC3, cv::Scalar(255,255,255));


public:
	target_follower_class(){

		//targetの位置座標をサブスクライブ
		this->sub_state = n.subscribe("/publish_state",1000, &target_follower_class::target_callback, this);

		//mapのマップデータをサブスクライブ
		this->sub_map = n.subscribe("/map_movebase",1,&target_follower_class::map_callback,this);

		//ロボットの車輪移動速度をパブリッシュ
		this->pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hsrb/opt_command_velocity",1);

		//HSRの首の角度をパブリッシュ
		this->pub_neck_ctrl = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 1 );

		//hsrのステートパブリッシュの可否
		this->sub_ctrl = n.subscribe("/hsr_ctrl_flag",1, &target_follower_class::hsr_ctrl, this);

		ROS_INFO("target_following_stand_up");



	}//target_follower_class

	~target_follower_class(){}

void target_callback(const geometry_msgs::Point::ConstPtr& msg_state)//ロボットの首の調整と移動速度の送信
	{
		if(map_copy_flag == false)	{return;}
		double target_rad_2d = 0.0;//ロボットから見たtargetの角度[rad]
		double target_distance_2d = 0.0;//ロボットとtargetの距離[m]
		this->target = *msg_state;
		target_rad_2d = atan2f( this->target.y , this->target.x );
		target_distance_2d = hypotf( this->target.x , this->target.y );
		cv::Mat costmap_img = cv::Mat(cv::Size(this->map_width, this->map_height), CV_8UC3, cv::Scalar(255,255,255));
		costmap_img = this->white_costmap_img_to_show.clone();
		
		//map基準から見たbase_footprintの座標系
		tf::StampedTransform transform;
		try{
	  		this->listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
	  		ROS_ERROR("%s",ex.what());
	  		ros::Duration(1.0).sleep();
		}

		//mapの位置座標を緑色の円で表示
		circle(costmap_img, cv::Point(this->map_x, this->map_y), 0.3, cv::Scalar(0,255,0), 3, CV_AA);

		//base_footprint位置座標[pix]
		int base_footprint_point_x = this->map_x + transform.getOrigin().x() / this->resolution_val;
		int base_footprint_point_y = this->map_y + transform.getOrigin().y() / this->resolution_val;

		//target[m]をpix単位に変換
		int target_point_x	= base_footprint_point_x + this->target.x / this->resolution_val;
		int target_point_y	= base_footprint_point_y + this->target.y / this->resolution_val;

		//mapの位置座標から移動したロボットの位置座標を青色で表示
		circle(costmap_img, cv::Point(base_footprint_point_x, base_footprint_point_y ), 0.3, cv::Scalar(255,0,0), 3, CV_AA);

		//base_footprintを中心とした時のlocal_areaにおける左上の点[pix]
		int local_base_point_x = base_footprint_point_x - this->possible_range * 0.5 / this->resolution_val;
		int local_base_point_y = base_footprint_point_y - this->possible_range * 0.5 / this->resolution_val;

		//global基準の移動可能領域の軸[pix]
		int global_base_bottom_axis = base_footprint_point_x - this->axis_radius / this->resolution_val;
		int global_base_upper_axis = base_footprint_point_x + this->axis_radius / this->resolution_val;
		int global_base_right_axis = base_footprint_point_y - this->axis_radius / this->resolution_val;
		int global_base_left_axis = base_footprint_point_y + this->axis_radius / this->resolution_val;

		//local基準における軸範囲[pix]
		int possible_range_pixel = this->possible_range / this->resolution_val;

		//local基準におけるbase_footprint位置座標[pix]
		int local_base_footprint_point_x = base_footprint_point_x - local_base_point_x;
		int local_base_footprint_point_y = base_footprint_point_y - local_base_point_y;

		//local基準の移動可能領域の軸[pix]
		int local_base_bottom_axis =  local_base_footprint_point_x - (base_footprint_point_x - global_base_bottom_axis);
		int local_base_upper_axis = local_base_footprint_point_x + (global_base_upper_axis - base_footprint_point_x);
		int local_base_right_axis = local_base_footprint_point_y - (base_footprint_point_y - global_base_right_axis);
		int local_base_left_axis = local_base_footprint_point_y + (global_base_left_axis - base_footprint_point_y);
		
		//base_footprintを基準とした幅(possible_range_pixel)[pix]の画像をlocal_area_mapとする
		cv::Mat local_area_map = costmap_img(cv::Rect(local_base_point_x, local_base_point_y, possible_range_pixel,possible_range_pixel));

		//local_area_mapにおける障害物(赤色の領域)を算出
		for(int i = 0; i < local_area_map.rows * local_area_map.cols; i++)//7
		{
			int pixel_x = i % local_area_map.cols;
			int pixel_y = i / local_area_map.cols;

			//pixel_y,pixel_xにおけるRGBの輝度値
			int pixel_val_b = local_area_map.at<cv::Vec3b>( pixel_y , pixel_x )[0];
			int pixel_val_g = local_area_map.at<cv::Vec3b>( pixel_y , pixel_x )[1];
			int pixel_val_r = local_area_map.at<cv::Vec3b>( pixel_y , pixel_x )[2];

			if(pixel_val_b < 125 && pixel_val_g < 125)//8 赤い場合
			{
				//9 前方に障害物があった場合 && front_flag == true
				if( local_base_upper_axis < pixel_x && local_base_left_axis > pixel_y && local_base_right_axis < pixel_y)
				{
					//std::cout << "upper side bad" << std::endl;
					this->front_flag = false;
				}//9
				//10 後方に障害物があった場合 && behind_flag == true
				if( local_base_bottom_axis > pixel_x && local_base_left_axis > pixel_y && local_base_right_axis < pixel_y)
				{
					//std::cout << "bottom side bad" << std::endl;
					this->behind_flag = false;
				}//10
				//11 左側に障害物があった場合 && left_flag == true
				if( local_base_left_axis < pixel_y && local_base_upper_axis > pixel_x && local_base_bottom_axis < pixel_x)
				{
					//std::cout << "left side bad" << std::endl;
					this->left_flag = false;
				}//11
				//12 右側に障害物があった場合 && right_flag == true
				if( local_base_right_axis > pixel_y && local_base_upper_axis > pixel_x && local_base_bottom_axis < pixel_x)
				{
					//std::cout << "right side bad" << std::endl;
					this->right_flag = false;
				}//12

			}//8
		}//7

		trajectory_msgs::JointTrajectoryPoint point;
		trajectory_msgs::JointTrajectory send_data;

		point.time_from_start = ros::Duration(2);
		point.positions.push_back( target_rad_2d );
		send_data.joint_names.push_back( "head_pan_joint" );	//フレーム
		send_data.points.push_back( point );	//hsrの首を傾ける角度[rad]をpush

		//targetとロボットの距離を調整する各パラメータ
		double possible_distance = 0.2;
		double keep_distance = 2.2;
		double keep_distance_x = fabs(keep_distance * cos(target_rad_2d));
		double keep_distance_y = fabs(keep_distance * sin(target_rad_2d));

		//移動方向の決定
		target_follower_class::decision_of_movement_direction(this->target.x, keep_distance_x, this->target.y, keep_distance_y);

		if(this->front_flag == false)		//ロボットの前方に障害物がある場合
			this->front_flag = true;
		if(this->behind_flag == false)     //ロボットの後ろに障害物がある場合
			this->behind_flag = true;
		if(this->right_flag == false)		//ロボットの右側に障害物がある場合
			this->right_flag = true;
		if(this->left_flag == false)		//ロボットの左側に障害物がある場合
			this->left_flag = true;


		if(this->hsr_ctrl_flag == true)
		{
			this->pub_neck_ctrl.publish(send_data);//hsrのsend_dataをpublish
			this->pub_cmd_vel.publish(msg_twist);	//ロボットの移動速度をpublish
		}
	
		//表示させる地図をmap基準のx-y座標系に合わせる
		//cv::transpose(costmap_img, costmap_img);//時計回りに90度回転
	   	//cv::flip(costmap_img, costmap_img, -1);//水平軸と垂直軸で反転
		//cv::transpose(local_area_map, local_area_map);//時計回りに90度回転
	   	//cv::flip(local_area_map, local_area_map, -1);//水平軸と垂直軸で反転


		//表示させるlocal_mapのサイズを変更する
		//cv::resize(local_area_map, local_area_map, cv::Size(), 10.0 ,10.0);
	
		//各地図の表示
		//cv::imshow("local_area_map", local_area_map);//可能領域(possible_range　×　possible_range[m])
		//cv::imshow("cost_grant_map_img", costmap_img);//map全体

		//キーの入力待ち
		//cv::waitKey(40);	

	}//target_info

void map_callback(const nav_msgs::OccupancyGrid& msg)
	{
		int cost_radius = 1.0;
		cv::Mat white_costmap_img;
		this->map_width = msg.info.width;
		this->map_height = msg.info.height;
		this->data_size = msg.data.size();
		this->resolution_val = msg.info.resolution;//[m/pix]

		//map基準の座標をpixel値に変換
		this->map_x = (0.0 - msg.info.origin.position.x) / msg.info.resolution;
		this->map_y = msg.info.height - (0.0 - msg.info.origin.position.y) / msg.info.resolution;

		//最大サイズで画像の枠を作っておく
		white_costmap_img = cv::Mat(cv::Size(this->map_width, this->map_height), CV_8UC3, cv::Scalar(255,255,255));

		//mapから地図を読み取る
		for(int i = 0; i < this->data_size; i++)//1
		{
			if(msg.data[i] == 0)//2
			{
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[0] = 255;
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[1] = 255;
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[2] = 255;

			}//2
			else//3
			{
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[0] = 255 - msg.data[i];
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[1] = 255 - msg.data[i];
				white_costmap_img.at<cv::Vec3b>(i / this->map_width, i % this->map_width)[2] = 255 - msg.data[i];
			}//3
		}//1


		//読み取った地図にコストを付与する
		for(int i = 0; i <= map_width; i++)//4
		{
			for(int j = 0; j <= map_height;j++)//5
			{
				if(white_costmap_img.at<cv::Vec3b>(i, j)[0] != 255 && white_costmap_img.at<cv::Vec3b>(i, j)[1] != 255 && white_costmap_img.at<cv::Vec3b>(i, j)[2] != 255)//6
				{
					circle(white_costmap_img, cv::Point(j , i), cost_radius , cv::Scalar(0,0,255), 3, CV_AA);//黒色の点にコスト(赤色)を付与する
				}//6
			}//5
		}//4

		//white_costmap_img_to_showに画像をコピー
		white_costmap_img.copyTo(this->white_costmap_img_to_show);
		this->map_copy_flag = true;
		ROS_INFO("map copy succeded");
	}//map_callback

	void hsr_ctrl(const std_msgs::Bool& input)
	{
		geometry_msgs::Twist init_twist;
		this->hsr_ctrl_flag = input.data;
		init_twist.linear.x = 0.0;
		init_twist.linear.y = 0.0;
		this->pub_cmd_vel.publish(init_twist);
	}//hsr_ctrl

	void decision_of_movement_direction(double target_x, double keep_distance_x, double target_y, double keep_distance_y)
	{
		double front_of_target_range = 0.2;

		//移動希望方向の設定
		if(target_x - keep_distance_x > 0) //人がロボットに対して遠ざかる場合
		{
			this->front_hope_flag = 1;
		}
		else if(fabs(target_x - keep_distance_x) <= front_of_target_range) //人がある一定の距離にいる場合
		{
			this->front_hope_flag = 0;
		}
		else	//人がロボットに対して近づいて来る場合
		{
			this->front_hope_flag = -1;
		}
		if(target_y - keep_distance_y > 0)	//人がロボットに対して左側にいる場合
		{
			this->left_hope_flag = 1 * this->front_hope_flag;
		}
		else if(fabs(fabs(target_y) - keep_distance_y) <= front_of_target_range) //人がロボットに対して中央にいる場合
		{
			this->left_hope_flag = 0 * this->front_hope_flag;
		}
		else	//人がロボットに対して右側にいる場合
		{
			this->left_hope_flag = -1 * this->front_hope_flag;
		}

		//移動方向の決定
		if(this->front_flag == true && this->front_hope_flag == 1)	//前へ移動可能なら前進
		{
			//ROS_INFO("move to front direction");
			this->msg_twist.linear.x = 0.1;
		}
		else if(this->behind_flag == true && this->front_hope_flag == -1)	//後ろへ移動可能なら後退
		{
			//ROS_INFO("move to behind direction");
			this->msg_twist.linear.x = -0.1;
		}
		else	//前後移動できないorある一定の距離である場合は停止
		{
			//ROS_INFO("stop[x direction]");
			this->msg_twist.linear.x = 0.0;
		}

		if(this->right_flag == true && (this->left_hope_flag == -1 || (this->behind_flag == false && this->left_hope_flag == 0)))//右へ移動可能(決め打ち)なら右へ進む
		{
			//ROS_INFO("move to right direction");
			this->msg_twist.linear.y = -0.1;
		}
		else if(this->left_flag == true && this->left_hope_flag == 1)	//左へ移動可能なら左へ進む
		{
			//ROS_INFO("move to left direction");
			this->msg_twist.linear.y = 0.1;
		}
		else	//左右移動できないor中央にいる場合は停止
		{
			//ROS_INFO("stop[y direction]");
			this->msg_twist.linear.y = 0.0;
		}
		if(this->front_hope_flag == 1 && this->left_hope_flag != 0 && (this->msg_twist.linear.x != 0.0 && this->msg_twist.linear.y != 0.0))	//y方向優先
			this->msg_twist.linear.x = 0.0;
		else if(this->front_hope_flag == -1 && (this->msg_twist.linear.x != 0.0 && this->msg_twist.linear.y != 0.0))	//x方向優先
			this->msg_twist.linear.y = 0.0;

	}//decision_of_movement_direction


};//target_follower_class







int main(int argc,char **argv)
{

	ros::init(argc,argv,"target_following");
	target_follower_class psc;
	ros::spin();
	return 0;
}







