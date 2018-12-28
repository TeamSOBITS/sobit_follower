#include <stdio.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <time.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <limits>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <float.h>
#include <vector>



#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <std_msgs/Header.h>



#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_msgs/TFMessage.h>



#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/Pose2D.h>


#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/SegmentObstacle.h>
#include <obstacle_detector/CircleObstacle.h>


//#include <turtlebot_op/odom_base.h>//kobuki ctrl

//#include <boost/thread/thread.hpp>
//#include <waypoint_nav/location_stock.h>
//#include <waypoint_nav/waypoint_nav_srv.h>
//#include <boost/thread/thread.hpp>
//#include <typeinfo>
//#include <ros/console.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/shared_ptr.hpp>
#include <typeinfo>
#include <target_scan_follower/point_states.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;


//define
#define nsec2sec 1e-9
#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180


struct point_xy {
		double x;
		double y;
};


class target_follower_class
{
	ros::NodeHandle nh;
	//SUB
	ros::Subscriber sub_ctrl;
	ros::Subscriber sub_obstacle_point;
	ros::Subscriber sub_goal;
	ros::Subscriber sub_map_info;
	//PUB
	ros::Publisher pub_twist;
	ros::Publisher pub_speech_word;
	ros::Publisher pub_target_person_marker;
	ros::Publisher pub_state;

	bool execute_flag;


	nav_msgs::OccupancyGrid odom_info;

	//tf::TransformBroadcaster br;
	tf::TransformListener listener;
	std::string robot_position_frame_name;
	std::string odom_frame_name;
	std::string urg_frame_name;
	std::string map_frame_name;


	double max_move_speed;
	double back_speed;
	double max_trun_speed;
	double max_move_distance;
	double keep_distance;
	double stop_distance;
	double constant_velocity_distance;
	double constant_velocity_deg;


	double Possible_range_x = 0.0;
	double Possible_range_y = 0.0;





	bool init_target_point_flag;
	int lost_target_count;
	geometry_msgs::Point last_target_point;
	double distance_from_last_target_point_limit;
	double last_target_distance_2d;

	double init_target_distance ;//[m]

	std::vector<double> old_speed_vec;
	bool old_speed_vec_size;





public:
	target_follower_class(){
		old_speed_vec.push_back( 0.0 );
		lost_target_count = 0;
		init_target_point_flag = true;
		execute_flag = true;





		max_move_speed = 1.0;//0.7; //[cm/s] //直進速度
		back_speed = -0.4; //[cm/s] //後退速度
		max_trun_speed = 120.0; //[deg/s] //回転速度
		max_move_distance = 5.0;//[m] //これ以上離れたら追跡しない距離（間違って計測されたとき用）
		keep_distance = 0.70;//[m] 保つ距離
		init_target_distance = 0.7;//[m]
		stop_distance = 0.02;//±[m] 保つ距離から一定距離内の小さな変化は無視して止まる
		constant_velocity_distance = 2.0;//[m] //この距離以内なら比例的に移動。これ以上離れたら等速で移動
		constant_velocity_deg = max_trun_speed*0.25;//30.0;//[deg] //この角度以内なら比例的に回転。これ以上の角度なら等速で回転
		distance_from_last_target_point_limit = 0.30; //[m]
		ros::param::get( "max_move_speed", max_move_speed );
		ros::param::get( "back_speed", back_speed );
		ros::param::get( "max_trun_speed", max_trun_speed );
		ros::param::get( "max_move_distance", max_move_distance );
		ros::param::get( "keep_distance", keep_distance );
		ros::param::get( "init_target_distance", init_target_distance );
		ros::param::get( "stop_distance", stop_distance );
		ros::param::get( "distance_from_last_target_point_limit", distance_from_last_target_point_limit );

		//old_speed_vec_size = 3;
		//ros::param::get( "old_speed_vec_size", old_speed_vec_size );

		robot_position_frame_name = "base_footprint";
		odom_frame_name = "odom";
		ros::param::get( "odom_frame_name", odom_frame_name );
		ros::param::get( "robot_position_frame_name", robot_position_frame_name );


		//std::string pub_twist_topic_name = "/cmd_vel_mux/input/teleop";
		//std::string pub_twist_topic_name = "/cmd_vel_mux/input/safety_controller";
		std::string pub_twist_topic_name = "/turtlebot_velocity_smoother/raw_cmd_vel";
		ros::param::get( "pub_twist_topic_name", pub_twist_topic_name );
		pub_twist = nh.advertise<geometry_msgs::Twist>( pub_twist_topic_name, 1 );

		std::string pub_speech_word_topic_name = "/speech_word";
    ros::param::get( "pub_speech_word_topic_name", pub_speech_word_topic_name );
		pub_speech_word = nh.advertise<std_msgs::String>( pub_speech_word_topic_name, 1 );

		std::string pub_target_person_marker_topic_name = "/target_person_marker";
    ros::param::get( "pub_target_person_marker_topic_name", pub_target_person_marker_topic_name );
		pub_target_person_marker = nh.advertise<visualization_msgs::Marker>( pub_target_person_marker_topic_name, 1 );


		sub_ctrl = nh.subscribe( "follow_ctrl", 1, &target_follower_class::ctrl_cb, this );

		std::string obstacle_point_topic_name = "/raw_obstacles";// "/raw_obstacles"
    ros::param::get( "obstacle_point_topic_name", obstacle_point_topic_name );
		sub_obstacle_point = nh.subscribe( obstacle_point_topic_name, 1, &target_follower_class::obstacle_point_cb, this );

		pub_state = nh.advertise<target_scan_follower::point_states>("/pub_point_states",1);
		//sub_goal = nh.subscribe("/sub_goal",1,&target_follower_class::following_obstacle,this);

		sub_map_info = nh.subscribe("/odom",1,&target_follower_class::possible_area,this);


		ROS_INFO_STREAM( "\ntarget_follower_class initialize ok. " );
		std_msgs::String speech_word_msg;
		speech_word_msg.data = "";
		pub_speech_word.publish( speech_word_msg );
		speech_word_msg.data = "start.";
		ROS_INFO_STREAM( "\n" << speech_word_msg.data );
		pub_speech_word.publish( speech_word_msg );


	}//target_follower_class

	~target_follower_class(){}


	void obstacle_point_cb(const obstacle_detector::Obstacles& input)
	{
		if( execute_flag == false ){return;}

		//ターゲットの初期化用
		double nearest_obstacle_distance = DBL_MAX;
		geometry_msgs::Point nearest_obstacle_point;

		//ターゲットの追跡用
		double min_obstacle_distance_from_last_target_point = DBL_MAX;
		geometry_msgs::Point nearest_obstacle_point_from_last_target_point;

		if( input.circles.size() == 0 )
		{
			lost_target_count++;
			return;
		}//if
		for( int i = 0 ; i < input.circles.size() ; i ++ )
		{
			geometry_msgs::Point temp_obstacle_point;//ワールド基準
			temp_obstacle_point.x = input.circles[i].center.x;
			temp_obstacle_point.y = input.circles[i].center.y;
			temp_obstacle_point.z = 0.0;

			if( init_target_point_flag == true )//ターゲットの初期化用
			{
				//いったんロボット基準の座標に戻して、ロボットと一番近い障害物を探す
				geometry_msgs::PointStamped world_obstacle_point;//変換前のワールド基準の座標を格納する
				geometry_msgs::PointStamped obstacle_point_from_robot;//変換後のロボットの座標を格納する
				try{
					world_obstacle_point.header.frame_id = input.header.frame_id;
					world_obstacle_point.header.stamp = ros::Time(0);//input.header.stamp;
					world_obstacle_point.point.x = temp_obstacle_point.x;
					world_obstacle_point.point.y = temp_obstacle_point.y;
					world_obstacle_point.point.z = temp_obstacle_point.z;
					listener.transformPoint( robot_position_frame_name, world_obstacle_point, obstacle_point_from_robot );
					//std::cout << "x: " << obstacle_point_from_robot.point.x << std::endl;
					//std::cout << "y: " << obstacle_point_from_robot.point.y << std::endl;
					//std::cout << "z: " << obstacle_point_from_robot.point.z << " but not use z." << std::endl;
					//std::cout << "############### " << std::endl;
				}//try
				catch(tf::TransformException& ex)
				{
			    ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
					continue;//return;
			  }//catch


				
				geometry_msgs::Point temp_front_point;
				temp_front_point.x = this->init_target_distance;//[m]正面
				temp_front_point.y = 0.0;
				temp_front_point.z = 0.0;
				double distance_from_front_robot_point = hypotf( obstacle_point_from_robot.point.x - temp_front_point.x , obstacle_point_from_robot.point.y - temp_front_point.y );
				if( distance_from_front_robot_point > distance_from_last_target_point_limit )
				{
					continue;//targetがdistance_from_last_target_point_limitよりも距離が遠ければ，以下の処理をスキップする
				}//if
				if( nearest_obstacle_distance > distance_from_front_robot_point )
				{
					nearest_obstacle_distance = distance_from_front_robot_point;
					nearest_obstacle_point = temp_obstacle_point;
				}//if
			}//if
			else//ターゲットの追跡用
			{
				double obstacle_distance_from_last_target_point = hypotf( temp_obstacle_point.x  - last_target_point.x , temp_obstacle_point.y - last_target_point.y );
				if( min_obstacle_distance_from_last_target_point > obstacle_distance_from_last_target_point )
				{
					min_obstacle_distance_from_last_target_point = obstacle_distance_from_last_target_point;
					nearest_obstacle_point_from_last_target_point = temp_obstacle_point;
					ROS_INFO_STREAM("target : \n" << temp_obstacle_point );
				}//if
			}//else

		}//for
		if( init_target_point_flag == true )//ターゲットの初期化用
		{
			if( nearest_obstacle_point.x == 0 && nearest_obstacle_point.y == 0 )//ワールド基準
			{//うまくターゲットを検出できなかった
				return;
			}//if
			init_target_point_flag = false;//初期化フラグの回収
			last_target_point = nearest_obstacle_point;

			std_msgs::String speech_word_msg;
			speech_word_msg.data = "I found a target person.";
			ROS_INFO_STREAM( "\n" << speech_word_msg.data );
			pub_speech_word.publish( speech_word_msg );

			return;
		}//if
		else//ターゲットの追跡用
		{
			ROS_INFO_STREAM( "min_obstacle_distance_from_last_target_point: " << min_obstacle_distance_from_last_target_point << "[m]" );

			target_follower_class::last_target_point_pub( last_target_point );
			int say_lost_num = 3;
			if( min_obstacle_distance_from_last_target_point > distance_from_last_target_point_limit )//前回のターゲットの位置から離れすぎているので、ターゲットをロストしたと考える
			{
				ROS_WARN_STREAM( "target_lost" );
				if( lost_target_count == say_lost_num )
				//if( (lost_target_count % say_lost_num) == 0 && lost_target_count != 0 )//一回言えばわかるはず
				{
					std_msgs::String speech_word_msg;
					speech_word_msg.data = "Come back in front of me.";
					ROS_INFO_STREAM( "\n" << speech_word_msg.data );
					pub_speech_word.publish( speech_word_msg );

					geometry_msgs::Twist msg_twist;
					msg_twist.linear.x =  0.0 ;
					pub_twist.publish( msg_twist );
				}//if
				lost_target_count++;
				return;
			}//if

			if( lost_target_count > say_lost_num )//ターゲットを見失っていたということ→「復帰しました」
			{
				std_msgs::String speech_word_msg;
				speech_word_msg.data = "OK, I found you.";
				ROS_INFO_STREAM( "\n" << speech_word_msg.data );
				pub_speech_word.publish( speech_word_msg );
			}//if
			lost_target_count = 0;
			last_target_point = nearest_obstacle_point_from_last_target_point;//最後のターゲットの位置の更新
			target_follower_class::target_point_pub( nearest_obstacle_point_from_last_target_point );
			//nearest_obstacle_point_from_last_target_point.x;

			last_target_point.x = 2.0 * nearest_obstacle_point_from_last_target_point.x - last_target_point.x;
			last_target_point.y = 2.0 * nearest_obstacle_point_from_last_target_point.y - last_target_point.y;

			//ワールド座標をロボット基準の座標に変換する
			geometry_msgs::PointStamped world_obstacle_point;//変換前のワールド基準の座標を格納する
			geometry_msgs::PointStamped obstacle_point_from_robot;//変換後のロボットの座標を格納する
			try{
				world_obstacle_point.header.frame_id = input.header.frame_id;
				world_obstacle_point.header.stamp = ros::Time(0);//input.header.stamp;
				world_obstacle_point.point.x = nearest_obstacle_point_from_last_target_point.x;
				world_obstacle_point.point.y = nearest_obstacle_point_from_last_target_point.y;
				world_obstacle_point.point.z = nearest_obstacle_point_from_last_target_point.z;
				listener.transformPoint( robot_position_frame_name, world_obstacle_point, obstacle_point_from_robot );
				//std::cout << "x: " << obstacle_point_from_robot.point.x << std::endl;
				//std::cout << "y: " << obstacle_point_from_robot.point.y << std::endl;
				//std::cout << "z: " << obstacle_point_from_robot.point.z << " but not use z." << std::endl;
				//std::cout << "############### " << std::endl;
			}//try
			catch(tf::TransformException& ex)
			{
				ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
				return;
			}//catch

			//map基準から見たbase_footprintの座標系
			tf::StampedTransform transform;
			try{
		  		this->listener.lookupTransform("/base_laser_link", "/odom", ros::Time(0), transform);
			}//try
			catch (tf::TransformException ex){
		  		ROS_ERROR("%s",ex.what());
		  		ros::Duration(1.0).sleep();
			}//try

			geometry_msgs::Point robot_point;//odom基準のロボットの位置座標
			robot_point.x = transform.getOrigin().x();
			robot_point.y = transform.getOrigin().y();
			robot_point.z = 0.0;
			target_scan_follower::point_states states;
			states.robot_point = robot_point;
			states.target_point.point = obstacle_point_from_robot.point;

			target_follower_class::pub_state.publish( states );

			/*geometry_msgs::PointStamped target_point;//変換後のtargetの座標を格納する
			try{
				obstacle_point_from_robot.header.frame_id = input.header.frame_id;
				obstacle_point_from_robot.header.stamp = input.header.stamp;
				obstacle_point_from_robot.x = nearest_obstacle_point_from_last_target_point.x;
				obstacle_point_from_robot.y = nearest_obstacle_point_from_last_target_point.y;
				obstacle_point_from_robot.z = nearest_obstacle_point_from_last_target_point.z;
				listener.transformPoint( "target",obstacle_point_from_robot , obstacle_point_from_robot );
				//std::cout << "x: " << obstacle_point_from_robot.point.x << std::endl;
				//std::cout << "y: " << obstacle_point_from_robot.point.y << std::endl;
				//std::cout << "z: " << obstacle_point_from_robot.point.z << " but not use z." << std::endl;
				//std::cout << "############### " << std::endl;
			}//try
			catch(tf::TransformException& ex)
			{
				ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
				return;
			}//catch*/

			
			double target_rad_2d = atan2f( obstacle_point_from_robot.point.y , obstacle_point_from_robot.point.x );
			double target_distance_2d = hypotf( obstacle_point_from_robot.point.x , obstacle_point_from_robot.point.y );
			double target_deg_2d = target_rad_2d * rad2deg;
			double target_x = obstacle_point_from_robot.point.x;
			double target_y = obstacle_point_from_robot.point.y;
			ROS_INFO_STREAM( "\ntarget_deg: "      << target_deg_2d << "[deg]" );
			ROS_INFO_STREAM( "\ntarget_distance: " << target_distance_2d << "[m]" );

			target_follower_class::following_obstacle( target_rad_2d, target_distance_2d , target_x , target_y );//障害物回避を考慮した移動
			return;
		}//else

		//target_follower_class::stop_pub();



		return;
	}//obstacle_point_cb


	void ctrl_cb(const std_msgs::Bool& input)
	{
		execute_flag = input.data;
		if(execute_flag == true){
			ROS_INFO_STREAM("target_follower_class -> Start");

			std_msgs::String speech_word_msg;
			speech_word_msg.data = "I follow you.";
			ROS_INFO_STREAM( "\n" << speech_word_msg.data );
			pub_speech_word.publish( speech_word_msg );
			init_target_point_flag = true;
		}//if
		else{
			ROS_INFO_STREAM("target_follower_class -> Stopped");
			target_follower_class::stop_pub();

			std_msgs::String speech_word_msg;
			speech_word_msg.data = "I will end following.";
			ROS_INFO_STREAM( "\n" << speech_word_msg.data );
			pub_speech_word.publish( speech_word_msg );
			init_target_point_flag = false;
		}//else
	}//contrl_CB







	void stop_pub()
	{
		geometry_msgs::Twist msg_twist;
		msg_twist.angular.z = 0.0;
		msg_twist.linear.x =  0.0 ;
		pub_twist.publish( msg_twist );//停止命令
		return;
	}//stop_pub




	double smoothing_speed( double speed )
	{
		old_speed_vec.push_back( speed );
		while(1)//old_speed_vecが大きくなり過ぎないように古いものを消す
		{
			if( old_speed_vec.size() > old_speed_vec_size )
			{
				old_speed_vec.erase( old_speed_vec.begin() );//一番古いものを削除
			}//if
			else
			{
				break;//while
			}//else
		}//while
		double ave_speed = 0.0;
		for(int i = 0 ; i < old_speed_vec.size() ; i++ )
		{
			ave_speed += old_speed_vec[i];
		}
		ave_speed = ave_speed / ( (double)old_speed_vec.size() );

		//ave_speed = 0.0;
		return ave_speed;
	}//smoothing_speed





	bool following_obstacle(double target_rad_2d , double target_distance_2d , double target_x , double target_y)//障害物回避を考慮した移動
	{
		//geometry_msgs::Point target;
		//target = *goal;
		//double target_rad_2d = atan2f( target.y , target.x );
		//double target_distance_2d = hypotf( target.x , target.y );
		
		geometry_msgs::Twist msg_twist;//ロボットの車輪の移動と回転速度
		if( target_distance_2d > max_move_distance )
		{
			msg_twist.linear.x =  0.0 ;
		}//if
		
		double target_deg_2d = rad2deg * target_rad_2d;
		double necessary_move_distance = target_distance_2d - keep_distance;
		ROS_INFO_STREAM( "\ntarget_distance_2d =  " << target_distance_2d );
		ROS_INFO_STREAM( "\nnecessary_move_distance =  " << necessary_move_distance );

		if( fabs( necessary_move_distance ) < stop_distance )
		{
			msg_twist.linear.x =  0.0 ;
			ROS_INFO_STREAM( "\nmsg_twist.linear.x =  stop " << msg_twist.linear.x);
		}//if
		else//3
		{
			if( target_distance_2d < constant_velocity_distance )//4 距離に比例した速度(ロボットと人との距離が2.0[m]より小さい場合)
			{
				if( fabs( target_deg_2d ) < 90.0 )//5
				{
					msg_twist.linear.x = max_move_speed * ( necessary_move_distance / (double)(constant_velocity_distance - keep_distance ) ) * cos( target_rad_2d );
					//ROS_INFO_STREAM( "\nmsg_twist.linear.x =  constant " << msg_twist.linear.x);
				}//5
				else//6
				{
					msg_twist.linear.x = max_move_speed * ( necessary_move_distance / (double)( keep_distance ) ) * cos( target_rad_2d );
				}//6
				ROS_INFO_STREAM( "\nmsg_twist.linear.x =  " << msg_twist.linear.x);
			}//4
			else//7 距離に比例した速度(ロボットと人との距離が2.0[m]より大きい場合)
			{
				msg_twist.linear.x = max_move_speed;
				ROS_INFO_STREAM( "\nmsg_twist.linear.x =  constant " << msg_twist.linear.x);
			}//7

			if(necessary_move_distance <= 0)//8 もし人がロボットに近づいた場合，後退する
			{
				msg_twist.linear.x = -1 * msg_twist.linear.x;
				ROS_INFO_STREAM( "\nnecessary_move_distance <= 0 " << msg_twist.linear.x);
			}//8
			
			double candidate_position_x = msg_twist.linear.x * 1;//移動先の候補

			/*if(candidate_position_x > Possible_range_x)//9 ロボットが移動する候補の位置が移動可能な領域より外側の場合
			{
				msg_twist.linear.x = 0.0;//
				if(msg_twist.linear.x < 0.0)//後退する場合
					
				else//前進する場合
					
			}//9*/

			msg_twist.linear.x =  msg_twist.linear.x ; //smoothing_speed関数へ移動

			//move;
		}//3
			ROS_INFO( "\nmsg_twist.linear.x = %f  ", msg_twist.linear.x);//速度の確認



		if( fabs( target_deg_2d ) > 0 && fabs( target_deg_2d ) <= constant_velocity_deg )//車輪の回転調整
		{
			msg_twist.angular.z = ( deg2rad * max_trun_speed ) * ( target_deg_2d / (double)constant_velocity_deg );
			ROS_INFO_STREAM( "\nangular.z : abs( target_deg_2d ) > 0 && abs( target_deg_2d ) <= constant_velocity_deg " );
		}//if

		else if( target_deg_2d > constant_velocity_deg )
		{
			msg_twist.angular.z = deg2rad * max_trun_speed ;
			ROS_INFO_STREAM( "\nangular.z : target_deg_2d > constant_velocity_deg " );
		}//else if
		else if( target_deg_2d < -constant_velocity_deg )
		{
			msg_twist.angular.z = -deg2rad * max_trun_speed ;
			ROS_INFO_STREAM( "\nangular.z : target_deg_2d < -constant_velocity_deg " );
		}//else if

		ROS_INFO_STREAM( "\nangular.z = " << msg_twist.angular.z );
			//move;
		

		ROS_INFO_STREAM( "\nmsg_twist: " << msg_twist );
		pub_twist.publish( msg_twist );


		return true;
	}//following_WITHOUT_obstacle_avoidance



	void possible_area(const nav_msgs::OccupancyGrid& msg)
	{
		double distance_num = 0.5;
		odom_info = msg;
		Possible_range_x = fabs(msg.info.origin.position.x - distance_num);
		Possible_range_y = fabs(msg.info.origin.position.y - distance_num);
	}

	void last_target_point_pub( geometry_msgs::Point world_last_target_point  )
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = odom_frame_name;
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::CYLINDER;

		marker.ns = "last_target_point";
		marker.header.frame_id = odom_frame_name;
		marker.id = 9;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = 0.03;//[m]
		marker.scale.y = 0.03;//[m]
		marker.scale.z = 1.0;//[m]
		marker.pose.position.x = world_last_target_point.x;
		marker.pose.position.y = world_last_target_point.y;
		marker.pose.position.z = marker.scale.z * 0.5;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		return;
	}//last_target_point_pub

	void target_point_pub( geometry_msgs::Point world_target_point  )
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = urg_frame_name;
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::SPHERE;

		marker.ns = "target_point_area";
		marker.header.frame_id = odom_frame_name;
		marker.id = 10;
		marker.color.a = 0.1;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = distance_from_last_target_point_limit;//[m]
		marker.scale.y = distance_from_last_target_point_limit;//[m]
		marker.scale.z = 0.01;//[m]
		marker.pose.position.x = world_target_point.x;
		marker.pose.position.y = world_target_point.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		return;
	}//target_point_pub




};//target_follower_class



int main(int argc, char** argv)
{
	ros::init(argc, argv, "target_follower_node");
	target_follower_class tf_class;
	ros::spin();
	return 0;
}//main

