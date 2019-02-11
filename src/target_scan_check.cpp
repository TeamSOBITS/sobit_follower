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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/shared_ptr.hpp>
#include <typeinfo>

#include <sobit_follower/grouped_points_array.h>
#include <sobit_follower/grouped_points.h>

#include <sobit_follower/target_predict.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;


//define
#define nsec2sec 1e-9
#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180


struct point_xy {
		double x;
		double y;
};

class target_scan_class
{
	ros::NodeHandle nh;
	//SUB
	ros::Subscriber sub_ctrl;
	ros::Subscriber sub_obstacle_point;
	ros::Subscriber sub_goal;
	ros::Subscriber sub_map_info;
	ros::Subscriber sub_goal_flag;
	ros::Subscriber sub_points_states;
	//PUB
	ros::Publisher pub_twist;
	ros::Publisher pub_speech_word;
	ros::Publisher pub_target_person_marker;
	ros::Publisher pub_state;
	ros::Publisher pub_obstacles_states;
	ros::Publisher axis_state;
	ros::Publisher pub_judge;

	//CLIENT
	ros::ServiceClient client;

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

	bool target_lost_flag;
	bool init_target_point_flag;
	bool pub_propriety_flag;
	int lost_target_count;
	geometry_msgs::Point last_target_point;
	double distance_from_last_target_point_limit;
	double last_target_distance_2d;

	double init_target_distance ;//[m]

	//std::vector<double> old_speed_vec;
	//bool old_speed_vec_size;

public:
	target_scan_class(){
		//old_speed_vec.push_back( 0.0 );
		lost_target_count = 0;
		init_target_point_flag = true;
		execute_flag = true;
		this->target_lost_flag = false;
		this->pub_propriety_flag = true;


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

		axis_state = nh.advertise<visualization_msgs::Marker>("axis_marker", 1 );


		sub_ctrl = nh.subscribe( "follow_ctrl", 1, &target_scan_class::ctrl_cb, this );

		std::string obstacle_point_topic_name = "/raw_obstacles";// "/raw_obstacles"
    ros::param::get( "obstacle_point_topic_name", obstacle_point_topic_name );
		//sub_obstacle_point = nh.subscribe( "circle_states", 1, &target_scan_class::obstacle_point_cb, this );

		this->pub_judge = nh.advertise<std_msgs::Bool>("/target_judge",1);
		this->sub_goal_flag = nh.subscribe("/pub_goal_flag",1,&target_scan_class::scan_judge,this);
		//this->sub_goal = nh.subscribe("/sub_goal",1,&target_scan_class::following_obstacle,this);

		//sub_map_info = nh.subscribe("/odom",1,&target_scan_class::possible_area,this);
		//target情報をパブリッシュ
		//this->pub_state = nh.advertise<geometry_msgs::Point>("/target_states",1);
		//Points_groupingから点群情報をサブスクライブ
		this->sub_points_states = nh.subscribe("/points_states",1,&target_scan_class::points_cb,this);
		//障害物情報をパブリッシュ
		this->pub_obstacles_states = nh.advertise<sobit_follower::grouped_points_array>("/target_and_obstacle_states", 1 );

		this->client = nh.serviceClient<sobit_follower::target_predict>("kalman_check");

		ROS_INFO_STREAM( "\ntarget_scan_class initialize ok. " );
		std_msgs::String speech_word_msg;
		speech_word_msg.data = "";
		pub_speech_word.publish( speech_word_msg );
		speech_word_msg.data = "start.";
		ROS_INFO_STREAM( "\n" << speech_word_msg.data );
		pub_speech_word.publish( speech_word_msg );

	}//target_scan_class

	~target_scan_class(){}

	void points_cb(const sobit_follower::grouped_points_arrayPtr input)
	{
		if( execute_flag == false ){return;}

		//ターゲットの初期化用
		double nearest_obstacle_distance = DBL_MAX;
		geometry_msgs::Point nearest_obstacle_point;

		//ターゲットの追跡用
		double min_obstacle_distance_from_last_target_point = DBL_MAX;
		geometry_msgs::Point nearest_obstacle_point_from_last_target_point;
		int num = 0;
		//サービス
		sobit_follower::target_predict srv;
		if( input->grouped_points_array.size() == 0 )
		{
			lost_target_count++;
			return;
		}//if
		/*for( int i = 0 ; i < input->grouped_points_array.size() ; i ++ )
		{
			geometry_msgs::Point obstacle_point_from_robot;
			obstacle_point_from_robot.x = input->grouped_points_array[i].center_x;
			obstacle_point_from_robot.y = input->grouped_points_array[i].center_y;
			obstacle_point_from_robot.z = 0.0;
			if((input->grouped_points_array[i].center_x > 0.0 && input->grouped_points_array[i].center_x <= 0.7) &&(input->grouped_points_array[i].center_y >= -0.2 && input->grouped_points_array[i].center_y <= 0.2))
			std::cout << "obstacle_point_from_robot :: " << obstacle_point_from_robot << std::endl;
		}//*/
		for( int i = 0 ; i < input->grouped_points_array.size() ; i ++ )
		{
			geometry_msgs::Point temp_obstacle_point;//ワールド基準
			temp_obstacle_point.x = input->grouped_points_array[i].center_x;
			temp_obstacle_point.y = input->grouped_points_array[i].center_y;
			temp_obstacle_point.z = 0.0;

			geometry_msgs::Point obstacle_point_from_robot;//base_laser_link基準
			obstacle_point_from_robot.x = input->grouped_points_array[i].center_x;
			obstacle_point_from_robot.y = input->grouped_points_array[i].center_y;
			obstacle_point_from_robot.z = 0.0;
			geometry_msgs::PointStamped laser_base_obstacle_point;//変換前のlaser基準の座標を格納する
			geometry_msgs::PointStamped footprint_base_obstacle_point;//変換後のロボットの座標を格納する
			try{
				laser_base_obstacle_point.header.frame_id = "/base_laser_link";
				laser_base_obstacle_point.header.stamp = ros::Time(0);
				laser_base_obstacle_point.point = obstacle_point_from_robot;
				listener.transformPoint(robot_position_frame_name, laser_base_obstacle_point, footprint_base_obstacle_point );
			}//try
			catch(tf::TransformException& ex)
			{
		    ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
				continue;//return;
		  }//catch*/
			if( init_target_point_flag == true )//ターゲットの初期化用
			{
				geometry_msgs::Point temp_front_point;

				temp_front_point.x = this->init_target_distance;
				temp_front_point.y = 0.0;

				double distance_from_front_robot_point = hypotf( footprint_base_obstacle_point.point.x - temp_front_point.x , footprint_base_obstacle_point.point.y - temp_front_point.y );
				if( distance_from_front_robot_point > distance_from_last_target_point_limit )
				{
					continue;//targetがdistance_from_last_target_point_limitよりも距離が遠ければ，以下の処理をスキップする
				}//if
				if( nearest_obstacle_distance > distance_from_front_robot_point )
				{
					nearest_obstacle_distance = distance_from_front_robot_point;
					nearest_obstacle_point = footprint_base_obstacle_point.point;
					num = i;
				}//if
			}//if
			else//ターゲットの追跡用
			{
				double obstacle_distance_from_last_target_point = hypotf( footprint_base_obstacle_point.point.x  - last_target_point.x , footprint_base_obstacle_point.point.y - last_target_point.y );
				/*if( distance_from_front_robot_point > distance_from_last_target_point_limit )
				{
					continue;//targetがdistance_from_last_target_point_limitよりも距離が遠ければ，以下の処理をスキップする
				}//if*/
				if( min_obstacle_distance_from_last_target_point > obstacle_distance_from_last_target_point )
				{
					min_obstacle_distance_from_last_target_point = obstacle_distance_from_last_target_point;
					nearest_obstacle_point_from_last_target_point = footprint_base_obstacle_point.point;
					//std::cout << "min_obstacle_distance_from_last_target_point :: " << min_obstacle_distance_from_last_target_point << std::endl;
					//ROS_INFO_STREAM("target : \n" << footprint_base_obstacle_point.point );	
					num = i;
				}//if
			}//else
		}//for
		//ROS_INFO("nearest_obstacle_point_from_last_target_point::%f",nearest_obstacle_point_from_last_target_point.x);
		//std::cout << "last_target_point :: " << last_target_point << std::endl;
		//std::cout << "nearest_obstacle_point_from_last_target_point :: " << nearest_obstacle_point_from_last_target_point << std::endl;
		//ROS_INFO("nearest_obstacle_point_from_last_target_point::%f",nearest_obstacle_point_from_last_target_point);
		input->grouped_points_array[num].center_radius = 0.0;//target周辺のコストをなくす
		input->grouped_points_array[num].particle_radius = 0.0;
		//std::cout << "init_target_point_flag :: " << init_target_point_flag << std::endl;
		if( init_target_point_flag == true )//ターゲットの初期化用
		{
			if( nearest_obstacle_point.x == 0 && nearest_obstacle_point.y == 0 )//ワールド基準
			{//うまくターゲットを検出できなかった
				return;
			}//if
			init_target_point_flag = false;//初期化フラグの回収
			last_target_point = nearest_obstacle_point;

			std_msgs::String speech_word_msg;
			speech_word_msg.data = "OK I found a target person.";
			//ROS_INFO_STREAM( "\n" << speech_word_msg.data );
			pub_speech_word.publish( speech_word_msg );
			return;
		}//if
		else//ターゲットの追跡用
		{
			//std::cout << "last_target_point :: " << last_target_point << std::endl;
			target_scan_class::last_target_point_pub( last_target_point );
			int say_lost_num = 3;
			std_msgs::Bool lost_flag;
			if( min_obstacle_distance_from_last_target_point > distance_from_last_target_point_limit )//前回のターゲットの位置から離れすぎているので、ターゲットをロストしたと考える
			{
				//ROS_INFO_STREAM( "lost_target_count : " << lost_target_count << "[num]" );
				if(lost_target_count > 8)
				{
					this->target_lost_flag = true;
					ROS_WARN_STREAM( "target_lost" );
				}//if
				lost_flag.data = this->target_lost_flag;
				this->pub_judge.publish(lost_flag);
				if( lost_target_count == say_lost_num )
				//if( (lost_target_count % say_lost_num) == 0 && lost_target_count != 0 )//一回言えばわかるはず
				{
					std_msgs::String speech_word_msg;
					speech_word_msg.data = "Come back in front of me.";
					//ROS_INFO_STREAM( "\n" << speech_word_msg.data );
					pub_speech_word.publish( speech_word_msg );

					geometry_msgs::Twist msg_twist;
					msg_twist.linear.x =  0.0 ;
					pub_twist.publish( msg_twist );
				}//if
				lost_target_count++;

				//追従対象者を見失った場合
				srv.request.target_lost_flag = true;
				//追従対象者の位置を更新
				if(this->client.call(srv))//サービス呼び出しが上手くいった場合，カルマンで予測したtargetの位置を更新
				{
					ROS_INFO("SERVICE_RESPONSE_OK");
					input->target_point = srv.response.pre_target_point;
				}
				else//サービス呼び出しが上手くいかなかった場合，過去の最も新しいtargetの位置を更新
				{
					ROS_INFO("SERVICE_RESPONSE_NO");
					input->target_point = last_target_point;
				}
				this->pub_obstacles_states.publish(input);
				return;
			}//if
			if( this->target_lost_flag == true && this->pub_propriety_flag == true )//ターゲットを見失っていたということ→「復帰しました」 lost_target_count > say_lost_num
			{
				init_target_point_flag = true;//初期化し，前に人がいるかを判断
				this->target_lost_flag = false;
				lost_flag.data = this->target_lost_flag;
				std_msgs::String speech_word_msg;
				ROS_INFO_STREAM("target_searching");
				speech_word_msg.data = "OK, I found you.";
				//ROS_INFO_STREAM( "\n" << speech_word_msg.data );
				this->pub_judge.publish(lost_flag);
				pub_speech_word.publish( speech_word_msg );
			}//if
			lost_target_count = 0;
			last_target_point = nearest_obstacle_point_from_last_target_point;//最後のターゲットの位置の更新
			target_scan_class::target_point_pub( nearest_obstacle_point_from_last_target_point );
			//nearest_obstacle_point_from_last_target_point.x;

			last_target_point.x = 2.0 * nearest_obstacle_point_from_last_target_point.x - last_target_point.x;
			last_target_point.y = 2.0 * nearest_obstacle_point_from_last_target_point.y - last_target_point.y;
			//this->pub_state.publish( nearest_obstacle_point_from_last_target_point );//path_plan_testに人位置をパブリッシュ
			//std::cout << "this->pub_propriety_flag:: " << this->pub_propriety_flag << std::endl;
			//std::cout << "this->target_lost_flag:: " << this->target_lost_flag << std::endl;

			//追従対象者を見失っていないor再度見つけた場合
			srv.request.target_lost_flag = false;
			srv.request.target_point = nearest_obstacle_point_from_last_target_point;

			//追従対象者の位置を更新
			if(this->client.call(srv))
			{
				ROS_INFO("SERVICE_RESPONSE_OK");
			}
			else
			{
				ROS_INFO("SERVICE_RESPONSE_NO");
			}
			//targetの位置を更新
			input->target_point = nearest_obstacle_point_from_last_target_point;
			//path生成ノードにパブリッシュ
			this->pub_obstacles_states.publish(input);

			/*double target_rad_2d = atan2f( footprint_base_obstacle_point.point.y , footprint_base_obstacle_point.point.x );
			double target_distance_2d = hypotf( footprint_base_obstacle_point.point.x , footprint_base_obstacle_point.point.y );
			double target_deg_2d = target_rad_2d * rad2deg;
			double target_x = footprint_base_obstacle_point.point.x;
			double target_y = footprint_base_obstacle_point.point.y;
			ROS_INFO_STREAM( "\ntarget_deg: "      << target_deg_2d << "[deg]" );
			ROS_INFO_STREAM( "\ntarget_distance: " << target_distance_2d << "[m]" );*/

			//target_scan_class::following_obstacle( target_rad_2d, target_distance_2d , target_x , target_y );//障害物回避を考慮した移動
			return;
		}//else

		//target_scan_class::stop_pub();
		return;
	}//obstacle_point_cb

	void scan_judge(const std_msgs::Bool& msg)
	{
		if(this->target_lost_flag == true)
		{
			this->pub_propriety_flag = msg.data;
		}
		else
		{
			//printf("targetを見失っていません\n");
		}
	}//scan_judge


	void ctrl_cb(const std_msgs::Bool& input)
	{
		execute_flag = input.data;
		if(execute_flag == true){
			//ROS_INFO_STREAM("target_scan_class -> Start");

			std_msgs::String speech_word_msg;
			speech_word_msg.data = "I follow you.";
			ROS_INFO_STREAM( "\n" << speech_word_msg.data );
			pub_speech_word.publish( speech_word_msg );
			init_target_point_flag = true;
		}//if
		else{
			//ROS_INFO_STREAM("target_scan_class -> Stopped");
			target_scan_class::stop_pub();

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




	/*double smoothing_speed( double speed )
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
	}//smoothing_speed*/





	/*bool following_obstacle(double target_rad_2d , double target_distance_2d , double target_x , double target_y)//障害物回避を考慮した移動
	{
		geometry_msgs::Point target;
		target = *goal;
		double target_rad_2d = atan2f( target.y , target.x );
		double target_distance_2d = hypotf( target.x , target.y );
		
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

			if(candidate_position_x > Possible_range_x)//9 ロボットが移動する候補の位置が移動可能な領域より外側の場合
			{
				msg_twist.linear.x = 0.0;//
				if(msg_twist.linear.x < 0.0)//後退する場合
					
				else//前進する場合
					
			}//9

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
*/


	/*void possible_area(const nav_msgs::OccupancyGrid& msg)
	{
		double distance_num = 0.5;
		odom_info = msg;
		Possible_range_x = fabs(msg.info.origin.position.x - distance_num);
		Possible_range_y = fabs(msg.info.origin.position.y - distance_num);
	}*/

	void last_target_point_pub( geometry_msgs::Point world_last_target_point  )
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = robot_position_frame_name;
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::CYLINDER;

		marker.ns = "last_target_point";
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

		double local_range = 1.2;
		double possible_range = 0.4;

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

		/*marker.ns = "axis_point";
		marker.header.frame_id = "base_footprint";
		marker.id = 10;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.scale.x = 0.01;//[m]
		marker.scale.y = 0.01;//[m]
		marker.scale.z = 10.0;//[m]
		marker.pose.position.x = -0.3;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "local_area";
		marker.header.frame_id = "base_footprint";
		marker.id = 11;
		marker.color.a = 0.30;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.scale.x = local_range;//[m]
		marker.scale.y = local_range;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "possible_area_side";
		marker.header.frame_id = "base_footprint";
		marker.id = 12;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.scale.x = possible_range * 2;//[m]
		marker.scale.y = possible_range * 2;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "possible_area_side_upper";
		marker.header.frame_id = "base_footprint";
		marker.id = 13;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = (local_range * 0.5 - possible_range * 0.5) * 0.5;//[m]
		marker.scale.y = possible_range * 2;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = (local_range * 0.5 - possible_range) * 0.5 + possible_range;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "possible_area_side_bottom";
		marker.header.frame_id = "base_footprint";
		marker.id = 14;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = (local_range * 0.5 - possible_range * 0.5) * 0.5;//[m]
		marker.scale.y = possible_range * 2;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = -((local_range * 0.5 - possible_range) * 0.5 + possible_range);
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "possible_area_side_left";
		marker.header.frame_id = "base_footprint";
		marker.id = 15;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = possible_range * 2;//[m]
		marker.scale.y = (local_range * 0.5 - possible_range * 0.5) * 0.5;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = 0.0;
		marker.pose.position.y = (local_range * 0.5 - possible_range) * 0.5 + possible_range;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );

		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "possible_area_side_right";
		marker.header.frame_id = "base_footprint";
		marker.id = 17;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.scale.x = possible_range * 2;//[m]
		marker.scale.y = (local_range * 0.5 - possible_range * 0.5) * 0.5;//[m]
		marker.scale.z = 0.0001;//[m]
		marker.pose.position.x = 0.0;
		marker.pose.position.y = -((local_range * 0.5 - possible_range) * 0.5 + possible_range);
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		pub_target_person_marker.publish( marker );*/


		return;
	}//target_point_pub





};//target_scan_class



int main(int argc, char** argv)
{
	ros::init(argc, argv, "target_scan_node");
	target_scan_class tf_class;
	ros::spin();
	return 0;
}//main

