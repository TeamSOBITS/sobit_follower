#include <stdio.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <time.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <iomanip>
#include <limits>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <float.h>
#include <list>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/SegmentObstacle.h>
#include <obstacle_detector/CircleObstacle.h>

#include <sobit_follower/grouped_points_array.h>
#include <sobit_follower/grouped_points.h>

typedef struct {         /* 構造体の型枠を定義する */
	double regression_radius;
    double radius;
    double center_x;
	double center_y;
} circle_set;

typedef struct {         /* 構造体の型枠を定義する */
    double x;
	double y;
} point_set;

typedef struct {         /* 構造体の型枠を定義する */
	std::list<point_set> buf;
    double radius;
	double center_radius;
    double center_x;
	double center_y;
} tp_set;

class Points_grouping_class
{
	ros::NodeHandle nh;
	//SUB
	ros::Subscriber sub_laser_scan;
	//PUB
	ros::Publisher pub_circle_states;
	ros::Publisher pub_circle_states_correction;
	ros::Publisher pub_circle_states_multiple;
	ros::Publisher pub_circle_states_part;
	ros::Publisher pub_cost_marker;
	ros::Publisher pub_points_states;
	ros::Publisher pub_test_marker;
	//list
	std::list<point_set> points_list;
	std::list<circle_set> group_list;
	std::list<tp_set> sample_group;
	std::list<point_set> group_points_list;
	std::string frame_id;
	//param
	double laser_point_x = 0.0;
	double laser_point_y = 0.0;
	double max_points_range = 0.1;	//隣同士の点間範囲(距離)
	double max_circle_radius = 0.3;	//中心点からの最大半径
	double min_circle_radius = 0.04;//中心点からの最小半径
	double correction_of_radius = 0.05;//各円の半径を微調整
	int pp;
	double base_radius = 0.4;
	//struct
	circle_set correction_values;
	tp_set all_set;
	
public:
	Points_grouping_class(){
	//Laser_scanをサブスクライブ
	this->sub_laser_scan = nh.subscribe("/scan",1,&Points_grouping_class::laser_points,this);
	//circle_statesをパブリッシュ
	this->pub_circle_states = nh.advertise<obstacle_detector::Obstacles>("/circle_states", 1 );
	//points_statesをパブリッシュ
	this->pub_points_states = nh.advertise<sobit_follower::grouped_points_array>("/points_states", 1 );
	//this->pub_circle_states_correction = nh.advertise<obstacle_detector::Obstacles>("circle_states_correction", 1 );
	//this->pub_circle_states_multiple = nh.advertise<obstacle_detector::Obstacles>("multiple_circle_states", 1 );
	this->pub_circle_states_part = nh.advertise<obstacle_detector::Obstacles>("part_circle_states", 1 );
	//this->cost_grant_point = nh.advertise<geometry_msgs::Point>("cost", 1 );
	this->pub_cost_marker = nh.advertise<visualization_msgs::MarkerArray>( "/point_marker", 1 );
	this->pub_test_marker = nh.advertise<visualization_msgs::MarkerArray>( "/test_marker", 1 );
	//this->sub_laser_scan = nh.subscribe("circle_states",1,&Points_grouping_class::marker,this);
	//this->group_lists.grouped_points_array[group_size].particle_x.resize(group_points_list.size());
	}//public

~Points_grouping_class(){}

void laser_points(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	{
		ros::Time now = ros::Time::now();
		if(now.sec - scan_msg->header.stamp.sec > 1)	{return;}

		this->frame_id = scan_msg->header.frame_id;//base_laser_link
		double phi = scan_msg->angle_min;	//angleの開始角度

		for(const float ranges : scan_msg->ranges)//1
		{
			if (ranges >= scan_msg->range_min && ranges <= scan_msg->range_max)
			{	
				point_set temp_point_set;
				temp_point_set.x = ranges * cos(phi);
				temp_point_set.y = ranges * sin(phi);
				points_list.push_back(temp_point_set);
			}//if
			phi += scan_msg->angle_increment;
		}//for
		decision_of_grouping();
	}//laser_points

void decision_of_grouping()
	{
		double points_ranges = 0.0;
		auto past_point = points_list.begin();
		int point_counter = 0;
		//this->pp = 0;
		//折角なのでobstacleの型に合わせる
		obstacle_detector::ObstaclesPtr all_circle_states(new obstacle_detector::Obstacles);	//全ての点群のステート
		obstacle_detector::CircleObstacle circle;
		all_circle_states->header.frame_id = this->frame_id;

		obstacle_detector::ObstaclesPtr part_circle_states(new obstacle_detector::Obstacles);	//試験用
		obstacle_detector::CircleObstacle part_circle;
		part_circle_states->header.frame_id = this->frame_id;
		int i = 0;

		for(auto point = this->points_list.begin(); point != this->points_list.end(); point++) //for points_list loop
		{
			auto first_point = this->points_list.begin();
			points_ranges = hypotf((past_point->x - point->x), (past_point->y - point->y));//2点のpoint間の範囲を計算
			if(points_ranges > this->max_points_range || point_counter == points_list.size()-1 )//グループの更新（新しいグループに入った） if_1
			{
				first_point = point;
				//this->end_points_list.push_back(past_point);	//listの末尾に前の点を追加
				//this->end_points_list.push_front(first_point);	//listの先端に新しい点を追加
				double center_x_total = 0.0;
				double center_y_total = 0.0;

				auto begin_set = this->group_points_list.begin();
				auto end_set = this->group_points_list.end();

				double begin_point_x = begin_set->x;
				double begin_point_y = begin_set->y;
				double end_point_x = end_set->x;
				double end_point_y = end_set->y;
				Points_grouping_class::linear_regression();//回帰
				this->sample_group.push_back(this->all_set);
				i++;
				//保存用としてgroup_listに挿入
				circle_set temp_circle_set;
				temp_circle_set.radius  = this->correction_values.radius;
				temp_circle_set.center_x = this->correction_values.center_x;
				temp_circle_set.center_y = this->correction_values.center_y;
				group_list.push_back(temp_circle_set);

					//publishする時の型に合わせる
				if( this->min_circle_radius < this->correction_values.radius && this->correction_values.radius < this->max_circle_radius) 					{	//if_3
        			circle.center.x = this->correction_values.center_x;
        			circle.center.y = this->correction_values.center_y;
					circle.radius = this->correction_values.radius + 0.1;
					all_circle_states->circles.push_back(circle);
				}//if_3
				this->all_set.buf.clear();
				this->group_points_list.clear();//空にする
			}//if_1
			past_point = point;	//過去の点を更新
			this->group_points_list.push_back( *point );	//group_points_listに追加
			point_counter++;	//余りよろしくない
		}//for points_list loop
		this->pub_circle_states.publish(all_circle_states);//all_circle_statesをパブリッシュ
		tp_set group_set;
		int group_number = 0;
		sobit_follower::grouped_points_array group_lists;
		group_lists.grouped_points_array.resize(this->sample_group.size());	

		for(auto sg_num = this->sample_group.begin(); sg_num != this->sample_group.end(); sg_num++)//for_sg
		{
			auto sg_point_num = *sg_num;//全てのステートの初期化
			group_set = sg_point_num;//グループ番号を初期化
			//各グループ毎に点の配列を初期化
			group_lists.grouped_points_array[group_number].particle_x.resize(group_set.buf.size());
			group_lists.grouped_points_array[group_number].particle_y.resize(group_set.buf.size());
			group_lists.grouped_points_array[group_number].particle_radius = group_set.radius;
			group_lists.grouped_points_array[group_number].center_x = group_set.center_x;
			group_lists.grouped_points_array[group_number].center_y = group_set.center_y;
			group_lists.grouped_points_array[group_number].center_radius = group_set.center_radius;
			int point_number = 0;
			for(auto sg_t = group_set.buf.begin(); sg_t != group_set.buf.end(); sg_t++)//for_sg_t
			{
				group_lists.grouped_points_array[group_number].particle_x[point_number] = sg_t->x;
				group_lists.grouped_points_array[group_number].particle_y[point_number] = sg_t->y;
				point_number++;
			}//for_sg_t
			group_number++;
		}//for_sg
		test_marker();

		this->pub_points_states.publish(group_lists);
		this->sample_group.clear();

		obstacle_detector::ObstaclesPtr circle_states_correction(new obstacle_detector::Obstacles);//比較用 
		obstacle_detector::CircleObstacle circle_obs;
		circle_states_correction->header.frame_id = this->frame_id;

		obstacle_detector::ObstaclesPtr multiple_circle_list(new obstacle_detector::Obstacles);//複数のサークルが近い時の各サークルの情報を保存
		obstacle_detector::CircleObstacle circle_states_save;
		multiple_circle_list->header.frame_id = this->frame_id;

		double next_radius = 0.0;
		double next_center_x = 0.0;
		double next_center_y  = 0.0;
		int	set_count = 0;
		int circle_id_count = 0;
		for( int i = 0 ; i < all_circle_states->circles.size() ; i ++ )//for
		{
			if(i == 0){/*std::cout << "-------------------------------------------------------"<< std::endl;*/}
			next_radius = all_circle_states->circles[i+1].radius;
			next_center_x = all_circle_states->circles[i+1].center.x;
			next_center_y = all_circle_states->circles[i+1].center.y;
			double center_distance = hypotf((next_center_x - all_circle_states->circles[i].center.x),(next_center_y - all_circle_states->circles[i].center.y));//過去のサークル同士の中心点の距離
			double radius_distance = next_radius + all_circle_states->circles[i].radius;//隣のサークル同士の半径合計

			if(next_radius != all_circle_states->circles[i].radius && center_distance < radius_distance)//次の隣の円と接触している場合
			{
				//接触している円のステートを保存
				circle_states_save.center.x = all_circle_states->circles[i].center.x;
				circle_states_save.center.y = all_circle_states->circles[i].center.y;
				circle_states_save.radius = all_circle_states->circles[i].radius;
				multiple_circle_list->circles.push_back(circle_states_save);
				set_count++;
			}//if
			else//次の隣の円と接触していない場合 else_1
			{
				if(set_count > 1)	//次の隣の円と比較した時，前の円のステートを更新　書き方が綺麗じゃないので改めて考える
				{
					circle_states_save.center.x = all_circle_states->circles[i].center.x;
					circle_states_save.center.y = all_circle_states->circles[i].center.y;
					circle_states_save.radius = all_circle_states->circles[i].radius;
					multiple_circle_list->circles.push_back(circle_states_save);
					//std::cout << "Multiple_obstacle_ID_2:: " << i << std::endl;
					//ROS_INFO("-------------------------------------------------------");
					double center_point_x_total = 0.0;
					double center_point_y_total = 0.0;
					double max_radius_candidate = 0.0;
					if(multiple_circle_list->circles.size() > 0)
					{
						for(int j = 0;j <= multiple_circle_list->circles.size() ; j++) //接触している複数円のステートを参照 for
						{
							center_point_x_total += multiple_circle_list->circles[j].center.x;
							center_point_y_total += multiple_circle_list->circles[j].center.y;
							if(max_radius_candidate < multiple_circle_list->circles[j].radius)
							{
								max_radius_candidate = multiple_circle_list->circles[j].radius;
							}//if
						}//for
						//値が無限になる場合，小数点3位以下は切り捨て
						double x_correction = floor((center_point_x_total / multiple_circle_list->circles.size()) * 1000) / 1000;
						double y_correction = floor((center_point_y_total / multiple_circle_list->circles.size()) * 1000) / 1000;
						double radius_correction = floor(max_radius_candidate * 1000) / 1000;
						int k = max_radius_candidate * 1000;
						circle_obs.center.x = x_correction;
						circle_obs.center.y = y_correction;
						circle_obs.radius = max_radius_candidate;
						multiple_circle_list->circles.clear();	//マルチサークルが更新され次第，空にする
					}//if
					set_count = 0;
				}//if
				else	//single_obstacleの場合 else_2
				{
					//std::cout << "-------------------------------------------------------"<< std::endl;
					circle_obs.center.x = all_circle_states->circles[i].center.x;
					circle_obs.center.y = all_circle_states->circles[i].center.y;
					circle_obs.radius = all_circle_states->circles[i].radius;	//現在の接触していない円のステートを更新
					//circle_states_2->circles.push_back(circle_obs);
					//std::cout << "single_obstacle_ID:: " << i << std::endl;
					//std::cout << "-------------------------------------------------------"<< std::endl;
				}//else_2
				circle_states_correction->circles.push_back(circle_obs);	//接触している複数円から新たな円のステートを更新
			}//else_1
			//circle_states_2->circles.push_back(circle_2);
		}//for
		//this->pub_circle_states_correction.publish(circle_states_correction);
		//this->pub_circle_states_multiple.publish(multiple_circle_list);
		circle_states_correction->circles.clear();	//空にする
		all_circle_states->circles.clear();	//空にする
		this->points_list.clear();//空にする
	}//decision_of_grouping

void linear_regression()
	{
		int dimension_size = 1;
		float A[group_points_list.size()][dimension_size + 1] = {};	//確認用
		float B[group_points_list.size()][dimension_size] = {};	//確認用
		float A_TRANS[dimension_size + 1][group_points_list.size()] = {};	//確認用
		float A_TRANS_to_A[dimension_size + 1][dimension_size + 1] = {};
		float INVERSE[dimension_size + 1][dimension_size + 1] = {};
		float A_TRANS_to_Y[dimension_size + 1][dimension_size] = {};
		double p_1 = 0.0;
		double p_2 = 0.0;
		double q_1 = 0.0;
		double q_2 = 0.0;
		double y_1 = 0.0;
		double y_2 = 0.0;
		int i = 0;
		int point_c = 0;
		double estimated_error = 0.0;
		auto end_point = this->group_points_list.begin();
		auto begin_point = this->group_points_list.begin();
		//ROS_INFO("group_size:: %f",this->group_lists.grouped_points_array.size());
		if(group_points_list.size() == 1)
		{
			this->correction_values.center_x = begin_point->x;
			this->correction_values.center_y = begin_point->y;
			this->correction_values.radius = 0.1;	//決め打ち
			return;
		}//if

		for(auto point = group_points_list.begin(); point != group_points_list.end(); point++)//for_1
		{
			auto l =  group_points_list.end();
			if(point_c == group_points_list.size()-1) { end_point = point; }//if
			//group_lists.grouped_points_array[group_size].particle_x[point_c] = point->x;
			//group_lists.grouped_points_array[group_size].particle_y[point_c] = point->y;

			point_c++;
			this->all_set.buf.push_back(*point);//1グループ内の点を一つずつpush
			for(int step = 0; step < dimension_size + 1; step++)
			{
				if(step == 0)//0列(行)目の値は1
				{
					A[i][step] = 1.0;	//i行0列目に1.0の数字を入れる

					B[i][step] = point->y;	

					A_TRANS[step][i] = A[i][step];	//0行i列目に1.0の値を入れる

					p_1 += A_TRANS[step][i] * A[i][step];//A[i][step] = 1.0とA_TRANS[step][i] = 1.0が更新された後

					y_1 += A_TRANS[step][i] * B[i][step];

				}//if
				if(step == 1)//1列(行)目の値はX
				{
					A[i][step] = point->x;	//i行1列目にxの値を入れる

					A_TRANS[step][i] = A[i][step];	//1行i列目にxの値を入れる

					p_2 += A_TRANS[step - 1][i] * A[i][step];//A[i][step] = point_in_group->xとA_TRANS[step][i] = 1.0が更新された後

					q_1 += A_TRANS[step][i] * A[i][step - 1]; //A[i][step] = 1.0とA_TRANS[step][i] = point_in_group->xが更新された後

					q_2 += A_TRANS[step][i] * A[i][step];//A[i][step] = point_in_group->xとA_TRANS[step][i] = point_in_group->xが更新された後

					y_2 += A_TRANS[step][i] * B[i][step - 1];
				}//if
			}//for
			i++;
		}//for_1
			A_TRANS_to_A[0][0] = p_1;	//0行0列目
			A_TRANS_to_A[0][1] = p_2;	//0行1列目
			A_TRANS_to_A[1][0] = q_1;	//1行0列目
			A_TRANS_to_A[1][1] = q_2;	//1行1列目
			A_TRANS_to_Y[1][0] = y_2;
			A_TRANS_to_Y[0][0] = y_1;

			double abs_A_TRANS_to_A = 0.0;
			double index_value = 0.0;
			double slope = 0.0;
			double intercept = 0.0;
			abs_A_TRANS_to_A = (p_1 * q_2) - (p_2 * q_1);

		for(int i = 0;i < dimension_size + 1;i++)
		{
			for(int j = 0;j < dimension_size + 1;j++)
			{
				index_value = A_TRANS_to_A[dimension_size - i][dimension_size - j];						
				if(i != j) { index_value = -1.0 * index_value; }
				INVERSE[i][j] = index_value / abs_A_TRANS_to_A;	//逆行列
				if(i == 0) { intercept +=  INVERSE[i][j] * A_TRANS_to_Y[j][i]; }
				else { slope += (floor(INVERSE[i][j] * 1000) / 1000) * (floor(A_TRANS_to_Y[j][i - i] * 1000) / 1000); }
			}//for
		}//for

		point_set intersection_begin;	//開始点
		point_set intersection_end;		//終点	
		geometry_msgs::Point p1;//修正始点
		geometry_msgs::Point p2;//修正終点
		geometry_msgs::Point p3;//始点
		geometry_msgs::Point p4;//終点
		geometry_msgs::Point p5;//開始点(直線の)
		geometry_msgs::Point p6;//終点(直線の)

		intersection_begin.x = (slope * (begin_point->y - intercept) + begin_point->x) / (pow(slope,2) + 1);
		intersection_begin.y = (slope * intersection_begin.x) + intercept;
		intersection_end.x = (slope * (end_point->y - intercept) + end_point->x) / (pow(slope,2) + 1);
		intersection_end.y = (slope * intersection_end.x) + intercept;

		double center_x = ( begin_point->x + end_point->x ) / 2.0;
		double center_y = ( begin_point->y + end_point->y ) / 2.0;
		double radius = hypotf(center_x - begin_point->x,center_y - begin_point->y);

		p1.x = intersection_begin.x;
		p1.y = intersection_begin.y;
		p2.x = intersection_end.x;
		p2.y = intersection_end.y;
		p3.x = begin_point->x;
		p3.y = begin_point->y;
		p4.x = end_point->x;
		p4.y = end_point->y;
		p5.x = (0.0 - intercept) / slope;
		p5.y = (slope * 0.0) + intercept;
		p6.x = (3.0 - intercept) / slope;
		p6.y = (slope * 3.0) + intercept;

		//this->correction_values.center_x = (intersection_begin.x + intersection_end.x) / 2.0;
		//this->correction_values.center_y = (intersection_begin.y + intersection_end.y) / 2.0;
		//this->correction_values.radius = hypotf(intersection_begin.x - intersection_end.x , intersection_begin.y - intersection_end.y) / 2.0;

		this->correction_values.center_x = (begin_point->x + end_point->x) / 2.0;
		this->correction_values.center_y = (begin_point->y + end_point->y) / 2.0;
		this->correction_values.radius = hypotf(begin_point->x - end_point->x , begin_point->y - end_point->y) / 2.0;

		this->all_set.radius = this->base_radius;
		this->all_set.center_radius = this->correction_values.radius;
		this->all_set.center_y = this->correction_values.center_y;
		this->all_set.center_x = this->correction_values.center_x;
	}//linear_regression

void cost_marker(geometry_msgs::Point p1,geometry_msgs::Point p2,geometry_msgs::Point p3,geometry_msgs::Point p4,geometry_msgs::Point p5,geometry_msgs::Point p6)
	{
		visualization_msgs::MarkerArray marker;
		marker.markers.resize(4);
		marker.markers[0].header.frame_id = "base_laser_link";
		marker.markers[0].header.stamp = ros::Time::now();
		marker.markers[0].action = visualization_msgs::Marker::ADD;
		marker.markers[0].type = visualization_msgs::Marker::SPHERE;
		marker.markers[0].ns = "Correction_start_point";
		marker.markers[0].id = 1;
		marker.markers[0].color.a = 1.0;
		marker.markers[0].color.r = 0.0;
		marker.markers[0].color.g = 0.0;
		marker.markers[0].color.b = 1.0;
		marker.markers[0].scale.x = 0.01;//[m]
		marker.markers[0].scale.y = 0.01;//[m]
		marker.markers[0].scale.z = 1.0;//[m]
		marker.markers[0].pose.position.x = p1.x;
		marker.markers[0].pose.position.y = p1.y;
		marker.markers[0].pose.position.z = 0.0;
		marker.markers[0].pose.orientation.x = 0;
		marker.markers[0].pose.orientation.y = 0;
		marker.markers[0].pose.orientation.z = 0;
		marker.markers[0].pose.orientation.w = 1;

		marker.markers[1].header.frame_id = "base_laser_link";
		marker.markers[1].header.stamp = ros::Time::now();
		marker.markers[1].action = visualization_msgs::Marker::ADD;
		marker.markers[1].type = visualization_msgs::Marker::SPHERE;
		marker.markers[1].ns = "Correction_end_point";
		marker.markers[1].id = 2;
		marker.markers[1].color.a = 1.0;
		marker.markers[1].color.r = 0.0;
		marker.markers[1].color.g = 0.0;
		marker.markers[1].color.b = 1.0;
		marker.markers[1].scale.x = 0.01;//[m]
		marker.markers[1].scale.y = 0.01;//[m]
		marker.markers[1].scale.z = 1.0;//[m]
		marker.markers[1].pose.position.x = p2.x;
		marker.markers[1].pose.position.y = p2.y;
		marker.markers[1].pose.position.z = 0.0;
		marker.markers[1].pose.orientation.x = 0;
		marker.markers[1].pose.orientation.y = 0;
		marker.markers[1].pose.orientation.z = 0;
		marker.markers[1].pose.orientation.w = 1;

		marker.markers[2].header.frame_id = "base_laser_link";
		marker.markers[2].header.stamp = ros::Time::now();
		marker.markers[2].action = visualization_msgs::Marker::ADD;
		marker.markers[2].type = visualization_msgs::Marker::SPHERE;
		marker.markers[2].ns = "start_point";
		marker.markers[2].id = 3;
		marker.markers[2].color.a = 1.0;
		marker.markers[2].color.r = 1.0;
		marker.markers[2].color.g = 0.0;
		marker.markers[2].color.b = 0.0;
		marker.markers[2].scale.x = 0.01;//[m]
		marker.markers[2].scale.y = 0.01;//[m]
		marker.markers[2].scale.z = 1.0;//[m]
		marker.markers[2].pose.position.x = p3.x;
		marker.markers[2].pose.position.y = p3.y;
		marker.markers[2].pose.position.z = 0.0;
		marker.markers[2].pose.orientation.x = 0;
		marker.markers[2].pose.orientation.y = 0;
		marker.markers[2].pose.orientation.z = 0;
		marker.markers[2].pose.orientation.w = 1;

		marker.markers[3].header.frame_id = "base_laser_link";
		marker.markers[3].header.stamp = ros::Time::now();
		marker.markers[3].action = visualization_msgs::Marker::ADD;
		marker.markers[3].type = visualization_msgs::Marker::SPHERE;
		marker.markers[3].ns = "end_point";
		marker.markers[3].id = 4;
		marker.markers[3].color.a = 1.0;
		marker.markers[3].color.r = 1.0;
		marker.markers[3].color.g = 0.0;
		marker.markers[3].color.b = 0.0;
		marker.markers[3].scale.x = 0.01;//[m]
		marker.markers[3].scale.y = 0.01;//[m]
		marker.markers[3].scale.z = 1.0;//[m]
		marker.markers[3].pose.position.x = p4.x;
		marker.markers[3].pose.position.y = p4.y;
		marker.markers[3].pose.position.z = 0.0;
		marker.markers[3].pose.orientation.x = 0;
		marker.markers[3].pose.orientation.y = 0;
		marker.markers[3].pose.orientation.z = 0;
		marker.markers[3].pose.orientation.w = 1;
		this->pub_cost_marker.publish( marker );
	}//cost_marker

void test_marker()
{
	int num = 0;
	int group_num = 0;
	visualization_msgs::MarkerArray marker;

	marker.markers.resize(this->points_list.size() );//this->sample_group.size() * 
	for(auto pn = this->sample_group.begin(); pn != this->sample_group.end(); pn++)//for_6 
	{
		auto pa = *pn;
		tp_set option;
		option = pa;
		for(auto state = option.buf.begin(); state != option.buf.end(); state++)//for_7
		{
			marker.markers[num].header.frame_id = "base_laser_link";
			marker.markers[num].header.stamp = ros::Time::now();
			marker.markers[num].action = visualization_msgs::Marker::ADD;
			marker.markers[num].type = visualization_msgs::Marker::CYLINDER;
			marker.markers[num].ns = "points";//_" + std::to_string(group_num);
			marker.markers[num].id = num;
			marker.markers[num].scale.x = option.radius;
			marker.markers[num].scale.y = option.radius;
			marker.markers[num].scale.z = 0.05;
			marker.markers[num].color.a = 0.1;

			if(group_num % 2 == 0)
			{
				marker.markers[num].color.r = 1.0;
				marker.markers[num].color.g = 0.5;
				marker.markers[num].color.b = 0.5;
			}
			else if(group_num % 3 == 1)
			{
				marker.markers[num].color.r = 0.5;
				marker.markers[num].color.g = 1.0;
				marker.markers[num].color.b = 0.5;
			}
			else
			{
				marker.markers[num].color.r = 0.5;
				marker.markers[num].color.g = 0.5;
				marker.markers[num].color.b = 1.0;
			}
			geometry_msgs::Point test;
			test.x = state->x;
			test.y = state->y;
			geometry_msgs::Point p;
			p.x = state->x;
			p.y = state->y;
			p.z = 0.0;
			marker.markers[num].pose.position = p;
			marker.markers[num].pose.orientation.w = 1;

			num++;
		}//for_7
		group_num ++;
	}//for_6
	this->pub_test_marker.publish( marker );
}//test_marker

};//Points_grouping_class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Points_grouping_node");
	Points_grouping_class tf_class;
	ros::spin();
	return 0;
}//main
